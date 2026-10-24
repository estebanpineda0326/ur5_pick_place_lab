#!/usr/bin/env python3
"""
Nodo que usa el ecosistema MoveIt 2 para planificar y ejecutar trayectorias
sobre la simulación en Gazebo.

Habla directamente con move_group a través de la acción /move_action
(moveit_msgs/action/MoveGroup), así que no necesita bindings extra
(moveit_py / MoveGroupInterface) y funciona con cualquier paquete
generado por el MoveIt Setup Assistant.

Secuencia:
  1. Añade un piso como objeto de colisión en la escena de planificación.
  2. Mueve el grupo 'ur_arm' a un objetivo articular (home).
  3. Mueve 'tool0' a un objetivo cartesiano (pose) con la herramienta hacia abajo.
  4. Planifica (solo planifica) el grupo 'gripper' y lo acciona por tópico,
     porque el gripper simulado es un JointGroupPositionController sin acción.
  5. Mueve el brazo a un segundo objetivo articular y regresa a home.
"""

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, CollisionObject, Constraints,
                             JointConstraint, MoveItErrorCodes,
                             MotionPlanRequest, OrientationConstraint,
                             PlanningScene, PositionConstraint)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray

ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
GRIPPER_JOINTS = ['gripper_finger_joint', 'right_finger_joint']

HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
SIDE = [1.2, -1.30, 1.60, -1.87, -1.57, 1.2]


class MoveItGoalNode(Node):

    def __init__(self):
        super().__init__('moveit_goal_node')
        self.declare_parameter('arm_group', 'ur_arm')
        self.declare_parameter('gripper_group', 'gripper')
        self.declare_parameter('planning_frame', 'world')
        self.declare_parameter('ee_link', 'tool0')
        self.declare_parameter('velocity_scaling', 0.3)
        self.declare_parameter('acceleration_scaling', 0.3)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('add_floor', True)
        # Objetivo cartesiano de tool0 en el frame 'world' (m)
        self.declare_parameter('target_xyz', [0.40, 0.20, 0.35])

        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)

    # ------------------------------------------------------------ helpers
    def p(self, name):
        return self.get_parameter(name).value

    def _wait(self, future):
        while rclpy.ok() and not future.done():
            time.sleep(0.02)
        return future.result()

    def add_floor(self):
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2.0, 2.0, 0.02])
        pose = Pose()
        pose.position.z = -0.02          # cara superior en z = -0.01
        pose.orientation.w = 1.0
        obj = CollisionObject()
        obj.header.frame_id = self.p('planning_frame')
        obj.id = 'floor'
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [obj]
        for _ in range(50):               # espera a que move_group se suscriba
            if self.scene_pub.get_subscription_count() > 0:
                break
            time.sleep(0.1)
        self.scene_pub.publish(scene)
        self.get_logger().info('Piso añadido a la escena de planificación.')
        time.sleep(0.5)

    def _base_request(self, group):
        req = MotionPlanRequest()
        req.group_name = group
        req.num_planning_attempts = 10
        req.allowed_planning_time = float(self.p('planning_time'))
        req.max_velocity_scaling_factor = float(self.p('velocity_scaling'))
        req.max_acceleration_scaling_factor = float(self.p('acceleration_scaling'))
        req.start_state.is_diff = True
        ws = req.workspace_parameters
        ws.header.frame_id = self.p('planning_frame')
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -2.0
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 2.0
        return req

    def joint_goal(self, group, joints, values):
        c = Constraints(name='joint_goal')
        for j, v in zip(joints, values):
            c.joint_constraints.append(JointConstraint(
                joint_name=j, position=float(v),
                tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        req = self._base_request(group)
        req.goal_constraints = [c]
        return req

    def pose_goal(self, group, link, xyz, quat_xyzw):
        frame = self.p('planning_frame')
        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = link
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])
        center = Pose()
        center.position.x, center.position.y, center.position.z = xyz
        center.orientation.w = 1.0
        pc.constraint_region = BoundingVolume(primitives=[sphere], primitive_poses=[center])
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = link
        (oc.orientation.x, oc.orientation.y,
         oc.orientation.z, oc.orientation.w) = quat_xyzw
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0

        req = self._base_request(group)
        req.goal_constraints = [Constraints(name='pose_goal',
                                            position_constraints=[pc],
                                            orientation_constraints=[oc])]
        return req

    def execute(self, req, label, plan_only=False):
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        mode = 'Planificando' if plan_only else 'Planificando y ejecutando'
        self.get_logger().info(f'{mode}: {label}')
        handle = self._wait(self.client.send_goal_async(goal))
        if handle is None or not handle.accepted:
            self.get_logger().error(f'move_group rechazó la meta: {label}')
            return False
        result = self._wait(handle.get_result_async()).result
        ok = result.error_code.val == MoveItErrorCodes.SUCCESS
        n_pts = len(result.planned_trajectory.joint_trajectory.points)
        self.get_logger().info(
            f'{label}: {"OK" if ok else "FALLÓ"} (código {result.error_code.val}, '
            f'{n_pts} puntos, planificado en {result.planning_time:.3f} s)')
        return ok

    def gripper(self, values, label):
        self.gripper_pub.publish(Float64MultiArray(data=[float(v) for v in values]))
        self.get_logger().info(f'Gripper -> {label} {values}')
        time.sleep(1.5)

    # ------------------------------------------------------------- rutina
    def run(self):
        self.get_logger().info('Esperando la acción /move_action de move_group...')
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('move_group no está disponible.')
            return
        if self.p('add_floor'):
            self.add_floor()

        arm, grip, ee = self.p('arm_group'), self.p('gripper_group'), self.p('ee_link')
        down = (1.0, 0.0, 0.0, 0.0)   # rotación de 180° en X: tool0 apunta hacia -Z

        self.execute(self.joint_goal(arm, ARM_JOINTS, HOME), 'brazo -> HOME (articular)')
        self.execute(self.pose_goal(arm, ee, list(self.p('target_xyz')), down),
                     f'tool0 -> {list(self.p("target_xyz"))} (cartesiano)')

        # Grupo gripper: MoveIt lo planifica, la ejecución va por el tópico del controlador.
        self.execute(self.joint_goal(grip, GRIPPER_JOINTS, [0.004, 0.004]),
                     'gripper -> CLOSE', plan_only=True)
        self.gripper([0.004, 0.004], 'CLOSE')

        self.execute(self.joint_goal(arm, ARM_JOINTS, SIDE), 'brazo -> SIDE (articular)')
        self.execute(self.joint_goal(grip, GRIPPER_JOINTS, [0.04, 0.04]),
                     'gripper -> OPEN', plan_only=True)
        self.gripper([0.04, 0.04], 'OPEN')
        self.execute(self.joint_goal(arm, ARM_JOINTS, HOME), 'brazo -> HOME (articular)')
        self.get_logger().info('Secuencia MoveIt terminada.')


def main(args=None):
    rclpy.init(args=args)
    node = MoveItGoalNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
