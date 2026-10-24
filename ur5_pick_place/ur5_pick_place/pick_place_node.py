#!/usr/bin/env python3
"""
Nodo de simulación de una rutina Pick & Place para el UR5 + gripper simple.

- Brazo:   acción /scaled_joint_trajectory_controller/follow_joint_trajectory
           (control_msgs/action/FollowJointTrajectory)
- Gripper: tópico /gripper_controller/commands (std_msgs/msg/Float64MultiArray)
- Entradas digitales simuladas:  /io/di0, /io/di1, /io/di2   (std_msgs/msg/Bool)
- Salidas digitales simuladas:   /io/do0, /io/do1, /io/do2   (std_msgs/msg/Bool)
- Estado de la rutina:           /pick_place/status          (std_msgs/msg/String)

Para liberar una espera de entrada digital, desde otra terminal:
    ros2 topic pub --once /io/di0 std_msgs/msg/Bool "{data: true}"
"""

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Bool, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Puntos de interés (radianes). Verifícalos en RViz/Gazebo y ajústalos si hace falta.
# Con wrist_1 = -pi/2 - shoulder_lift - elbow la herramienta queda apuntando hacia abajo.
DEFAULT_POSES = {
    'home':           [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
    'pick_approach':  [0.6, -1.50, 1.40, -1.47, -1.57, 0.6],
    'pick':           [0.6, -1.30, 1.60, -1.87, -1.57, 0.6],
    'place_approach': [-0.6, -1.50, 1.40, -1.47, -1.57, -0.6],
    'place':          [-0.6, -1.30, 1.60, -1.87, -1.57, -0.6],
}

# Rutina: lista de pasos (tipo, argumento[, valor]). Cada punto de interés tiene
# al menos una operación: gripper, mensaje, espera de DI o activación de DO.
ROUTINE = [
    # --- Punto 1: HOME ---------------------------------------------------
    ('move', 'home'),
    ('info', 'P1 HOME: robot en posición inicial.'),
    ('gripper', 'open'),
    ('set_do', 0, True),    # DO0 = robot listo (lámpara)
    ('wait_di', 0),         # DI0 = botón de inicio de ciclo
    ('set_do', 0, False),
    # --- Punto 2: PICK ---------------------------------------------------
    ('move', 'pick_approach'),
    ('info', 'P2 PICK: aproximando a la zona de recogida.'),
    ('wait_di', 1),         # DI1 = sensor de pieza presente
    ('move', 'pick'),
    ('gripper', 'close'),
    ('set_do', 1, True),    # DO1 = pieza sujeta
    ('move', 'pick_approach'),
    # --- Punto 3: PLACE --------------------------------------------------
    ('move', 'place_approach'),
    ('info', 'P3 PLACE: verificando zona de entrega libre.'),
    ('wait_di', 2),         # DI2 = zona de entrega libre
    ('move', 'place'),
    ('gripper', 'open'),
    ('set_do', 1, False),
    ('move', 'place_approach'),
    # --- Regreso a HOME y fin de ciclo ----------------------------------
    ('move', 'home'),
    ('pulse_do', 2),        # DO2 = pulso de ciclo terminado
    ('info', 'Ciclo Pick & Place terminado.'),
]


class PickPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_place_node')

        self.declare_parameter('cycles', 1)              # 0 = infinito
        self.declare_parameter('move_time', 3.0)         # s por movimiento
        self.declare_parameter('gripper_open', [0.04, 0.04])
        self.declare_parameter('gripper_closed', [0.004, 0.004])
        self.declare_parameter('gripper_wait', 1.5)      # s tras comandar el gripper
        self.declare_parameter('pulse_time', 1.0)        # s de duración de un pulso DO
        self.declare_parameter('auto_inputs', False)     # True: DI se liberan solas
        self.declare_parameter('auto_input_delay', 2.0)  # s de espera si auto_inputs

        for name, values in DEFAULT_POSES.items():
            self.declare_parameter(f'poses.{name}', values)

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)
        self.status_pub = self.create_publisher(String, '/pick_place/status', 10)

        # Salidas: transient_local para que un `ros2 topic echo` tardío vea el último estado.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.do_pubs = [self.create_publisher(Bool, f'/io/do{i}', latched) for i in range(3)]
        for i in range(3):
            self.do_pubs[i].publish(Bool(data=False))

        self.di_state = [False, False, False]
        self.di_lock = threading.Lock()
        for i in range(3):
            self.create_subscription(Bool, f'/io/di{i}', self._make_di_cb(i), 10)

    # ------------------------------------------------------------------ E/S
    def _make_di_cb(self, idx):
        def cb(msg):
            with self.di_lock:
                self.di_state[idx] = msg.data
            self.get_logger().info(f'DI{idx} <- {msg.data}')
        return cb

    def log(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def set_do(self, idx, value):
        self.do_pubs[idx].publish(Bool(data=value))
        self.log(f'DO{idx} -> {"ON" if value else "OFF"}')

    def pulse_do(self, idx):
        self.set_do(idx, True)
        time.sleep(self.get_parameter('pulse_time').value)
        self.set_do(idx, False)

    def wait_di(self, idx):
        if self.get_parameter('auto_inputs').value:
            delay = self.get_parameter('auto_input_delay').value
            self.log(f'Esperando DI{idx} (auto en {delay:.1f} s)...')
            time.sleep(delay)
            return
        self.log(f'Esperando DI{idx} = true  ->  '
                 f'ros2 topic pub --once /io/di{idx} std_msgs/msg/Bool "{{data: true}}"')
        while rclpy.ok():
            with self.di_lock:
                if self.di_state[idx]:
                    self.di_state[idx] = False   # se consume: el siguiente ciclo pide otro flanco
                    break
            time.sleep(0.05)
        self.log(f'DI{idx} recibida, continuando.')

    # -------------------------------------------------------------- Gripper
    def gripper(self, state):
        key = 'gripper_open' if state == 'open' else 'gripper_closed'
        data = list(self.get_parameter(key).value)
        self.gripper_pub.publish(Float64MultiArray(data=data))
        self.log(f'Gripper -> {state.upper()} {data}')
        time.sleep(self.get_parameter('gripper_wait').value)

    # ---------------------------------------------------------------- Brazo
    def move(self, pose_name):
        positions = list(self.get_parameter(f'poses.{pose_name}').value)
        t = float(self.get_parameter('move_time').value)

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        goal.trajectory.points = [point]

        self.log(f'Moviendo a "{pose_name}" {positions}')
        send_future = self.arm_client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            time.sleep(0.02)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f'Meta rechazada por el controlador del brazo ({pose_name})')

        result_future = handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.02)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f'Fallo moviendo a {pose_name}: código {result.error_code} {result.error_string}')

    # --------------------------------------------------------------- Rutina
    def run(self):
        self.log('Esperando servidor de acción del brazo...')
        if not self.arm_client.wait_for_server(timeout_sec=30.0):
            self.log('No se encontró /scaled_joint_trajectory_controller/follow_joint_trajectory. '
                     '¿Está corriendo la simulación?')
            return

        cycles = int(self.get_parameter('cycles').value)
        n = 0
        while rclpy.ok() and (cycles == 0 or n < cycles):
            n += 1
            self.log(f'===== Ciclo {n} =====')
            for step in ROUTINE:
                kind = step[0]
                if kind == 'move':
                    self.move(step[1])
                elif kind == 'gripper':
                    self.gripper(step[1])
                elif kind == 'info':
                    self.log(step[1])
                elif kind == 'wait_di':
                    self.wait_di(step[1])
                elif kind == 'set_do':
                    self.set_do(step[1], step[2])
                elif kind == 'pulse_do':
                    self.pulse_do(step[1])
        self.log('Rutina finalizada.')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
    except (KeyboardInterrupt, RuntimeError) as exc:
        node.get_logger().error(str(exc))
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
