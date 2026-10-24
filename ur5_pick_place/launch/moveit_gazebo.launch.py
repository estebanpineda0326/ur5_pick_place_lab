"""
Lanza move_group + RViz (plugin MotionPlanning) conectados a la simulación
de Gazebo que ya está corriendo (sim_gz_sgb.launch.py launch_rviz:=false).

Requiere el paquete ur5_robotiq_moveit generado con el MoveIt Setup Assistant.
No usa demo.launch.py porque ese lanza su propio ros2_control con hardware mock.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

MOVEIT_PKG = 'ur5_robotiq_moveit'


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder('ur', package_name=MOVEIT_PKG).to_moveit_configs()

    sim_time = {'use_sim_time': True}
    # Gazebo nunca deja las articulaciones exactamente en el objetivo: se amplía la
    # tolerancia de estado inicial para evitar "start point deviates from current robot state".
    execution = {
        'trajectory_execution.allowed_start_tolerance': 0.05,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
    }

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), sim_time, execution],
    )

    rviz_config = os.path.join(get_package_share_directory(MOVEIT_PKG), 'config', 'moveit.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            sim_time,
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        move_group,
        rviz,
    ])
