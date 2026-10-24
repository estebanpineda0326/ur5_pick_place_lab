from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cycles = LaunchConfiguration('cycles')
    auto_inputs = LaunchConfiguration('auto_inputs')

    params_file = PathJoinSubstitution(
        [FindPackageShare('ur5_pick_place'), 'config', 'pick_place.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('cycles', default_value='1',
                              description='Número de ciclos (0 = infinito)'),
        DeclareLaunchArgument('auto_inputs', default_value='false',
                              description='true: las entradas digitales se liberan solas'),
        Node(
            package='ur5_pick_place',
            executable='pick_place_node',
            name='pick_place_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'use_sim_time': True,
                    'cycles': ParameterValue(cycles, value_type=int),
                    'auto_inputs': ParameterValue(auto_inputs, value_type=bool),
                },
            ],
        ),
    ])
