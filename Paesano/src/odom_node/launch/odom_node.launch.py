from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('odom_node'),
        'config',
        'odom.yaml',
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true for sim time (/clock), false for hardware time',
    )

    sim = LaunchConfiguration('sim')

    return LaunchDescription([
        sim_arg,
        Node(
            package='odom_node',
            executable='odom_node_exec',
            name='odom_node',
            parameters=[
                config_file,
                {'use_sim_time': ParameterValue(sim, value_type=bool)},
            ],
        ),
    ])
