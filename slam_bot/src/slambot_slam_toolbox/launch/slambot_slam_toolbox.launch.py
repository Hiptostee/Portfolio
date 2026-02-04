from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    slam_params = os.path.join(
        get_package_share_directory('slambot_slam_toolbox'),
        'config',
        'slam_toolbox.yaml'
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true for sim time (/clock), false for hardware time'
    )

    return LaunchDescription([
        sim_arg,
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params,
                {'use_sim_time': LaunchConfiguration('sim')},
            ],
        )
    ])
