from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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

    slam_toolbox_online_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    return LaunchDescription([
        sim_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_online_launch),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time': LaunchConfiguration('sim'),
            }.items()
        ),
    ])
