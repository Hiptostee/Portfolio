from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('mecanum_drive_controller'),
        'config',
        'mecanum_drive_controller.yaml',
    )

    return LaunchDescription([
        Node(
            package='mecanum_drive_controller',
            executable='mecanum_drive_controller_exec',
            name='mecanum_drive_controller',
            parameters=[config_file],
        ),
    ])
