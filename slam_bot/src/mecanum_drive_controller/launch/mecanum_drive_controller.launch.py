from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_drive_controller',
            executable='mecanum_drive_controller_exec',
            name='mecanum_drive_controller',
        ),
    ])
