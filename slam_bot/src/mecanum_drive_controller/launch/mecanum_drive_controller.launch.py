from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package='mecanum_drive_controller',
        executable='mecanum_drive_controller_node',
        name='mecanum_drive_controller',
        output='screen',
        parameters=[{
            'wheel_radius': 0.0485,
            'base_length': 0.297,
            'base_width': 0.256,
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([node])

