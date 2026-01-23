from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odom_node',
            executable='odom_node_exec',
            name='odom_node',
            parameters=[{'use_sim_time': True}],
        ),
    ])
