from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(
                    ['xacro ', 
                     '/home/slambot/ros2_ws/src/slambot_description/urdf/slambot.urdf']
                )
            }]
        ),
    ])