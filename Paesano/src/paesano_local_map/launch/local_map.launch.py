import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Define parameters for the local map node
    local_map_params = {
        'scan_topic': '/scan',
        'pose_topic': '/estimated_pose',
        'map_topic': '/local_map',
        'map_size_m': 2.0,
        'map_resolution': 0.05,  # 5cm per cell
        'scan_yaw_offset': -1.57079632679,
    }

    # Configure the Local Map Node execution
    local_map_node = Node(
        package='paesano_local_map',
        executable='local_map_node',  # Make sure this matches your CMakeLists.txt target name
        name='local_map_node',
        output='screen',
        parameters=[local_map_params]
    )

    return LaunchDescription([
        local_map_node
    ])
