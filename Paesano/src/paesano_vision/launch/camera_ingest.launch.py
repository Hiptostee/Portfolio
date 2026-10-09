import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
    )
    config_file = os.path.join(
        get_package_share_directory('paesano_vision'), 'config', 'camera_ingest.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_sync': 'true',
                'align_depth.enable': 'true',
                'pointcloud.enable': 'false',
            }.items(),
        ),
        ComposableNodeContainer(
            name='camera_ingest_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='paesano_vision',
                    plugin='paesano_vision::CameraIngest',
                    name='camera_ingest',
                    parameters=[config_file],
                ),
            ],
        ),
    ])
