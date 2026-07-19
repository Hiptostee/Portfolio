from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    slam_params = os.path.join(
        get_package_share_directory('paesano_mapping'),
        'config',
        'slam_toolbox.yaml'
    )
    mapping_pose_params = os.path.join(
        get_package_share_directory('paesano_mapping'),
        'config',
        'mapping_pose.yaml'
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true for sim time (/clock), false for hardware time'
    )
    auto_explore_arg = DeclareLaunchArgument(
        'auto_explore',
        default_value='false',
        description='publish map-frame pose for autonomous exploration'
    )

    slam_toolbox_online_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_sync_launch.py'
    )

    return LaunchDescription([
        sim_arg,
        auto_explore_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_online_launch),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time': LaunchConfiguration('sim'),
            }.items()
        ),
        ComposableNodeContainer(
            name='mapping_pose_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            condition=IfCondition(LaunchConfiguration('auto_explore')),
            composable_node_descriptions=[
                ComposableNode(
                    package='paesano_mapping',
                    plugin='paesano_mapping::MappingPosePublisher',
                    name='mapping_pose_publisher',
                    parameters=[
                        mapping_pose_params,
                        {
                            'use_sim_time': ParameterValue(
                                LaunchConfiguration('sim'), value_type=bool),
                        },
                    ],
                ),
            ],
        ),
    ])
