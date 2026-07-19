import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('paesano_explorer'),
        'config',
        'explorer.yaml',
    )

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true for simulation time, false for hardware time',
    )
    auto_explore_arg = DeclareLaunchArgument(
        'auto_explore',
        default_value='false',
        description='launch autonomous frontier exploration',
    )
    return LaunchDescription([
        sim_arg,
        auto_explore_arg,
        ComposableNodeContainer(
            name='explorer_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            condition=IfCondition(LaunchConfiguration('auto_explore')),
            composable_node_descriptions=[
                ComposableNode(
                    package='paesano_explorer',
                    plugin='paesano_explorer::ExplorerNode',
                    name='explorer_node',
                    parameters=[
                        config_file,
                        {
                            'use_sim_time': ParameterValue(
                                LaunchConfiguration('sim'), value_type=bool),
                        },
                    ],
                ),
            ],
        ),
    ])
