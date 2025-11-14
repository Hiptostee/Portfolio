from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path to your package
    pkg_path = get_package_share_directory('slambot_description')
    # Use the Xacro file (preferred)
    xacro_file = os.path.join(pkg_path, 'urdf', 'slambot.xacro')

    # Convert XACRO → URDF text for robot_description
    # Expand Xacro into URDF text for robot_state_publisher
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Start Gazebo (Ignition / Gazebo Harmonic)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    # Robot State Publisher (Publishes TF + URDF)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    # Spawn robot into Gazebo
    # Spawn the robot into Gazebo using the expanded Xacro string
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'slambot',
            '-string', Command(['xacro ', xacro_file])
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp_node,
        spawn_robot
    ])
