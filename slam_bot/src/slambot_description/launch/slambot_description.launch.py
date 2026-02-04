from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('slambot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'slambot.xacro')
    
    odom_node_launch = os.path.join(
        get_package_share_directory('odom_node'),
        'launch',
        'odom_node.launch.py'
    )

    ekf_launch = os.path.join(
        get_package_share_directory('slambot_localization'),
        'launch',
        'slambot_localization.launch.py'
    )

    slam_toolbox_launch = os.path.join(
        get_package_share_directory('slambot_slam_toolbox'),
        'launch',
        'slambot_slam_toolbox.launch.py'
    )

    # Expand XACRO → URDF
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file, ' ',
            'use_gz:=true'
        ]),
        value_type=str
    )



    # Start Gazebo with ODE world for mecanum friction modeling
    world_file = os.path.join(pkg_path, 'worlds', 'mecanum_ode.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Spawn robot in Gazebo (same xacro)
    # Wrap in TimerAction to wait for Gazebo Sim to be ready
    spawn_robot_delayed = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world', 'mecanum_ode',
                    '-name', 'slambot',
                    '-string', Command([
                        FindExecutable(name='xacro'), ' ', xacro_file, ' ',
                        'use_gz:=true'
                    ])
                ],
                output='screen'
            )
        ]
    )

    # Static TF alias for Gazebo Sim laser frame → URDF frame
    # Gazebo Sim LaserScan often uses model/link/sensor naming for frame_id.
    # Create an identity alias so transforms to base_link work.
    lidar_frame_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # Alias to bridge Gazebo Sim scan frame naming to URDF frame naming
        arguments=['0', '0', '0', '0', '0', '0', 'base_laser', 'slambot/base_link/base_laser'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # No ros2_control spawners needed in kinematic mode

    # Ensure odom node uses sim time when running in Gazebo
    odom_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odom_node_launch),
        launch_arguments={'sim': 'true'}.items()
    )

    ekf_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch),
        launch_arguments={'sim': 'true'}.items()
    )
    slam_toolbox_launch_ = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch),
        launch_arguments={'sim': 'true'}.items()
    )



    bridge_yaml = DeclareLaunchArgument(
        "bridge_yaml",
        default_value=os.path.join(pkg_path, "config", "bridge.yaml"),
        description="bridge YAML config",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("bridge_yaml")}],
    )

    return LaunchDescription([
        bridge_yaml,
        gazebo,
        rsp,
        spawn_robot_delayed,
        lidar_frame_alias,
        odom_node_launch,
        ekf_node_launch,
        slam_toolbox_launch_,
        bridge,
    ])
