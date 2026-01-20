from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('slambot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'slambot.xacro')
    
    # Kinematic toggle (use kinematic base plugin in Gazebo Sim)
    kinematic = LaunchConfiguration('kinematic')
    declare_kinematic = DeclareLaunchArgument(
        'kinematic', default_value='true', description='Use kinematic base plugin in Gazebo Sim'
    )

    odom_node_launch = os.path.join(
        get_package_share_directory('odom_node'),
        'launch',
        'odom_node.launch.py'
    )

    covariances_on_imu_launch = os.path.join(
        get_package_share_directory('slambot_localization'),
        'launch',
        'slambot_localization.launch.py'
    )

    # Expand XACRO → URDF (ros2_control params default is set in the xacro)
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file, ' ',
            'use_gz:=true', ' ',
            'kinematic:=', kinematic
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
                        'use_gz:=true', ' ',
                        'kinematic:=', kinematic
                    ])
                ],
                output='screen'
            )
        ]
    )

    # Spawners talk to Gazebo's controller manager (/controller_manager)
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    wheel_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_controller",
            "-c", "/controller_manager"
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=UnlessCondition(kinematic)
    )

    # Delay controller loading until robot is spawned (after spawn delay + buffer)
    delay = TimerAction(
        period=8.0,
        actions=[jsb_spawner, wheel_spawner]
    )

    odom_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odom_node_launch)
    )

    covariances_on_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(covariances_on_imu_launch)
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
        declare_kinematic,
        bridge_yaml,
        gazebo,
        rsp,
        spawn_robot_delayed,
        delay,
        odom_node_launch,
        covariances_on_imu_launch,
        bridge,
    ])
