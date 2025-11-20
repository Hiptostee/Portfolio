from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('slambot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'slambot.xacro')
    
    # Expand XACRO → URDF (ros2_control params default is set in the xacro)
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file, ' ', 'use_gz:=true']),
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
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'slambot',
            '-string', Command([FindExecutable(name='xacro'), ' ', xacro_file])
        ],
        output='screen'
    )

    # Spawners talk to Gazebo's controller manager (/controller_manager)
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output='screen'
    )

    wheel_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_controller",
            "-c", "/controller_manager"
        ],
        output='screen'
    )

    # Delay controller loading until robot is spawned
    delay = TimerAction(
        period=5.0,
        actions=[jsb_spawner, wheel_spawner]
    )

    # Mecanum controller node (translates /cmd_vel to joint velocities)
    mecanum_controller = Node(
        package='mecanum_drive_controller',
        executable='mecanum_drive_controller_node',
        name='mecanum_drive_controller',
        output='screen',
        parameters=[{
            'wheel_radius': 0.0485,
            'base_length': 0.297,
            'base_width': 0.256,
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_robot,
        delay,
        mecanum_controller,
    ])
