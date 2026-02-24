from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
import os

def generate_launch_description():
    # -------------------------------------------------------------------------
    # Paths to other launch files
    # -------------------------------------------------------------------------
    ros2_mpu6050_launch = os.path.join(
        get_package_share_directory('bno08x_driver'),
        'launch',
        'bno085_i2c.launch.py'
    )

    ldlidar_launch = os.path.join(
        get_package_share_directory('ldlidar'),
        'launch',
        'ld19.launch.py'
    )

    mecanum_drive_launch = os.path.join(
        get_package_share_directory('mecanum_drive_controller'),
        'launch',
        'mecanum_drive_controller.launch.py'
    )

    odom_node_launch = os.path.join(
        get_package_share_directory('odom_node'),
        'launch',
        'odom_node.launch.py'
    )

    localization_node_launch = os.path.join(
        get_package_share_directory('slambot_localization'),
        'launch',
        'slambot_localization.launch.py'
    )

    mapping_launch = os.path.join(
        get_package_share_directory('slambot_mapping'),
        'launch',
        'mapping.launch.py'
    )

    robot_description_file = os.path.join(
        get_package_share_directory('slambot_description'),
        'urdf',
        'slambot.urdf'
    )

    # IMU topic plumbed through to localization (hardware may use /imu/data)
    imu_input_arg = DeclareLaunchArgument(
        'imu_input_topic',
        default_value='/imu',
        description='Raw IMU topic (e.g., /imu or /imu/data)'
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true for sim time (/clock), false for hardware time'
    )
    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode',
        default_value='false',
        description='false: mapping with slam_toolbox, true: localization with particle filter'
    )
    map_yaml_default = PythonExpression([
        "'/home/slambot/ros2_ws/maps/my_map.yaml' if '",
        LaunchConfiguration('sim'),
        "' in ['true', 'True', '1'] else '/home/slambot/ros2_ws_pi/maps/my_map.yaml'"
    ])
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=map_yaml_default,
        description='Map yaml used by localization mode'
    )
    imu_input_topic = LaunchConfiguration('imu_input_topic')
    sim = LaunchConfiguration('sim')
    localization_mode = LaunchConfiguration('localization_mode')
    map_yaml = LaunchConfiguration('map_yaml')

    # -------------------------------------------------------------------------
    # robot_state_publisher (URDF)
    # -------------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', robot_description_file]),
                value_type=str
            )
        }],
        output='screen'
    )

   
    # -------------------------------------------------------------------------
    # Include IMU + LiDAR drivers
    # -------------------------------------------------------------------------
    imu_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_mpu6050_launch)
    )

    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ldlidar_launch)
    )

    mecanum_drive_launch_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mecanum_drive_launch)
    )

    odom_node_launch_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(odom_node_launch)
    )

    localization_node_launch_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_node_launch),
        launch_arguments={
            'imu_input_topic': imu_input_topic,
            'sim': sim,
            'localization_mode': localization_mode,
            'map_yaml': map_yaml,
        }.items(),
    )

    mapping_launch_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch),
        launch_arguments={'sim': sim}.items(),
        condition=UnlessCondition(localization_mode)
    )

    # -------------------------------------------------------------------------
    # Return everything
    # -------------------------------------------------------------------------
    return LaunchDescription([
        robot_state_publisher,
        imu_input_arg,
        sim_arg,
        localization_mode_arg,
        map_yaml_arg,
        imu_node,
        lidar_node,
        mecanum_drive_launch_node,
        odom_node_launch_node,
        mapping_launch_node,
        localization_node_launch_node,
    ])
