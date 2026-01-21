from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
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
    imu_input_topic = LaunchConfiguration('imu_input_topic')

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
        launch_arguments={'imu_input_topic': imu_input_topic}.items()
    )

    # -------------------------------------------------------------------------
    # Return everything
    # -------------------------------------------------------------------------
    return LaunchDescription([
        robot_state_publisher,
        imu_input_arg,
        imu_node,
        lidar_node,
        mecanum_drive_launch_node,
        odom_node_launch_node,
        localization_node_launch_node,
    ])
