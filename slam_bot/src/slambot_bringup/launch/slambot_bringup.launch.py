from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
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

    robot_description_file = os.path.join(
        get_package_share_directory('slambot_description'),
        'urdf',
        'slambot.urdf'
    )

    # -------------------------------------------------------------------------
    # robot_state_publisher (URDF)
    # -------------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(
                ['xacro ', robot_description_file]
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

    # -------------------------------------------------------------------------
    # Return everything
    # -------------------------------------------------------------------------
    return LaunchDescription([
        robot_state_publisher,
        imu_node,
        lidar_node
    ])