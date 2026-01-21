from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Resolve ekf.yaml from the installed package share directory (portable)
    ekf_param_file = os.path.join(
        get_package_share_directory('slambot_localization'),
        'config',
        'ekf.yaml',
    )

    # Allow overriding IMU input topic (e.g., /imu vs /imu/data)
    imu_input_arg = DeclareLaunchArgument(
        'imu_input_topic',
        default_value='/imu',
        description='Raw IMU topic to read from',
    )

    imu_input_topic = LaunchConfiguration('imu_input_topic')

    return LaunchDescription([
        imu_input_arg,
        # IMU covariance shaping node
        Node(
            package='slambot_localization',
            executable='slambot_localization_CovariancesOnImu',
            name='slambot_localization',
            parameters=[{
                'imu_input_topic': imu_input_topic,
                'imu_output_topic': '/imu_with_covariances',
            }],
        ),
        # EKF fuser
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_param_file],
            remappings=[('odometry/filtered', '/odom/filtered')],
        ),
    ])
