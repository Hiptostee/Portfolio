from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    ekf_param_file = os.path.join(
        get_package_share_directory('slambot_localization'),
        'config',
        'ekf.yaml',
    )

    imu_input_arg = DeclareLaunchArgument(
        'imu_input_topic',
        default_value='/imu',
        description='Raw IMU topic to read from',
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true for sim time (/clock), false for hardware time'
    )

    imu_input_topic = LaunchConfiguration('imu_input_topic')
    sim = LaunchConfiguration('sim')

    return LaunchDescription([
        imu_input_arg,
        sim_arg,

        # -------------------------
        # IMU covariance node (HW)
        # -------------------------
        Node(
            package='slambot_localization',
            executable='slambot_localization_CovariancesOnImu',
            name='imu_covariances',
            parameters=[{
                'imu_input_topic': imu_input_topic,
                'imu_output_topic': '/imu_with_covariances',
                'use_sim_time': False,
            }],
            condition=UnlessCondition(sim),
        ),

        # -------------------------
        # IMU covariance node (SIM)
        # -------------------------
        Node(
            package='slambot_localization',
            executable='slambot_localization_CovariancesOnImu',
            name='imu_covariances',
            parameters=[{
                'imu_input_topic': imu_input_topic,
                'imu_output_topic': '/imu_with_covariances',
                'use_sim_time': True,
            }],
            condition=IfCondition(sim),
        ),

        # ---------------
        # EKF (HW)
        # ---------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_param_file,
                {'use_sim_time': False},
            ],
            remappings=[('odometry/filtered', '/odom/filtered')],
            condition=UnlessCondition(sim),
        ),

        # ---------------
        # EKF (SIM)
        # ---------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_param_file,
                {'use_sim_time': True},
            ],
            remappings=[('odometry/filtered', '/odom/filtered')],
            condition=IfCondition(sim),
        ),
    ])