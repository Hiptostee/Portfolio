from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode',
        default_value='false',
        description='true enables particle filter node'
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/slambot/ros2_ws/maps/my_map.yaml',
        description='Initial map yaml for nav2 map_server',
    )
    imu_input_topic = LaunchConfiguration('imu_input_topic')
    sim = LaunchConfiguration('sim')
    localization_mode = LaunchConfiguration('localization_mode')
    map_yaml = LaunchConfiguration('map_yaml')

    return LaunchDescription([
        imu_input_arg,
        sim_arg,
        localization_mode_arg,
        map_yaml_arg,

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

        # -------------------------
        # Particle filter (HW)
        # -------------------------
        Node(
            package='slambot_localization',
            executable='slambot_localization_ParticleFilter',
            name='particle_filter',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(sim, value_type=bool),
            }],
            condition=IfCondition(localization_mode),
        ),

    
        # -------------------------
        # Map loader bridge service
        # -------------------------
        Node(
            package='slambot_localization',
            executable='slambot_localization_MapLoaderService',
            name='map_loader_service',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(sim, value_type=bool),
                'map_server_load_service': '/map_server/load_map',
                'global_load_map_service': '/slambot/load_map',
                'load_map_service': '/slambot/load_map',
                'map_topic': '/map',
            }],
        ),

        # -------------------------
        # Nav2 map server (for /map_server/load_map)
        # -------------------------
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(sim, value_type=bool),
                'yaml_filename': map_yaml,
            }],
            condition=IfCondition(localization_mode),
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(sim, value_type=bool),
                'autostart': True,
                'node_names': ['map_server'],
            }],
            condition=IfCondition(localization_mode),
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
