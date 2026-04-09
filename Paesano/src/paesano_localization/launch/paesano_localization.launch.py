from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    ekf_param_file = os.path.join(
        get_package_share_directory('paesano_localization'),
        'config',
        'ekf.yaml',
    )
    localization_param_file = os.path.join(
        get_package_share_directory('paesano_localization'),
        'config',
        'localization.yaml',
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
    map_yaml_default = PythonExpression([
        "'/home/Paesano/ros2_ws/maps/my_map.yaml' if '",
        LaunchConfiguration('sim'),
        "' in ['true', 'True', '1'] else '/home/Paesano/ros2_ws_pi/maps/my_map_real.yaml'"
    ])
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=map_yaml_default,
        description='Initial map yaml for nav2 map_server',
    )
    imu_input_topic = LaunchConfiguration('imu_input_topic')
    sim = LaunchConfiguration('sim')
    localization_mode = LaunchConfiguration('localization_mode')
    map_yaml = LaunchConfiguration('map_yaml')
    localization_mode_enabled = PythonExpression([
        "'",
        localization_mode,
        "'.lower() in ['true', '1', 'yes', 'on']"
    ])

    return LaunchDescription([
        imu_input_arg,
        sim_arg,
        localization_mode_arg,
        map_yaml_arg,

        # -------------------------
        # IMU covariance node
        # -------------------------
        Node(
            package='paesano_localization',
            executable='paesano_localization_CovariancesOnImu',
            name='imu_covariances',
            parameters=[
                localization_param_file,
                {
                    'imu_input_topic': imu_input_topic,
                    'use_sim_time': ParameterValue(sim, value_type=bool),
                },
            ],
        ),

        # ---------------
        # EKF
        # ---------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_param_file,
                {'use_sim_time': ParameterValue(sim, value_type=bool)},
            ],
            remappings=[('odometry/filtered', '/odom/filtered')],
        ),

        # -------------------------
        # Particle filter (HW)
        # -------------------------
        Node(
            package='paesano_localization',
            executable='paesano_localization_ParticleFilter',
            name='particle_filter',
            output='screen',
            parameters=[
                localization_param_file,
                {'use_sim_time': ParameterValue(sim, value_type=bool)},
            ],
            condition=IfCondition(localization_mode_enabled),
        ),

    
        # -------------------------
        # Map loader bridge service
        # -------------------------
        Node(
            package='paesano_localization',
            executable='paesano_localization_MapLoaderService',
            name='map_loader_service',
            output='screen',
            parameters=[
                localization_param_file,
                {'use_sim_time': ParameterValue(sim, value_type=bool)},
            ],
            condition=IfCondition(localization_mode_enabled),
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
            condition=IfCondition(localization_mode_enabled),
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
            condition=IfCondition(localization_mode_enabled),
        ),
    ])
