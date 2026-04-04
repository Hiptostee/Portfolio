from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='true for sim time (/clock), false for hardware time',
    )
    startup_mode_arg = DeclareLaunchArgument(
        'startup_mode',
        default_value='mapping',
        description='Initial robot mode to launch through the bridge',
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=PythonExpression([
            "'/home/slambot/ros2_ws/maps/my_map.yaml' if '",
            LaunchConfiguration('sim'),
            "' in ['true', 'True', '1'] else '/home/slambot/ros2_ws_pi/maps/my_map_real.yaml'",
        ]),
        description='Canonical localization map yaml',
    )
    map_save_prefix_arg = DeclareLaunchArgument(
        'map_save_prefix',
        default_value=PythonExpression([
            "'/home/slambot/ros2_ws/maps/my_map' if '",
            LaunchConfiguration('sim'),
            "' in ['true', 'True', '1'] else '/home/slambot/ros2_ws_pi/maps/my_map_real'",
        ]),
        description='Canonical map save prefix without extension',
    )
    http_host_arg = DeclareLaunchArgument(
        'http_host',
        default_value='0.0.0.0',
        description='HTTP bind host for the mobile bridge',
    )
    http_port_arg = DeclareLaunchArgument(
        'http_port',
        default_value='8000',
        description='HTTP bind port for the mobile bridge',
    )

    return LaunchDescription([
        sim_arg,
        startup_mode_arg,
        map_yaml_arg,
        map_save_prefix_arg,
        http_host_arg,
        http_port_arg,
        Node(
            package='slambot_mobile_bridge',
            executable='bridge_node',
            name='slambot_mobile_bridge',
            output='screen',
            parameters=[{
                'sim': LaunchConfiguration('sim'),
                'startup_mode': LaunchConfiguration('startup_mode'),
                'default_map_yaml': LaunchConfiguration('map_yaml'),
                'map_save_prefix': LaunchConfiguration('map_save_prefix'),
                'http_host': LaunchConfiguration('http_host'),
                'http_port': LaunchConfiguration('http_port'),
            }],
        ),
    ])
