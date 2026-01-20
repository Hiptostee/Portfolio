from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slambot_localization',
            executable='slambot_localization_CovariancesOnImu',
            name='slambot_localization',
        ),
    ])
