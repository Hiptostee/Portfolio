from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([Node(
        package='paesano_orchestrator', executable='orchestrator_node',
        name='orchestrator_node', output='screen',
        parameters=[os.path.join(get_package_share_directory('paesano_orchestrator'), 'config', 'orchestrator.yaml')])])
