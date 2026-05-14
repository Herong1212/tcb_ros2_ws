from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('tl_driver')
    config_path = os.path.join(pkg_dir, 'config', 'tl_driver_config.yaml')

    return LaunchDescription([
        Node(
            package='tl_driver',
            executable='tl_driver_node',
            name='tl_driver',
            output='screen',
            parameters= [config_path] if os.path.isfile(config_path) else []
        )
    ])