from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("tl_control")
    return LaunchDescription(
        [
            Node(
                package="tl_control",
                executable="tl_control_node",
                name="tl_control",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    os.path.join(pkg_dir, "config", "tl_control_config.yaml"),
                ],
            )
        ]
    )
