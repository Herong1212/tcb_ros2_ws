#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz", default="true")

    tl_driver_dir = get_package_share_directory("tl_driver")
    composite_robot_config_dir = get_package_share_directory("composite_robot_config")
    tl_control_dir = get_package_share_directory("tl_control")

    tl_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tl_driver_dir, "launch", "tl_driver.launch.py")
        )
    )

    # 延迟 2 秒启动 demo
    tcb710_06_demo_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(composite_robot_config_dir, "launch", "demo.launch.py")
                ),
                launch_arguments={"use_rviz": use_rviz}.items(),
            )
        ],
    )

    # 再延迟 2 秒启动 control
    tl_control_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tl_control_dir, "launch", "tl_control.launch.py")
                )
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz", default_value="true", description="是否启动RViz"
            ),
            tl_driver_launch,
            tcb710_06_demo_launch,
            tl_control_launch,
        ]
    )
