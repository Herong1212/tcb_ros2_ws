#!/usr/bin/env python3
# Copyright 2024 tlibot
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
启动文件：tl_moveit2_demo
用法：
    ros2 launch tl_moveit2_demo moveit2_demo.launch.py

前提：
    已通过 tcb610_06_config 包启动 move_group（或在同一 launch 中一并启动）。
    本文件默认假设 move_group 已在外部启动；若需一体化启动，
    请将 use_move_group 参数设为 true（需要 tcb610_06_config 包）。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── 参数声明 ──────────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="是否使用仿真时钟（Gazebo / Ignition）",
    )

    # ── 演示节点 ──────────────────────────────────────────────────────────────
    # 延迟 3 秒启动，等待 move_group 就绪
    demo_node = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="[tl_moveit2_demo] 启动演示节点……"),
            Node(
                package="tl_moveit2_demo",
                executable="moveit2_demo_node",
                name="moveit2_demo_node",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            demo_node,
        ]
    )
