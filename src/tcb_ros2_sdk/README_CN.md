# tcb_ros2_sdk 说明文档

## 概述
`tcb_ros2_sdk` 由一系列 ROS2 功能包组成，为天链 TCB 系列机械臂提供完整的 ROS2 支持：驱动、描述、启动、MoveIt2 配置、控制以及 ros2_control 硬件接口。以下说明帮助您搭建环境、编译、运行并安全使用机械臂。

---

## 支持环境
- **控制器协议**：2207 或 2403（可在 `tl_driver/config/tl_driver_config.yaml` 中修改）
- **操作系统**：Ubuntu 22.04
- **ROS2 发行版**：Humble
- **Python 包**
  - Pinocchio 3.7.0 (可选，仅用于运动学计算)
  - numpy 1.24.0
  - scipy 1.8.0

> 建议使用 `python3 -m pip install --user pin==3.7.0 numpy==1.24.0 scipy==1.8.0` 安装

---

## 快速搭建
### 1. 安装 ROS2
参照官方文档：<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

### 2. 安装 MoveIt2
参照指南：<https://moveit.ros.org/install-moveit2/binary/>

### 3. 创建工作空间并编译
```bash
mkdir -p ~/tcb_ros2_ws/src
cp -r tcb_ros2_sdk ~/tcb_ros2_ws/src
cd ~/tcb_ros2_ws

colcon build
source install/setup.bash
```

> 如果只需单个包，可使用 `--packages-select` 或 `--packages-glob` 限制编译。

---

##  功能包一览
1. **硬件驱动** `tl_driver`：ROS2 底层驱动，通信协议2207/2403。
2. **系统启动** `tl_bringup`：多节点启动集合。
3. **模型描述** `tl_robot_description`：URDF/TF 发布与可视化。
4. **MoveIt2 配置** `tl_moveit2_config`：各型号 MoveIt2 配置与示例。
5. **控制节点** `tl_control`：MoveIt2 运动规划与轨迹执行。
6. **ros2_control 接口** `tl_hardware`：硬件接口插件。

> 各包都有独立的 `README_CN.md`，请参阅以获取详细说明。

---

## 使用示例
### 1. 虚拟机械臂（在 MoveIt2 仿真）
1. 在对应配置包（`tcb<arm_version>_config`）的 `*.ros2_control.xacro` 中替换：
    ```xml
    <plugin>tl_hardware/TlHardwareInterface</plugin>
    <param name="control_mode">position</param>
    <param name="trajectory_mode">auto</param>
    ```
    为
    ```xml
    <plugin>mock_components/GenericSystem</plugin>
    ```
2. 重新编译：
    ```bash
    cd ~/tcb_ros2_ws
    colcon build --packages-select tcb<arm_version>_config
    source install/setup.bash
    ```
3. 启动仿真：
    ```bash
    ros2 launch tcb<arm_version>_config demo.launch.py use_rviz:=true
    ```
4. 在 RViz 中使用 MoveIt2 进行规划与执行。


### 2. 真实机械臂
> 需确保硬件已连接且网络可达。

1. 在 MoveIt2 配置包的 `*.ros2_control.xacro` 中保持默认硬件插件：
    ```xml
    <plugin>tl_hardware/TlHardwareInterface</plugin>
    <param name="control_mode">position</param>
    <param name="trajectory_mode">auto</param>
    ```
2. 支持的 `trajectory_mode`：
   - `controller`：仅使用 ros2 controller 的关节指令
   - `external`：仅使用 `/teleop/target_position` 外部指令
   - `auto`：有外部指令时使用 `external`，超时回切 `controller`
3. 启动流程：
    ```bash
    # 1. bringup 驱动与系统
    ros2 launch tl_bringup tcb<arm_version>_bringup.launch.py use_rviz:=true
    # 2. 启动 MoveIt2 环境
    ros2 launch tcb<arm_version>_config demo.launch.py use_rviz:=true
    # 3. （可选）启动控制节点
    ros2 launch tl_control tl_control.launch.py
    ```

> 或者在不同终端分步运行以上命令。

---

## 安全提示
- 开机前检查机械臂安装牢固、无松动部件。
- 确保急停开关处于可用状态。
- 工作空间内无障碍物，人员保持安全距离。
- 操作前明确控制目的，避免不必要动作。


---

如需进一步帮助，请参阅各子包文档或联系维护人员。

© 2026 TCB ROS2 SDK
