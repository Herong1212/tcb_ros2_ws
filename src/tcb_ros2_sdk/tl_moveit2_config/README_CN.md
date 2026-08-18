# tl_moveit2_config 中文说明

## 1. 功能简介
`tl_moveit2_config` 是天链 TCB 系列机械臂的 MoveIt2 配置包集合。该包包含了所有支持的机械臂型号的 MoveIt2 配置，包括 URDF 模型、运动学配置、控制器配置等。用户可以通过这些配置包快速启动 MoveIt2 运动规划框架。

主要特点：
- 支持单臂多种型号（TCB605、TCB610、TCB705、TCB710）
- 支持双臂复合机械臂
- 完整的 MoveIt2 集成
- 包含 RViz 可视化配置
- 提供多种启动文件

## 2. 包结构
```
tl_moveit2_config/
├── tcb605_05_config/              # TCB605_05 型号配置包
│   ├── launch/                    # 启动文件
│   ├── config/                    # 配置文件
│   ├── package.xml
│   └── CMakeLists.txt
├── tcb610_06_config/              # TCB610_06 型号配置包
│   ├── launch/
│   ├── config/
│   ├── package.xml
│   └── CMakeLists.txt
├── tcb705_05_config/              # TCB705_05 型号配置包
│   ├── launch/
│   ├── config/
│   ├── package.xml
│   └── CMakeLists.txt
├── tcb710_06_config/              # TCB710_06 型号配置包
│   ├── launch/
│   ├── config/
│   ├── package.xml
│   └── CMakeLists.txt
└── composite_robot_config/        # 双臂复合机械臂配置包
    ├── launch/
    ├── config/
    ├── package.xml
    └── CMakeLists.txt
```

## 3. 支持的机械臂型号

### 单臂型号
| 配置包 | 型号 | 自由度 | 负载 | 说明 |
|------|------|--------|------|------|
| `tcb605_05_config` | TCB605_05 | 6 | 5kg | 6自由度、5kg 负载 |
| `tcb610_06_config` | TCB610_06 | 6 | 6kg | 6自由度、6kg 负载 |
| `tcb705_05_config` | TCB705_05 | 7 | 5kg | 7自由度、5kg 负载 |
| `tcb710_06_config` | TCB710_06 | 7 | 6kg | 7自由度、6kg 负载 |

### 双臂型号
| 配置包 | 组成 | 说明 |
|------|------|------|
| `composite_robot_config` | 双臂 | 两个机械臂组成的复合系统 |

## 4. 各配置包的共同结构

每个配置包都包含以下主要文件：

### 4.1 启动文件（launch/）

| 文件 | 功能 |
|-----|------|
| `demo.launch.py` | 演示启动文件，包含完整的 MoveIt2 环境 |
| `move_group.launch.py` | MoveIt2 Move Group 启动文件 |
| `moveit_rviz.launch.py` | RViz 可视化启动文件 |
| `rsp.launch.py` | 机器人状态发布器启动文件 |
| `setup_assistant.launch.py` | MoveIt Setup Assistant 启动文件 |
| `spawn_controllers.launch.py` | 控制器启动文件 |
| `static_virtual_joint_tfs.launch.py` | 虚拟关节 TF 启动文件 |
| `warehouse_db.launch.py` | 轨迹数据库启动文件 |

### 4.2 配置文件（config/）

| 文件 | 功能 |
|-----|------|
| `<ROBOT_NAME>.urdf.xacro` | 机械臂 URDF 模型文件 |
| `<ROBOT_NAME>.srdf` | 语义机械臂描述文件（定义规划组等） |
| `<ROBOT_NAME>.ros2_control.xacro` | ros2_control 硬件配置 |
| `kinematics.yaml` | 运动学求解器配置 |
| `ros2_controllers.yaml` | ROS2 控制器配置 |
| `moveit_controllers.yaml` | MoveIt2 控制器配置 |
| `joint_limits.yaml` | 关节限制配置 |
| `pilz_cartesian_limits.yaml` | 笛卡尔空间限制配置 |
| `initial_positions.yaml` | 初始位置配置 |
| `moveit.rviz` | RViz 配置文件 |

## 5. 依赖包

所有配置包都依赖于以下 MoveIt2 相关包：

- `moveit_ros_move_group`：MoveIt2 Move Group
- `moveit_kinematics`：运动学求解器
- `moveit_planners`：运动规划器
- `moveit_simple_controller_manager`：控制器管理器
- `moveit_ros_visualization`：RViz 可视化
- `moveit_setup_assistant`：MoveIt Setup Assistant
- `joint_state_publisher`：关节状态发布器
- `joint_state_publisher_gui`：关节状态发布器 GUI
- `robot_state_publisher`：机器人状态发布器
- `controller_manager`：ROS2 Control 管理器
- `xacro`：Xacro 处理器

## 6. 运行环境

- Ubuntu 22.04
- ROS2 Humble
- MoveIt2
- RViz2

## 7. 编译与安装

在工作空间根目录执行：

```bash
cd ~/tcb_ros2_ws

# 编译所有配置包
colcon build --packages-glob "*config"

# 或编译特定配置包
colcon build --packages-select tcb710_06_config

source install/setup.bash
```

## 8. 启动方式

### 8.1 启动单臂演示（以 TCB710_06 为例）

启动完整的 MoveIt2 演示环境，包括 RViz 可视化：

```bash
ros2 launch tcb710_06_config demo.launch.py
```

其他型号对应的启动命令：
```bash
# TCB605_05
ros2 launch tcb605_05_config demo.launch.py

# TCB610_06
ros2 launch tcb610_06_config demo.launch.py

# TCB705_05
ros2 launch tcb705_05_config demo.launch.py
```

### 8.2 启动双臂演示

```bash
ros2 launch composite_robot_config demo.launch.py
```

### 8.3 启动单独的 Move Group

如果只需要运动规划功能，不需要 RViz：

```bash
ros2 launch tcb710_06_config move_group.launch.py
```

### 8.4 启动 RViz 可视化

与 Move Group 一起启动 RViz：

```bash
ros2 launch tcb710_06_config moveit_rviz.launch.py
```

## 9. 配置说明

### 9.1 运动学求解器配置（kinematics.yaml）

配置 IK 求解器参数：

```yaml
tcb710_06_group:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

关键参数：
- `kinematics_solver`：使用的运动学求解器（推荐 KDL）
- `kinematics_solver_search_resolution`：搜索分辨率（较小 = 更精确但较慢）
- `kinematics_solver_timeout`：求解超时时间（秒）

### 9.2 ROS2 控制器配置（ros2_controllers.yaml）

配置关节轨迹控制器：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # 更新频率（Hz）

    tcb710_06_group_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

tcb710_06_group_controller:
  ros__parameters:
    joints:
      - tl_robot_joint1
      - tl_robot_joint2
      # ...
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

### 9.3 关节限制配置（joint_limits.yaml）

定义关节的速度、加速度和努力限制。

### 9.4 初始位置配置（initial_positions.yaml）

定义机械臂的初始关节位置。


### 10.2 常见启动参数

启动文件中常见参数：

- `use_sim_time`：是否使用模拟时间
- `rviz_config_file`：RViz 配置文件路径
- `robot_description_semantic_file`：SRDF 文件路径

## 11. MoveIt2 规划组

每个配置包定义了以下规划组（在 SRDF 文件中）：

### 单臂配置
- `tcb605_05_group` / `tcb610_06_group` / `tcb705_05_group` / `tcb710_06_group`：对应的机械臂规划组

### 双臂配置
- `armleft_group`：左臂规划组
- `armright_group`：右臂规划组

## 12. 常见操作

### 12.1 在 RViz 中进行运动规划

1. 启动 demo：`ros2 launch tcb710_06_config demo.launch.py`
2. 在 RViz 中使用 Motion Planning 插件
3. 拖动 Goal State 的交互控制器到目标位置
4. 点击 "Plan" 按钮进行规划
5. 点击 "Execute" 按钮执行运动

### 12.2 通过代码进行运动规划

使用 `moveit_msgs` 发送规划请求或使用 MoveIt2 Python 接口。


## 13. 调试

### 13.1 查看运动学求解器状态

启用 debug 日志：
```bash
ros2 launch tcb710_06_config demo.launch.py --log-level debug
```

## 14. 常见问题

### 问题 1：启动时提示规划组不存在
**原因**：SRDF 文件中规划组定义与代码不匹配

**解决方案**：
- 使用 Setup Assistant 重新生成配置
- 检查 SRDF 文件中的规划组名称

### 问题 2：IK 求解失败
**原因**：目标位置不在机械臂的工作空间内或求解参数不合适

**解决方案**：
- 调整 `kinematics_solver_search_resolution` 和 `kinematics_solver_timeout`
- 检查目标位置是否在工作空间内
- 使用 FK 验证目标位置的可达性

### 问题 3：RViz 中无法看到机械臂模型
**原因**：URDF 加载失败或模型资源路径不正确

**解决方案**：
- 检查 `.xacro` 文件路径
- 重新启动节点
- 查看控制台错误信息

### 问题 4：运动规划速度慢
**原因**：规划参数设置不合理

**解决方案**：
- 增加 `kinematics_solver_timeout`
- 减少 `kinematics_solver_search_resolution`
- 检查 CPU 性能

## 15. 文件自定义

### 15.1 修改初始位置

编辑 `config/initial_positions.yaml`：

```yaml
tcb710_06_group:
  - joint_name: tl_robot_joint1
    position: 0.0
  - joint_name: tl_robot_joint2
    position: -1.57
  # ...
```

### 15.2 修改关节限制

编辑 `config/joint_limits.yaml`：

```yaml
joint_limits:
  tl_robot_joint1:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: true
    max_acceleration: 1.0
```

### 15.3 修改控制器参数

编辑 `config/ros2_controllers.yaml` 中的控制器参数。

## 16. 相关文档

更详细的信息请参考：
- [tl_bringup](../tl_bringup/README_CN.md)：启动包说明
- [tl_driver](../tl_driver/README_CN.md)：驱动包说明
- [tl_control](../tl_control/README_CN.md)：控制包说明
- [tl_robot_description](../tl_robot_description/README_CN.md)：机械臂模型说明
- [MoveIt2 官方文档](https://moveit.ros.org/)
- [ROS2 Control 官方文档](https://control.ros.org/)

## 17. 许可证

BSD License
