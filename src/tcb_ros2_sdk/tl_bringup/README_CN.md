# tl_bringup 中文说明

## 1. 功能简介
`tl_bringup` 是天链 TCB 系列机械臂的 ROS2 启动包，用于集成和启动整个机械臂系统的所有必要组件。该包提供了统一的启动文件，可以方便地启动驱动、控制、规划等各个模块。

## 2. 包结构
```
tl_bringup/
├── launch/                           # 启动文件目录
│   ├── composite_robot_bringup.launch.py    # 双臂复合机械臂启动文件
│   ├── tcb605_05_bringup.launch.py          # TCB605_05 型号启动文件
│   ├── tcb610_06_bringup.launch.py          # TCB610_06 型号启动文件
│   ├── tcb705_05_bringup.launch.py          # TCB705_05 型号启动文件
│   └── tcb710_06_bringup.launch.py          # TCB710_06 型号启动文件
├── resource/                         # 资源目录
├── test/                             # 测试目录
├── package.xml                       # ROS2 包配置文件
├── setup.py                          # Python 包配置文件
├── setup.cfg                         # 设置配置文件
└── README_CN.md                      # 本文件
```

## 3. 依赖包
- `tl_driver`：机械臂底层驱动包
- `tcb605_05_config`：TCB605_05 机械臂 MoveIt2 配置
- `tcb610_06_config`：TCB610_06 机械臂 MoveIt2 配置
- `tcb705_05_config`：TCB705_05 机械臂 MoveIt2 配置
- `tcb710_06_config`：TCB710_06 机械臂 MoveIt2 配置
- `tl_control`：机械臂控制包
- `rclpy`：ROS2 Python 客户端库

## 4. 运行环境
- Ubuntu 22.04
- ROS2 Humble
- Python3
- 已正确配置 ROS2 工作空间

## 5. 编译与安装

在工作空间根目录执行：

```bash
cd ~/tcb_ros2_ws
colcon build --packages-select tl_bringup
source install/setup.bash
```

## 6. 启动方式

### 6.1 启动单臂系统（以 TCB605_05 为例）

```bash
ros2 launch tl_bringup tcb605_05_bringup.launch.py
```

其中，机械臂型号可选：
- `tcb605_05`：6自由度、5kg 负载
- `tcb610_06`：6自由度、6kg 负载
- `tcb705_05`：7自由度、5kg 负载
- `tcb710_06`：7自由度、6kg 负载

对应启动命令：
```bash
ros2 launch tl_bringup tcb605_05_bringup.launch.py use_rviz:=true
ros2 launch tl_bringup tcb610_06_bringup.launch.py use_rviz:=true
ros2 launch tl_bringup tcb705_05_bringup.launch.py use_rviz:=true
ros2 launch tl_bringup tcb710_06_bringup.launch.py use_rviz:=true
```

### 6.2 启动双臂系统

```bash
ros2 launch tl_bringup composite_robot_bringup.launch.py
```

## 7. 启动文件参数说明

启动文件中可能包含以下常见参数（具体参数请查阅对应的 launch 文件）：

- `use_sim_time`：是否使用模拟时间（仿真时为 `true`，真实硬件为 `false`）
- `arm_type`：机械臂型号
- `arm_mode`：`single`（单臂）/ `dual`（双臂）
- 其他特定硬件配置参数

## 8. 检验启动成功

启动后，可以通过以下命令检查系统是否正常运行：

```bash
# 查看所有活跃节点
ros2 node list

# 查看所有话题
ros2 topic list

# 查看关节状态话题内容
ros2 topic echo /tl_driver/current_joint_states

# 查看末端位姿话题内容
ros2 topic echo /tl_driver/end_pose
```

## 9. 常见问题

### 问题 1：启动时提示找不到依赖包
**解决方案**：
```bash
# 确保已构建所有必要的包
colcon build
source install/setup.bash
```

### 问题 2：与硬件连接失败
**解决方案**：
- 检查机械臂电源是否打开
- 检查网络连接是否正常
- 查看 `tl_driver` 配置文件中的 IP 地址和端口号是否正确

### 问题 3：节点启动后立即退出
**解决方案**：
- 查看控制台的错误日志
- 检查依赖的硬件配置是否完整
- 确保 ROS2 环境变量正确设置

## 10. 相关文档

更详细的信息请参考：
- [tl_driver](../tl_driver/README_CN.md)：驱动包说明
- [tl_robot_description](../tl_robot_description/README_CN.md)：机械臂模型说明
- [tl_control](../tl_control/README_CN.md)：控制包说明
- [tl_moveit2_config](../tl_moveit2_config/)：MoveIt2 配置说明

## 11. 许可证

Apache License 2.0
