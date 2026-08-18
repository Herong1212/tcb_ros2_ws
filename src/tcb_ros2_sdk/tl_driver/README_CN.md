# tl_driver 中文说明

## 1. 功能简介
`tl_driver` 是天链 TCB 系列机械臂的 ROS2 底层驱动包，基于 Python SDK（2207/2403 协议）实现：
- 机械臂上电、下电控制
- 关节轨迹接收与下发（`queue` / `motion_control` / `servo_j`）
- 关节状态与末端位姿发布
- 单臂与双臂模式切换

## 2. 包结构
```
tl_driver/
├── tl_driver/
│   ├── __init__.py                   # 包初始化文件
│   └── tl_driver.py                  # 驱动主节点
├── launch/
│   └── tl_driver.launch.py           # 启动文件
├── config/
│   └── tl_driver_config.yaml         # 参数配置文件
├── sdk/                              # 机械臂 SDK 与模型资源
├── resource/                         # 资源目录
├── test/                             # 测试目录
├── package.xml                       # ROS2 包配置文件
├── setup.py                          # Python 包配置文件
├── setup.cfg                         # 设置配置文件
└── README_CN.md                      # 本文件
```

## 3. 运行环境
- Ubuntu 22.04
- ROS2 Humble
- Python3
- 机械臂控制器协议：`2207` 或 `2403`

## 4. 编译与启动
在工作空间根目录执行：

```bash
cd ~/tcb_ros2_ws
colcon build --packages-select tl_driver
source install/setup.bash
```

启动驱动：

```bash
ros2 launch tl_driver tl_driver.launch.py
```

## 5. 主要参数（`config/tl_driver_config.yaml`）
### 通用参数
- `arm_mode`：`single`（单臂）/ `dual`（双臂）
- `frequency`：关节状态发布频率（Hz）
- `debug_logging`：是否开启调试日志
- `port_6001`、`port_7000`：机械臂通信端口

### 左臂（单臂模式下即唯一机械臂）
- `ip1`：机械臂 IP
- `sdk_version1`：`2207` 或 `2403`
- `dof1`：自由度（6 或 7）
- `trajectory_timeout1`：轨迹聚合超时（秒）
- `stride1`：轨迹采样步长（`queue`/`motion_control` 模式）
- `arm_control_mode1`：`queue` / `motion_control` / `servo_j`
- `speed1`：全局速度（1~100）
- `position_tolerance1`：到位位置容差（度）
- `velocity_tolerance1`：到位速度容差（度/秒）
- `settle_time1`：到位稳定时间（秒）
- `servo_j_frequency1`：`servo_j` 发送频率（Hz）

### 右臂（仅双臂模式）
与左臂同名参数后缀改为 `2`：`ip2`、`sdk_version2`、`dof2` 等。

## 6. 话题接口
### 订阅
- `tl_driver/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`)：轨迹输入
- `tl_driver/cmd` (`std_msgs/msg/String`)：控制命令
- `tl_driver/set_speed` 或 `tl_driver/<arm_key>/set_speed` (`std_msgs/msg/Int32`)：运行时调速

### 发布
- `tl_driver/current_joint_states` (`sensor_msgs/msg/JointState`)：关节状态
- 单臂：`tl_driver/motion_complete` (`std_msgs/msg/String`)
- 双臂：`tl_driver/armleft/motion_complete`、`tl_driver/armright/motion_complete` (`std_msgs/msg/String`)
- 单臂：`/tl_driver/end_pose` (`geometry_msgs/msg/PoseStamped`)
- 双臂：`/tl_driver/armleft/end_pose`、`/tl_driver/armright/end_pose` (`geometry_msgs/msg/PoseStamped`)

## 7. 控制命令示例
### 上电/下电

```bash
# 双臂同时上电
ros2 topic pub --once /tl_driver/cmd std_msgs/msg/String "data: 'arm_power_on'"

# 双臂同时下电
ros2 topic pub --once /tl_driver/cmd std_msgs/msg/String "data: 'arm_power_off'"

# 单独控制左臂/右臂（仅双臂模式）
ros2 topic pub --once /tl_driver/cmd std_msgs/msg/String "data: 'armleft_power_on'"
ros2 topic pub --once /tl_driver/cmd std_msgs/msg/String "data: 'armright_power_off'"
```

### 调整速度（1~100）

```bash
# 单臂
ros2 topic pub --once /tl_driver/set_speed std_msgs/msg/Int32 "data: 30"

# 双臂（左臂示例）
ros2 topic pub --once /tl_driver/armleft/set_speed std_msgs/msg/Int32 "data: 30"
```

## 8. 轨迹输入说明
- 输入单位：`JointTrajectory.points[*].positions` 使用弧度（rad）
- 驱动内部会自动换算为 SDK 所需角度（deg）
- 单臂模式支持两种关节命名：
  - `tl_robot_joint1 ...`
  - `left_tl_robot_joint1 ...`
- 双臂模式需在 `joint_names` 中同时包含：
  - 左臂：`left_tl_robot_joint1 ...`
  - 右臂：`right_tl_robot_joint1 ...`

## 9. 注意事项
- 首次运行前请确认 IP、端口、协议版本与机械臂实际配置一致。
- 建议先在低速（如 `speed=10~30`）下验证轨迹。
- 运动期间请确保急停有效，工作空间无人员和障碍物。
- `servo_j` 模式为在线控制，话题输入频率应与 `servo_j_frequency` 匹配。
