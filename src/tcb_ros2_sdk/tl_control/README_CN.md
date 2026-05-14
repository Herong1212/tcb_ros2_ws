# tl_control 中文说明

## 1. 功能简介
`tl_control` 是天链 TCB 系列机械臂的 ROS2 控制包，基于 MoveIt2 运动规划框架实现。该包提供了一个 C++ 控制节点，用于与 MoveIt2 通信，实现机械臂的轨迹规划、运动执行和位姿控制。支持单臂和双臂两种工作模式。

## 2. 包结构
```
tl_control/
├── src/
│   └── tl_control.cpp                # 控制节点主程序（C++）
├── include/
│   └── tl_control/
│       └── tl_control.hpp            # 控制节点头文件
├── launch/
│   └── tl_control.launch.py          # 节点启动文件
├── config/
│   └── tl_control_config.yaml        # 参数配置文件
├── CMakeLists.txt                    # CMake 构建配置
├── package.xml                       # ROS2 包配置
└── README_CN.md                      # 本文件
```

## 3. 主要功能
- **MoveIt2 集成**：与 MoveIt2 运动规划框架深度集成
- **单臂/双臂支持**：通过配置参数灵活切换工作模式
- **轨迹规划**：支持点对点运动规划
- **关节状态订阅**：监听关节状态并与 MoveIt2 同步
- **位姿控制**：支持末端位姿（笛卡尔空间）控制
- **速度加速度缩放**：动态调整运动速度和加速度

## 4. 依赖包
- `rclcpp`：ROS2 C++ 客户端库
- `std_msgs`：标准消息类型
- `sensor_msgs`：传感器消息类型
- `geometry_msgs`：几何消息类型
- `moveit_ros_planning_interface`：MoveIt2 规划接口
- `tf2_ros`：变换坐标系库
- `tf2_geometry_msgs`：几何消息变换
- `moveit_ros_move_group`：MoveIt2 Move Group 服务

## 5. 运行环境
- Ubuntu 22.04
- ROS2 Humble
- C++14 或更高版本
- MoveIt2 框架
- 已正确配置 ROS2 工作空间

## 6. 编译与安装

在工作空间根目录执行：

```bash
cd ~/tcb_ros2_ws
colcon build --packages-select tl_control
source install/setup.bash
```

## 7. 启动方式

### 7.1 直接启动控制节点

使用默认配置启动：
```bash
ros2 launch tl_control tl_control.launch.py
```

### 7.2 通过 bringup 包启动

推荐通过 `tl_bringup` 包启动整个系统（包括驱动和控制）：
```bash
ros2 launch tl_bringup tcb710_06_bringup.launch.py
```

## 8. 参数配置

### 8.1 配置文件位置
`config/tl_control_config.yaml`

### 8.2 主要参数说明

#### 运行模式参数
| 参数 | 类型 | 默认值 | 说明 |
|-----|------|--------|------|
| `arm_mode` | string | `single` | 工作模式：`single`（单臂）或 `dual`（双臂） |

#### MoveIt2 规划组参数
| 参数 | 类型 | 说明 |
|-----|------|------|
| `planning_groups.single` | string | 单臂模式下的规划组名称 |
| `planning_groups.left` | string | 双臂模式下的左臂规划组名称 |
| `planning_groups.right` | string | 双臂模式下的右臂规划组名称 |

#### 规划组名称说明

**单臂模式**（选择其中一个）：
- `tcb605_05_group`：TCB605_05 型号（6自由度、5kg 负载）
- `tcb610_06_group`：TCB610_06 型号（6自由度、6kg 负载）
- `tcb705_05_group`：TCB705_05 型号（7自由度、5kg 负载）
- `tcb710_06_group`：TCB710_06 型号（7自由度、6kg 负载）

**双臂模式**：
- `left`：`armleft_group`
- `right`：`armright_group`

### 8.3 配置示例

**单臂配置示例**（TCB710_06）：
```yaml
tl_control:
  ros__parameters:
    arm_mode: "single"
    planning_groups:
      single: "tcb710_06_group"
      left: ""
      right: ""
```

**双臂配置示例**：
```yaml
tl_control:
  ros__parameters:
    arm_mode: "dual"
    planning_groups:
      single: ""
      left: "armleft_group"
      right: "armright_group"
```

## 9. 话题接口

### 9.1 订阅话题

**单臂模式**：
- `/tl_driver/current_joint_states` (`sensor_msgs/msg/JointState`)：关节状态

**双臂模式**：
- `/tl_driver/armleft/current_joint_states` (`sensor_msgs/msg/JointState`)：左臂关节状态
- `/tl_driver/armright/current_joint_states` (`sensor_msgs/msg/JointState`)：右臂关节状态

### 9.2 发布话题

与 MoveIt2 通过 Move Group 接口通信，具体话题由 MoveIt2 框架管理。

可以使用下面的命令发送控制，需要确认好关节空间控制下的 position 或者笛卡尔空间下控制的 frame_id、position、orientation 是否正确。

```bash
ros2 topic pub --once /tl_control/joint_motion sensor_msgs/msg/JointState "
name:
- tl_robot_joint1
- tl_robot_joint2
- tl_robot_joint3
- tl_robot_joint4
- tl_robot_joint5
- tl_robot_joint6
- tl_robot_joint7
position:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 1.57
"
```

```bash
ros2 topic pub --once /tl_control/cartesian_motion geometry_msgs/msg/PoseStamped "
header:
  frame_id: tl_robot_link0
pose:
  position:
    x: 0.40
    y: 0.0
    z: 0.468
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0
"
```

```bash
ros2 topic pub --once /tl_control/cartesian_linear_motion geometry_msgs/msg/PoseStamped "
header:
  frame_id: tl_robot_link0
pose:
  position:
    x: 0.450
    y: 0.0
    z: 0.468
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0
"
```

## 10. 规划参数

控制节点中的规划参数（在 `initArm()` 方法中设置）：

| 参数 | 默认值 | 说明 |
|-----|--------|------|
| 规划时间 | 3.0 秒 | 运动规划器允许的最大规划时间 |
| 规划尝试次数 | 5 次 | 规划失败后的重试次数 |
| 最大速度缩放因子 | 0.5 | 取值范围 0.0~1.0 |
| 最大加速度缩放因子 | 0.5 | 取值范围 0.0~1.0 |

这些参数可根据需要在代码中进行修改。

## 11. 节点架构

```
tl_control
├── 单臂模式
│   ├── MoveGroupInterface (single)
│   └── 话题订阅
│       └── /tl_driver/current_joint_states
└── 双臂模式
    ├── MoveGroupInterface (left)
    ├── MoveGroupInterface (right)
    └── 话题订阅
        ├── /tl_driver/armleft/current_joint_states
        └── /tl_driver/armright/current_joint_states
```

## 12. 典型工作流

1. **系统启动**：通过 `tl_bringup` 启动整个系统
2. **驱动初始化**：`tl_driver` 连接到机械臂硬件
3. **控制节点初始化**：`tl_control` 初始化 MoveIt2 接口
4. **状态同步**：监听关节状态并同步到 MoveIt2
5. **轨迹规划**：通过 MoveIt2 进行运动规划
6. **轨迹执行**：将规划结果发送给驱动执行

## 13. 常见问题

### 问题 1：启动时提示规划组不存在
**原因**：配置文件中的规划组名称与 MoveIt2 配置不匹配

**解决方案**：
- 确认机械臂型号对应的规划组名称
- 检查 `config/tl_control_config.yaml` 中的规划组名称是否正确
- 确保相应的 MoveIt2 配置包已正确编译

### 问题 2：控制节点无法连接到 MoveIt2
**原因**：MoveIt2 Move Group 未正常启动

**解决方案**：
- 检查 `tl_driver` 是否已启动
- 检查 MoveIt2 配置包的启动过程是否有错误
- 查看节点日志获取详细错误信息

### 问题 3：规划失败或规划时间过长
**原因**：规划参数设置不合理或起始位置不可达

**解决方案**：
- 调整 `tl_control.cpp` 中的规划参数（规划时间、尝试次数等）
- 检查机械臂当前位置是否有自碰撞
- 使用 RViz 可视化查看规划过程

## 14. 调试技巧

### 查看活跃节点
```bash
ros2 node list
```

### 查看所有话题
```bash
ros2 topic list
```

### 监听关节状态
```bash
ros2 topic echo /tl_driver/current_joint_states
```

### 查看节点日志
```bash
ros2 run tl_control tl_control_node --ros-args --log-level debug
```

## 15. 相关文档

更详细的信息请参考：
- [tl_driver](../tl_driver/README_CN.md)：驱动包说明
- [tl_bringup](../tl_bringup/README_CN.md)：启动包说明
- [tl_robot_description](../tl_robot_description/README_CN.md)：机械臂模型说明
- [MoveIt2 官方文档](https://moveit.ros.org/)

## 16. 许可证

Apache License 2.0
