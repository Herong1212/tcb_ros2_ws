# tl_moveit2_demo

MoveIt2 运动规划演示包，面向 TCB610_06N 六轴机械臂的仿真环境，演示关节空间规划、笛卡尔直线规划及规划场景障碍物操作。

> **注意**：本包仅用于仿真演示，不涉及真实硬件。

## 概述

本包提供了一个完整的 MoveIt2 运动规划仿真演示，展示了如何使用 MoveIt2 接口控制 TCB610_06N 机械臂模型执行各种类型的运动规划任务。演示包括：

1. **关节空间规划**（Joint Space Planning）：将机械臂的6个关节移动到指定角度
2. **笛卡尔位姿规划**（Pose Goal Planning）：将末端执行器运动到指定的笛卡尔位姿
3. **笛卡尔直线路径规划**（Cartesian Path Planning）：末端沿直线轨迹运动
4. **回零位**（Named Target）：将机械臂返回到初始零位

## 🔧 系统要求

### 硬件要求
- **无特殊硬件要求**：本包为纯仿真演示，无需真实机械臂硬件

### 软件环境
- **操作系统**：Ubuntu 22.04
- **ROS2 发行版**：Humble
- **Python 包**：
  - Pinocchio 3.7.0（可选，仅用于运动学计算）
  - numpy 1.24.0
  - scipy 1.8.0

```bash
python3 -m pip install --user pinocchio==3.7.0 numpy==1.24.0 scipy==1.8.0
```

### 依赖包
本包依赖以下 ROS2 功能包：
- `rclcpp`：ROS2 C++ 客户端库
- `geometry_msgs`：几何消息类型
- `moveit_ros_planning_interface`：MoveIt2 规划接口
- `moveit_visual_tools`：MoveIt2 可视化工具
- `tf2_ros`：TF2 ROS 接口
- `tf2_geometry_msgs`：TF2 几何消息转换
- `moveit_ros_move_group`：MoveIt2 运动组
- `tcb610_06_config`：TCB610_06N 机械臂 MoveIt2 配置包

## 编译安装

### 1. 准备工作空间

```bash
# 创建一个工作空间
mkdir -p ~/tcb_ros2_ws/src
cd ~/tcb_ros2_ws/src

# 将 tcb_ros2_sdk 复制到工作空间
mv -r tcb_ros2_sdk ~/tcb_ros2_ws/src
# 克隆 moveit2 仓库
git clone https://github.com/moveit/moveit2.git
# 克隆 ros_testing 仓库
git clone https://github.com/ros2/ros_testing.git

cd ~/tcb_ros2_ws
colcon build
# 后续可以仅编译本包
# colcon build --packages-select tl_moveit2_demo

# 加载环境
source install/setup.bash
```

## 🚀 使用方法

### 前置条件

在启动演示之前，需要先启动 MoveIt2 仿真环境：

```bash
# 首先启动 MoveIt2 仿真环境（包含 RViz 可视化）
ros2 launch tcb610_06_config demo.launch.py use_rviz:=true
```

> 该命令会自动启动 move_group 服务、RViz 可视化界面，并使用 mock_components 插件模拟硬件接口。

### 启动演示

# 然后启动本测试案例文件
```bash
ros2 launch tl_moveit2_demo moveit2_demo.launch.py
```

默认情况下，启动文件会：
- 等待 3 秒让 `move_group` 服务完全启动
- 启动演示节点并执行所有演示任务

## 🎯 演示内容详解

### 1. 关节空间规划

演示如何将机械臂的6个关节移动到指定的目标角度：

```cpp
目标关节角度（单位：弧度）：
- J1: 90°  (π/2)
- J2: -45° (-π/4)
- J3: 60°  (π/3)
- J4: 20°  (π/9)
- J5: 35°  (约0.61弧度)
- J6: 0°   (0)
```

**参数设置**：
- 最大速度缩放因子：0.3
- 最大加速度缩放因子：0.3
- 规划时间：5.0 秒

### 2. 笛卡尔位姿规划

演示如何将末端执行器运动到指定的笛卡尔位姿：

```cpp
目标位姿（相对于 BASE_FRAME）：
- 位置：x=0.1m, y=0.2m, z=0.1m
- 姿态：绕 Y 轴旋转 180°（末端朝下）
```

**参数设置**：
- 最大速度缩放因子：0.3
- 最大加速度缩放因子：0.3
- 规划时间：8.0 秒
- 位置容差：5 mm
- 姿态容差：~0.57°

### 3. 笛卡尔直线路径规划

演示如何让末端执行器沿直线轨迹运动：

```cpp
路径点序列：
1. 起点：当前位姿
2. 路点1：沿 Z 轴下降 0.2m
3. 路点2：沿 X 轴前进 0.2m
4. 路点3：沿 Z 轴上升 0.2m（回到原始高度）
5. 终点：沿 X 轴后退 0.2m（回到原始位置）
```

**参数设置**：
- 最大速度缩放因子：0.2
- 最大加速度缩放因子：0.2
- 末端插值步长：1 cm
- 跳跃阈值：0.0（禁用跳跃检测）
- 路径完成率要求：≥90%

### 4. 回零位

演示如何将机械臂返回到初始零位：

- 优先使用 SRDF 中定义的命名目标（`home`、`zero` 或 `ready`）
- 如果未找到命名目标，则将所有关节置零

**参数设置**：
- 最大速度缩放因子：0.3
- 最大加速度缩放因子：0.3
- 规划时间：5.0 秒

## 📁 包结构

```
tl_moveit2_demo/
├── CMakeLists.txt              # 编译配置文件
├── package.xml                 # 包描述文件
├── README.md                   # 本文档
├── config/                     # 配置文件目录
├── include/                    # 头文件目录
│   └── tl_moveit2_demo/
│       └── moveit2_demo.hpp    # 演示节点头文件
├── launch/                     # 启动文件目录
│   └── moveit2_demo.launch.py  # 演示启动文件
└── src/                        # 源代码目录
    └── moveit2_demo.cpp        # 演示节点实现
```

## 📝 代码示例

### 自定义关节角度

修改 `src/moveit2_demo.cpp` 中的 `demoJointSpacePlanning()` 函数：

```cpp
std::vector<double> target_joints = {
    30.0 * M_PI / 180.0,   // J1 (30°)
    -45.0 * M_PI / 180.0,  // J2 (-45°)
    60.0 * M_PI / 180.0,   // J3 (60°)
    0.0 * M_PI / 180.0,    // J4 (0°)
    45.0 * M_PI / 180.0,   // J5 (45°)
    0.0 * M_PI / 180.0     // J6 (0°)
};
```

### 自定义笛卡尔路径

修改 `src/moveit2_demo.cpp` 中的 `demoCartesianPathPlanning()` 函数，添加自定义路径点：

```cpp
geometry_msgs::msg::Pose custom_waypoint;
custom_waypoint.position.x = 0.3;
custom_waypoint.position.y = 0.0;
custom_waypoint.position.z = 0.2;
custom_waypoint.orientation.x = 0.0;
custom_waypoint.orientation.y = 1.0;
custom_waypoint.orientation.z = 0.0;
custom_waypoint.orientation.w = 0.0;
waypoints.push_back(custom_waypoint);
```