# tl_hardware 中文说明

## 1. 功能简介
`tl_hardware` 是天链 TCB 系列机械臂的 ROS2 硬件接口包，基于 `ros2_control` 框架实现。该包实现了 `hardware_interface::SystemInterface` 接口，用于连接 ROS2 control 框架与底层 `tl_driver` 驱动，实现关节状态的读取和关节命令的下发。

主要特点：
- 实现 ros2_control 硬件接口标准
- 支持关节位置、速度、力矩读写
- 与 tl_driver 无缝集成
- 支持多种轨迹指令来源模式（Controller、External、Auto）

## 2. 包结构
```
tl_hardware/
├── src/
│   └── tl_hardware_interface.cpp     # 硬件接口主实现（C++）
├── include/
│   └── tl_hardware/
│       └── tl_hardware_interface.hpp # 硬件接口头文件
├── tl_hardware_interface.xml         # 插件描述文件
├── CMakeLists.txt                    # CMake 构建配置
├── package.xml                       # ROS2 包配置
└── README_CN.md                      # 本文件
```

## 3. 主要功能
- **硬件接口实现**：实现 `hardware_interface::SystemInterface` 接口
- **状态读取**：从 tl_driver 读取关节状态（位置、速度、力矩）
- **命令下发**：将控制命令下发给 tl_driver
- **多模式支持**：支持三种轨迹指令来源模式
- **生命周期管理**：支持 ros2_control 的完整生命周期（init、configure、activate、deactivate、cleanup）

## 4. 依赖包
- `rclcpp`：ROS2 C++ 客户端库
- `rclcpp_lifecycle`：ROS2 生命周期库
- `std_msgs`：标准消息类型
- `hardware_interface`：ros2_control 硬件接口库
- `pluginlib`：插件库
- `sensor_msgs`：传感器消息类型
- `trajectory_msgs`：轨迹消息类型

## 5. 运行环境
- Ubuntu 22.04
- ROS2 Humble
- C++14 或更高版本
- ros2_control 框架
- 已正确配置 ROS2 工作空间

## 6. 编译与安装

在工作空间根目录执行：

```bash
cd ~/tcb_ros2_ws
colcon build --packages-select tl_hardware
source install/setup.bash
```

## 7. 硬件接口原理

### 7.1 系统架构

```
ROS2 Controller Framework
       ↓
hardware_interface (tl_hardware)
       ↓
tl_driver (底层驱动)
       ↓
机械臂硬件
```

### 7.2 数据流向

**读取流程（Read）**：
```
机械臂状态 → tl_driver (发布关节状态) → tl_hardware (读取) → ROS2 Framework
```

**写入流程（Write）**：
```
ROS2 Controller → hardware_interface → tl_hardware → tl_driver (发布轨迹) → 机械臂执行
```

## 8. 轨迹指令模式

硬件接口支持三种轨迹指令来源模式：

### 8.1 CONTROLLER 模式
- 仅使用 ROS2 controller 的关节命令
- 适用于标准的 ros2_control 控制框架
- 默认运行模式

### 8.2 EXTERNAL 模式
- 仅使用外部话题 `/teleop/target_position` 的命令
- 适用于遥操作或外部规划器
- 外部命令优先级最高

### 8.3 AUTO 模式
- 自动切换模式
- 有外部指令时使用 EXTERNAL 模式
- 外部指令超时（默认 200ms）后自动切回 CONTROLLER 模式
- 提供灵活的混合控制

### 8.4 模式配置

在 URDF 文件中配置硬件参数（示例）：

```xml
<hardware>
  <plugin>tl_hardware/TlHardwareInterface</plugin>
  <param name="trajectory_mode">AUTO</param>
  <!-- 目前只有位置控制 -->
  <param name="control_mode">position</param>
</hardware>
```

## 9. 状态和命令接口

### 9.1 状态接口（State Interface）

导出的状态接口包括：

- `<joint_name>/position`：关节位置（rad）
- `<joint_name>/velocity`：关节速度（rad/s）
- `<joint_name>/effort`：关节力矩（N·m）

### 9.2 命令接口（Command Interface）

导出的命令接口包括：

- `<joint_name>/position`：关节位置命令（rad）
- `<joint_name>/velocity`：关节速度命令（rad/s）
- `<joint_name>/effort`：关节力矩命令（N·m）

## 10. 话题接口

### 10.1 订阅话题

- `/tl_driver/current_joint_states` (`sensor_msgs/msg/JointState`)：关节状态
- `/teleop/target_position` (`std_msgs/msg/Float64MultiArray`)：外部位置命令（AUTO/EXTERNAL 模式）

### 10.2 发布话题

- `/tl_driver/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`)：关节轨迹指令

## 11. 生命周期管理

硬件接口遵循 ros2_control 的完整生命周期：

```
┌─────────────┐
│   Unconfigured (初始状态)
└──────┬──────┘
       │ on_configure()
       ↓
┌─────────────┐
│   Inactive   │
└──────┬──────┘
       │ on_activate()
       ↓
┌─────────────┐
│   Active (可读写)
└──────┬──────┘
       │ on_deactivate()
       ↓
┌─────────────┐
│   Inactive   │
└──────┬──────┘
       │ on_cleanup()
       ↓
┌─────────────┐
│   Unconfigured
└─────────────┘
```

## 12. 读写操作

### 12.1 read() 方法
- 从 tl_driver 订阅关节状态
- 更新所有关节的位置、速度、力矩
- 称由 ros2_control 框架周期性调用

### 12.2 write() 方法
- 根据选定的轨迹指令模式获取关节命令
- 检查 AUTO 模式的外部指令超时
- 发布轨迹到 `/tl_driver/joint_trajectory`
- 由 ros2_control 框架周期性调用

## 13. 调试技巧

### 13.1 查看硬件接口状态
```bash
ros2 control list_hardware_interfaces
```

### 13.2 查看关节状态
```bash
ros2 topic echo /tl_driver/current_joint_states
```

### 13.3 查看下发轨迹
```bash
ros2 topic echo /tl_driver/joint_trajectory
```

## 14. 常见问题

### 问题 1：硬件接口无法加载
**原因**：插件库未正确编译或路径配置错误

**解决方案**：
- 重新编译：`colcon build --packages-select tl_hardware`
- 检查 `tl_hardware_interface.xml` 文件是否正确安装
- 查看详细错误日志

### 问题 2：无法读取关节状态
**原因**：tl_driver 未正常启动或话题名称配置不匹配

**解决方案**：
- 确认 tl_driver 已启动：`ros2 node list | grep tl_driver`
- 检查话题是否发布：`ros2 topic list | grep joint_states`
- 验证话题名称与代码中的订阅一致

### 问题 3：关节命令无法下发
**原因**：硬件接口未激活或轨迹话题未连接

**解决方案**：
- 检查硬件接口状态：`ros2 control list_hardware_interfaces`
- 确认硬件接口处于 Active 状态
- 查看 `/tl_driver/joint_trajectory` 话题是否有数据发布

### 问题 4：AUTO 模式外部指令不工作
**原因**：外部话题名称或消息格式不匹配

**解决方案**：
- 检查外部话题是否发布：`ros2 topic list | grep teleop`
- 验证话题消息类型为 `std_msgs/msg/Float64MultiArray`
- 检查超时时间设置是否合理

## 15. 性能优化建议

1. **采样频率**：根据控制要求调整 ros2_control 循环频率（通常 50-500Hz）
2. **同步机制**：使用互斥锁保护共享数据，避免竞态条件
3. **话题缓冲**：可在启动文件中调整话题队列大小
4. **实时性**：在关键场景使用实时线程优先级

## 16. 相关文档

更详细的信息请参考：
- [tl_driver](../tl_driver/README_CN.md)：底层驱动包说明
- [tl_bringup](../tl_bringup/README_CN.md)：启动包说明
- [ROS2 Control 官方文档](https://control.ros.org/)
- [ROS2 Control Hardware Interface](https://control.ros.org/master/doc/hardware_interface/doc/hardware_interface_types.html)

## 17. 许可证

TODO: License declaration (待确认许可证)
