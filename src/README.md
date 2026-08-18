# TCB ROS 2 工作空间

天链 TCB 系列机械臂的 ROS 2 Humble 工作空间，集成机械臂驱动、机器人模型、`ros2_control` 硬件接口、MoveIt 2 配置、整机启动和运动规划示例。支持单臂及双臂复合机器人。

> 操作真实机械臂前，请先阅读[安全注意事项](#安全注意事项)，核对控制器协议、自由度和网络参数，并以低速完成首次联调。

## 支持环境

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2 / `ros2_control`
- Python 3
- TCB 控制器通信协议：2207 或 2403

仓库已在 `src/` 中包含 MoveIt 2 和 `ros_testing` 源码。如果改为使用系统安装的 MoveIt 2，请避免同时构建两套同名包。

## 支持型号

| 型号 | 配置包 | 自由度 | 额定负载 |
| --- | --- | ---: | ---: |
| TCB605_05 | `tcb605_05_config` | 6 | 5 kg |
| TCB610_06 | `tcb610_06_config` | 6 | 6 kg |
| TCB705_05 | `tcb705_05_config` | 7 | 5 kg |
| TCB710_06 | `tcb710_06_config` | 7 | 6 kg |
| 双臂复合机器人 | `composite_robot_config` | 由两臂配置决定 | — |

## 工作空间结构

```text
tcb_ros2_ws/
├── src/
│   ├── tcb_ros2_sdk/
│   │   ├── tl_driver/             # 控制器通信、状态发布和轨迹下发
│   │   ├── tl_hardware/           # ros2_control SystemInterface 插件
│   │   ├── tl_control/            # 基于 MoveIt 2 的运动控制节点
│   │   ├── tl_bringup/            # 单臂/双臂整机启动入口
│   │   ├── tl_robot_description/  # URDF、网格模型和 TF/RViz 启动文件
│   │   ├── tl_moveit2_config/     # 各型号 MoveIt 2 配置
│   │   └── tl_moveit2_demo/       # TCB610_06 仿真规划示例
│   ├── moveit2/                   # MoveIt 2 源码
│   └── ros_testing/               # ROS 2 测试依赖
├── build/                         # colcon 构建产物（构建后生成）
├── install/                       # 安装空间（构建后生成）
└── log/                           # colcon 日志（构建后生成）
```

各功能包的详细接口和配置说明见 [`src/tcb_ros2_sdk/README_CN.md`](src/tcb_ros2_sdk/README_CN.md)。

## 安装依赖

1. 安装 ROS 2 Humble，并在当前终端加载环境：

   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. 在工作空间根目录安装可解析的系统依赖：

   ```bash
   sudo rosdep init          # 本机首次使用 rosdep 时执行
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. 可选的 Python 运动学依赖：

   ```bash
   python3 -m pip install --user pin==3.7.0 numpy==1.24.0 scipy==1.8.0
   ```

   `pin`（Pinocchio）仅在相关运动学功能需要时安装。若当前环境已有兼容版本，建议不要强制降级。

## 编译

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

机器内存有限时，可减少并行任务数：

```bash
colcon build --symlink-install --executor sequential
```

只构建 TCB 相关包及其依赖：

```bash
colcon build --symlink-install \
  --packages-up-to tl_bringup tl_control tl_driver tl_hardware \
                    tl_robot_description tl_moveit2_demo
```

每次新开终端都需要加载环境：

```bash
source /opt/ros/humble/setup.bash
source <工作空间绝对路径>/install/setup.bash
```

## 快速开始：仿真

MoveIt 2 配置中的硬件插件决定启动后连接真机还是使用模拟硬件。当前源码状态如下：

| 配置包 | 默认硬件接口 |
| --- | --- |
| `tcb610_06_config`、`tcb710_06_config` | `mock_components/GenericSystem`（仿真） |
| `tcb605_05_config`、`tcb705_05_config`、`composite_robot_config` | `tl_hardware/TlHardwareInterface`（真机） |

以默认采用模拟硬件的 TCB610_06 为例：

```bash
ros2 launch tcb610_06_config demo.launch.py use_rviz:=true
```

在 RViz 的 MotionPlanning 面板中设置目标位姿，先执行 **Plan**，确认轨迹无碰撞后再执行 **Execute**。

运行仓库自带的 TCB610_06 规划示例时，先保持上述终端运行，再开一个已加载工作空间环境的终端执行：

```bash
ros2 launch tl_moveit2_demo moveit2_demo.launch.py
```

其他型号也可运行其 `demo.launch.py`，但必须先检查对应的 `*.ros2_control.xacro` 是否使用 `mock_components/GenericSystem`。切换插件后需重新编译相应配置包并重新加载环境。

## 连接真实机械臂

### 1. 配置驱动

编辑 [`src/tcb_ros2_sdk/tl_driver/config/tl_driver_config.yaml`](src/tcb_ros2_sdk/tl_driver/config/tl_driver_config.yaml)，至少核对：

- `arm_mode`：`single` 或 `dual`
- `ip1` / `ip2`：机械臂控制器 IP
- `sdk_version1` / `sdk_version2`：`2207` 或 `2403`
- `dof1` / `dof2`：6 或 7
- `arm_control_mode1` / `arm_control_mode2`：`queue`、`motion_control` 或 `servo_j`
- `speed1` / `speed2`：首次联调建议使用 10～30

双臂配置中左右臂参数后缀分别为 `1` 和 `2`。修改 YAML 后需重新构建 `tl_driver`，因为启动文件读取的是安装空间中的配置：

```bash
colcon build --symlink-install --packages-select tl_driver
source install/setup.bash
```

### 2. 选择真实硬件插件

打开对应配置包 `config/` 目录中的 `*.ros2_control.xacro`，启用：

```xml
<plugin>tl_hardware/TlHardwareInterface</plugin>
<param name="control_mode">position</param>
<param name="trajectory_mode">auto</param>
```

并注释掉：

```xml
<plugin>mock_components/GenericSystem</plugin>
```

`trajectory_mode` 可选值：

- `controller`：只采用 `ros2_control` 控制器命令
- `external`：只采用 `/teleop/target_position` 外部命令
- `auto`：有外部命令时优先采用，超时后返回控制器命令

重新构建所选配置包，例如：

```bash
colcon build --symlink-install --packages-select tcb710_06_config
source install/setup.bash
```

### 3. 启动整机

`tl_bringup` 会依次启动驱动、对应 MoveIt 2 环境和 `tl_control`。单臂示例：

```bash
ros2 launch tl_bringup tcb710_06_bringup.launch.py use_rviz:=true
```

可替换为：

- `tcb605_05_bringup.launch.py`
- `tcb610_06_bringup.launch.py`
- `tcb705_05_bringup.launch.py`
- `tcb710_06_bringup.launch.py`

双臂启动：

```bash
ros2 launch tl_bringup composite_robot_bringup.launch.py use_rviz:=true
```

确保 `tl_driver_config.yaml`、`tl_control_config.yaml` 和所选 MoveIt 配置的单/双臂模式、自由度及规划组相互一致。

## 常用操作

查看节点、控制器和关节状态：

```bash
ros2 node list
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /tl_driver/current_joint_states
```

上电、下电与调速：

```bash
ros2 topic pub --once /tl_driver/cmd std_msgs/msg/String "data: 'arm_power_on'"
ros2 topic pub --once /tl_driver/cmd std_msgs/msg/String "data: 'arm_power_off'"
ros2 topic pub --once /tl_driver/set_speed std_msgs/msg/Int32 "data: 20"
```

双臂可使用 `armleft_power_on`、`armright_power_on` 等命令分别控制，并通过 `/tl_driver/armleft/set_speed`、`/tl_driver/armright/set_speed` 分别调速。

## 验证与测试

构建后检查 TCB 包是否被正确发现：

```bash
colcon list | grep -E '^(tl_|tcb|composite_robot_config)'
ros2 pkg executables tl_driver
ros2 pkg executables tl_control
```

运行包测试并查看结果：

```bash
colcon test --packages-select tl_driver tl_bringup tl_robot_description
colcon test-result --verbose
```

## 常见问题

### 找不到功能包或 launch 文件

确认当前终端先加载 ROS 2，再加载本工作空间，并检查目标包是否成功构建：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg list | grep '^tl_'
```

### 无法连接机械臂

确认机械臂已上电，主机和控制器位于可达网段，随后检查 YAML 中的 IP、6001/7000 端口和 SDK 协议版本。可先用 `ping <机械臂IP>` 验证基础网络连通性。

### RViz 中没有模型或状态不更新

检查 `/robot_description`、`/joint_states`、`/tf` 和驱动关节状态：

```bash
ros2 topic list | grep -E 'joint|tf|robot_description'
ros2 topic hz /tl_driver/current_joint_states
```

同时确认模型关节名、驱动自由度和 MoveIt 配置属于同一型号。

### 控制器未激活或轨迹不执行

```bash
ros2 control list_controllers
ros2 topic echo /tl_driver/joint_trajectory
```

确认硬件插件已正确加载、控制器为 `active`，并检查 `trajectory_mode` 是否允许当前命令来源。

## 安全注意事项

- 保证机械臂、底座和末端工具安装牢固，线缆不会进入运动范围。
- 确认急停开关有效，操作人员能够随时触及。
- 首次运行使用低速、小范围目标，并先在仿真中验证轨迹。
- 真机执行前清空工作空间，人员与障碍物保持安全距离。
- 检查目标位姿、坐标系、关节顺序和角度单位；ROS 接口中的关节位置使用弧度。
- 不要同时运行多个可能向同一机械臂下发命令的节点。

## 子包文档

- [`tcb_ros2_sdk` 总览](src/tcb_ros2_sdk/README_CN.md)
- [`tl_driver`](src/tcb_ros2_sdk/tl_driver/README_CN.md)
- [`tl_hardware`](src/tcb_ros2_sdk/tl_hardware/README_CN.md)
- [`tl_control`](src/tcb_ros2_sdk/tl_control/README_CN.md)
- [`tl_bringup`](src/tcb_ros2_sdk/tl_bringup/README_CN.md)
- [`tl_robot_description`](src/tcb_ros2_sdk/tl_robot_description/README_CN.md)
- [`tl_moveit2_config`](src/tcb_ros2_sdk/tl_moveit2_config/README_CN.md)
- [`tl_moveit2_demo`](src/tcb_ros2_sdk/tl_moveit2_demo/README.md)

## 许可证

各组件的许可证以对应源码目录中的 `package.xml`、`LICENSE` 或上游项目声明为准。部分 TCB 子包的 `package.xml` 尚未填写最终许可证，请在分发或商用前向项目维护者确认授权条款。
