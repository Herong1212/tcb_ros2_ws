# tl_robot_description 中文说明

## 1. 功能简介
`tl_robot_description` 是天链 TCB 系列机械臂的描述包，用于发布 URDF 模型、TF 变换以及相关可视化。该包帮助用户在 RViz 和其他工具中直观查看机械臂模型，并为 MoveIt2、控制和仿真模块提供基础描述。

## 2. 包结构
```
tl_robot_description/
├── urdf/                            # Xacro/URDF 模型
├── launch/                          # 启动文件
│   ├── tcb605_05.launch.py
│   ├── tcb610_06.launch.py
│   ├── tcb705_05.launch.py
│   └── tcb710_06.launch.py
├── meshes/                          # 机械臂资源
├── resource/                        # 资源索引
├── package.xml                      # ROS2 包配置
├── setup.py                         # Python 安装脚本
├── setup.cfg                        # 安装配置
├── test/                            # 单元测试（如果有）
└── README_CN.md                     # 本文件
```

## 3. 支持的型号
- TCB605_05
- TCB610_06
- TCB705_05
- TCB710_06

每个型号都有对应的 URDF xacro 文件，可通过 `<arm_version>` 参数选择。

## 4. 运行环境
- Ubuntu 22.04
- ROS2 Humble
- Python3
- 已正确配置 ROS2 工作空间

## 5. 使用方法

### 5.1 发布机器人模型与 TF

```bash
ros2 launch tl_robot_description tcb<arm_version>.launch.py
```

例如：
```bash
ros2 launch tl_robot_description tcb605_05.launch.py
ros2 launch tl_robot_description tcb710_06.launch.py
```

启动后会发布 `/robot_description` 参数并开始广播关节 TF 变换，可在 RViz 中加载模型查看。

### 5.2 在 RViz 中查看

往 RViz 添加：
- RobotModel（从 `/robot_description` 读取）
- TF 显示以观察坐标变换

下图为启动成功后的界面示例：

![image](doc/tl_description2.png)

## 6. 与其它包的配合
- `tl_bringup`：通常会在 bringup 启动文件中包含此描述包
- `tl_moveit2_config`：描述包为 MoveIt2 配置提供 URDF/TF
- `tl_control`/`tl_driver`：控制和驱动模块引用机器人描述中的关节名称

## 7. 自定义和扩展
- 修改 `urdf/` 目录下的 `.xacro` 文件以更改模型
- 若需添加传感器或末端执行器，可在 xacro 中包含新的子组件
- Meshes 放在 `meshes/` 中，可替换为更高精度模型

## 8. 编译与安装
在工作空间根目录执行：

```bash
cd ~/tcb_ros2_ws
colcon build --packages-select tl_robot_description
source install/setup.bash
```

## 9. 常见问题

### 启动后看不到模型
- 确认 `/robot_description` 参数是否存在：
  ```bash
  ros2 param get /<node> robot_description
  ```
- 在 RViz 中选择正确的 `RobotModel` 主题
- xacro 文件语法错误：运行 `ros2 run xacro xacro --inorder -o /dev/null <file>` 验证

### TF 不发布
- 查看是否有 `/tf` 话题：
  ```bash
  ros2 topic list | grep tf
  ```
- 验证关节状态发布与描述一致

## 10. 许可证
Apache License 2.0
