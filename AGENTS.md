# Repository Guidelines

## 项目结构与模块组织

本仓库是面向 Ubuntu 22.04 的 ROS 2 Humble 工作空间。机器人代码位于 `src/tcb_ros2_sdk/`：

- `tl_driver`：控制器通信与状态发布。
- `tl_hardware`：ros2_control 硬件接口插件。
- `tl_control`：运动控制；`tl_bringup`：整机启动入口。
- `tl_robot_description`：机器人模型与网格资源。
- `tl_moveit2_config`：各型号的 MoveIt 规划配置。

`src/` 还包含 `moveit2`、`moveit_msgs`、`moveit_resources` 和 `ros_testing` 等依赖源码。修改这些依赖时遵循对应目录的贡献指南，避免无关改动。测试放在各包的 `test/` 下；`build/`、`install/` 和 `log/` 是生成目录，不要提交。

## 构建、测试与开发命令

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

上述命令依次加载 ROS 环境、安装依赖、构建并加载工作空间。内存不足时添加 `--executor sequential`。仅构建相关包及其依赖可使用：

```bash
colcon build --symlink-install --packages-up-to tl_bringup tl_control tl_driver tl_hardware
```

运行测试、查看结果及启动规划演示：

```bash
colcon test --packages-select tl_driver tl_bringup tl_robot_description
colcon test-result --verbose
ros2 launch tcb610_06_config demo.launch.py use_rviz:=true
```

启动演示前，检查配置中的硬件插件，确认使用预期的模拟或真实硬件。

## 代码风格与命名规范

Python 使用四空格缩进；C++ 保持所在文件的缩进与花括号风格。Python 函数、模块和 ROS 包使用 `snake_case`，C++ 接口沿用现有命名。启动文件命名为 `*.launch.py`，参数存放在包内的 `config/` 目录。

Python 使用 `ament_flake8` 和 `ament_pep257` 检查。MoveIt 有独立的 pre-commit 配置，包含 Black 和 clang-format-14；按对应组件的配置执行格式检查。

## 测试指南

Python 测试使用 pytest，文件放在包内 `test/` 目录并命名为 `test_*.py`。随仓库提供的 MoveIt 还使用 C++ GoogleTest 和启动测试。行为变更应补充回归检查，构建后运行受影响包的测试。仓库未规定统一的覆盖率门槛；提交说明中应单独列出跳过的测试和依赖真实硬件的验证。

## 提交与合并请求指南

现有提交历史使用 `add packages` 等简短标题，尚无正式格式约定。建议以简洁的动作描述作为标题，并注明受影响的包。

合并请求应说明行为变化、涉及的机器人型号、相关问题，以及构建和测试命令与结果。涉及模型或可视化变化时，附上 RViz 截图。

## 硬件配置注意事项

运行前核对控制器 IP、通信协议、自由度和 ros2_control 插件。开发时优先使用 `mock_components/GenericSystem`；部分配置默认连接真实硬件。修改配置后重新构建对应包，并重新加载工作空间环境。
