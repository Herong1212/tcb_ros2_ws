/**
 * @file moveit2_demo.cpp
 * @brief MoveIt2 运动规划演示 —— TCB610_06N 六轴机械臂
 *
 * 演示内容：
 *   1. 关节空间规划（Joint Space Planning）
 *   2. 笛卡尔目标位姿规划（Pose Goal Planning）
 *   3. 笛卡尔直线路径规划（Cartesian Path Planning）
 *   4. 回零位（Named Target）
 */

#include "tl_moveit2_demo/moveit2_demo.hpp"

#include <cmath>
#include <chrono>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

// ─────────────────────────────────────────────────────────────────────────────
// 构造函数
// ─────────────────────────────────────────────────────────────────────────────
MoveIt2DemoNode::MoveIt2DemoNode(const rclcpp::NodeOptions &options)
    : Node("moveit2_demo_node", options)
{
    RCLCPP_INFO(get_logger(), "MoveIt2DemoNode 已创建，调用 run() 开始演示。");
}

// ─────────────────────────────────────────────────────────────────────────────
// run() —— 顺序执行全部演示步骤
// ─────────────────────────────────────────────────────────────────────────────
void MoveIt2DemoNode::run()
{
    RCLCPP_INFO(get_logger(), "========== MoveIt2 演示开始 ==========");

    // demoJointSpacePlanning(); // 将 6 个关节移动到指定位置
    std::this_thread::sleep_for(1s);

    // demoPoseGoalPlanning(); // 末端运动到指定笛卡尔位姿
    std::this_thread::sleep_for(1s);

    demoCartesianPathPlanning(); // 末端沿直线折线轨迹运动
    std::this_thread::sleep_for(1s);

    // demoReturnToZero(); // 回零位s

    RCLCPP_INFO(get_logger(), "========== MoveIt2 演示结束 ==========");
}

// ─────────────────────────────────────────────────────────────────────────────
// 步骤 1：关节空间规划
// ─────────────────────────────────────────────────────────────────────────────
/**
 * @brief 执行关节空间规划演示
 *
 * 该函数演示如何将机械臂的6个关节移动到指定的目标角度。主要流程包括：
 * - 1、初始化 MoveGroup 接口并配置运动参数（速度、加速度缩放因子和规划时间）
 * - 2、获取当前关节状态并设置目标关节角度
 * - 3、根据实际关节数量调整目标角度向量
 * - 4、执行路径规划并移动到目标位置
 *
 * 目标关节角度设置为：
 * - J1: 90° (π/2 弧度)
 * - J2: -45° (-π/4 弧度)
 * - J3: 60° (π/3 弧度)
 * - J4: 20° (π/9 弧度)
 * - J5: 35° (约0.61弧度)
 * - J6: 0° (0 弧度)
 */
void MoveIt2DemoNode::demoJointSpacePlanning()
{
    RCLCPP_INFO(get_logger(), "---------- [1/4] 关节空间规划 ----------");

    // 创建MoveGroup接口并配置运动规划参数
    MoveGroupInterface move_group(shared_from_this(), PLANNING_GROUP);
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setPlanningTime(5.0);

    // 获取当前关节值并修改
    std::vector<double> joint_values = move_group.getCurrentJointValues();
    RCLCPP_INFO(get_logger(), "当前关节数量：%zu", joint_values.size());

    // 目标关节角度（单位：弧度）
    // J1=30°, J2=-45°, J3=60°, J4=0°, J5=45°, J6=0°
    std::vector<double> target_joints = {
        90.0 * M_PI / 180.0,  // J1
        -45.0 * M_PI / 180.0, // J2
        60.0 * M_PI / 180.0,  // J3
        20.0 * M_PI / 180.0,  // J4
        35.0 * M_PI / 180.0,  // J5
        0.0 * M_PI / 180.0    // J6
    };

    // 若关节数量不足 6 则截断
    if (joint_values.size() < target_joints.size())
    {
        target_joints.resize(joint_values.size());
    }

    move_group.setJointValueTarget(target_joints);
    printCurrentPose(move_group);

    if (planAndExecute(move_group, "关节空间规划"))
    {
        RCLCPP_INFO(get_logger(), "关节空间规划执行成功。");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 步骤 2：笛卡尔位姿规划
// ─────────────────────────────────────────────────────────────────────────────
void MoveIt2DemoNode::demoPoseGoalPlanning()
{
    RCLCPP_INFO(get_logger(), "---------- [2/4] 笛卡尔位姿规划 ----------");

    MoveGroupInterface move_group(shared_from_this(), PLANNING_GROUP);
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setPlanningTime(8.0);
    move_group.setEndEffectorLink(EEF_LINK);

    // 设置目标位姿（相对于 BASE_FRAME）
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.1;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.1;
    // 末端朝下（绕 Y 轴旋转 180°）
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 1.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;

    move_group.setPoseTarget(target_pose, EEF_LINK);
    move_group.setGoalPositionTolerance(0.005);   // 5 mm
    move_group.setGoalOrientationTolerance(0.01); // ~0.57°

    RCLCPP_INFO(get_logger(),
                "目标位姿 → x=%.3f  y=%.3f  z=%.3f",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);

    if (planAndExecute(move_group, "笛卡尔位姿规划"))
    {
        RCLCPP_INFO(get_logger(), "笛卡尔位姿规划执行成功。");
        printCurrentPose(move_group);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 步骤 3：笛卡尔直线路径规划
// ─────────────────────────────────────────────────────────────────────────────
void MoveIt2DemoNode::demoCartesianPathPlanning()
{
    RCLCPP_INFO(get_logger(), "---------- [3/4] 笛卡尔直线路径规划 ----------");

    MoveGroupInterface move_group(shared_from_this(), PLANNING_GROUP);
    move_group.setMaxVelocityScalingFactor(0.2);     // 最大速度为额定速度的 20%
    move_group.setMaxAccelerationScalingFactor(0.2); // 最大加速度为额定加速度的 20%
    move_group.setEndEffectorLink(EEF_LINK);

    // * 以当前位姿为起点，构造一段 "起点 —> Z 轴下降 —> X 轴前进 —> Z 轴上升 —> 终点" 的折线
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose(EEF_LINK).pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;

    // 第一段
    geometry_msgs::msg::Pose wp1 = current_pose; // 起始点：当前位姿 current_pose
    wp1.position.z -= 0.20;                      // 动作：沿 Z 轴负方向移动 0.20 米（20 厘米）
    waypoints.push_back(wp1);

    // 第二段
    geometry_msgs::msg::Pose wp2 = wp1; // 起始点：第一段的终点 wp1
    wp2.position.x += 0.20;             // 动作：沿 X 轴正方向移动 0.20 米（20 厘米）
    waypoints.push_back(wp2);

    // 第三段（回到原始高度）
    geometry_msgs::msg::Pose wp3 = wp2; // 起始点：第二段的终点 wp2
    wp3.position.z += 0.20;             // 动作：沿 Z 轴正方向移动 0.20 米（20 厘米）
    waypoints.push_back(wp3);

    // 第四段（回到原始位置）
    geometry_msgs::msg::Pose wp4 = wp3; // 起始点：第三段的终点 wp3
    wp4.position.x -= 0.20;             // 动作：沿 X 轴负方向移动 0.20 米（20 厘米），即：回到初始位置
    waypoints.push_back(wp4);

    RCLCPP_INFO(get_logger(), "笛卡尔路径共 %zu 个路点。", waypoints.size());

    // 计算笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;                                                                       // 末端插值步长 1 cm，控制路径点的密度
    const double jump_threshold = 0.0;                                                                  // 禁用跳跃检测（仿真中常设为 0），允许较大的关节角度变化
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); // 路径完成率，要求 ≥90% 才执行
    RCLCPP_INFO(get_logger(), "笛卡尔路径规划完成率：%.1f%%", fraction * 100.0);

    if (fraction < 0.9)
    {
        RCLCPP_WARN(get_logger(), "路径完成率不足 90%%，跳过执行。");
        return;
    }

    // 执行轨迹
    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto result = move_group.execute(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(get_logger(), "笛卡尔直线路径执行成功。");
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "笛卡尔直线路径执行失败，错误码：%d",
                     static_cast<int>(result.val));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 步骤 4：回零位
// ─────────────────────────────────────────────────────────────────────────────
void MoveIt2DemoNode::demoReturnToZero()
{
    RCLCPP_INFO(get_logger(), "---------- [4/4] 回零位 ----------");

    MoveGroupInterface move_group(shared_from_this(), PLANNING_GROUP);
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setPlanningTime(5.0);

    // 尝试使用 SRDF 中定义的 "home" 命名目标
    const std::vector<std::string> named_targets = move_group.getNamedTargets();
    bool has_home = false;
    for (const auto &t : named_targets)
    {
        if (t == "home" || t == "zero" || t == "ready")
        {
            has_home = true;
            move_group.setNamedTarget(t);
            RCLCPP_INFO(get_logger(), "使用命名目标 '%s' 回零。", t.c_str());
            break;
        }
    }

    if (!has_home)
    {
        // 若无命名目标，则将所有关节置零
        RCLCPP_INFO(get_logger(), "未找到命名零位，将所有关节置零。");
        std::vector<double> zero_joints(
            move_group.getCurrentJointValues().size(), 0.0);
        move_group.setJointValueTarget(zero_joints);
    }

    if (planAndExecute(move_group, "回零位"))
    {
        RCLCPP_INFO(get_logger(), "机械臂已回零位。");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 内部工具：规划并执行
// ─────────────────────────────────────────────────────────────────────────────
bool MoveIt2DemoNode::planAndExecute(
    MoveGroupInterface &move_group,
    const std::string &step_name)
{
    MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        RCLCPP_ERROR(get_logger(), "[%s] 规划失败！", step_name.c_str());
        return false;
    }

    RCLCPP_INFO(get_logger(), "[%s] 规划成功，开始执行……", step_name.c_str());
    auto exec_result = move_group.execute(plan);

    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "[%s] 执行失败，错误码：%d",
                     step_name.c_str(), static_cast<int>(exec_result.val));
        return false;
    }

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 内部工具：打印当前末端位姿
// ─────────────────────────────────────────────────────────────────────────────
void MoveIt2DemoNode::printCurrentPose(MoveGroupInterface &move_group)
{
    const auto stamped = move_group.getCurrentPose(EEF_LINK);
    const auto &p = stamped.pose.position;
    const auto &q = stamped.pose.orientation;
    RCLCPP_INFO(get_logger(),
                "当前末端位姿 → pos(%.4f, %.4f, %.4f)  quat(%.4f, %.4f, %.4f, %.4f)",
                p.x, p.y, p.z, q.x, q.y, q.z, q.w);
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // MoveGroupInterface 需要 SingleThreadedExecutor 之外的执行器
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<MoveIt2DemoNode>(options);

    // 在独立线程中运行 Executor，以便 MoveGroupInterface 的回调可以被处理
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]()
                            { executor.spin(); });

    // 稍等片刻，让 MoveGroup 连接到 move_group 服务
    std::this_thread::sleep_for(std::chrono::seconds(2));

    node->run();

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
