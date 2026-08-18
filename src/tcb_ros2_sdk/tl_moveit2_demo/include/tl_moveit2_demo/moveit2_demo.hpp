#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// #include "moveit2/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h"
#include "../../../../moveit2/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h"
#include "../../../../moveit2/moveit_ros/planning_interface/planning_scene_interface/include/moveit/planning_scene_interface/planning_scene_interface.h"

/**
 * @brief MoveIt2 运动规划演示节点
 *
 * 演示内容：
 *   1. 关节空间规划（Joint Space Planning）
 *   2. 笛卡尔目标位姿规划（Pose Goal Planning）
 *   3. 笛卡尔直线路径规划（Cartesian Path Planning）
 *   4. 回零位（Named Target）
 */
class MoveIt2DemoNode : public rclcpp::Node
{
public:
    explicit MoveIt2DemoNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /// 节点构造完成后调用，执行全部演示流程
    void run();

private:
    // ── 演示步骤 ──────────────────────────────────────────────────────────────

    /// 步骤 1：关节空间规划 —— 将各关节运动到指定角度
    void demoJointSpacePlanning();

    /// 步骤 2：笛卡尔位姿规划 —— 将末端运动到指定位姿
    void demoPoseGoalPlanning();

    /// 步骤 3：笛卡尔直线路径规划 —— 末端沿直线轨迹运动
    void demoCartesianPathPlanning();

    /// 步骤 4：回零位
    void demoReturnToZero();

    // ── 内部工具 ──────────────────────────────────────────────────────────────

    /// 执行规划并运动，返回是否成功
    bool planAndExecute(moveit::planning_interface::MoveGroupInterface &move_group,
                        const std::string &step_name);

    /// 打印当前末端位姿
    void printCurrentPose(moveit::planning_interface::MoveGroupInterface &move_group);

    // ── 成员变量 ──────────────────────────────────────────────────────────────
    static constexpr const char *PLANNING_GROUP = "tcb610_06_group";
    static constexpr const char *BASE_FRAME = "tl_robot_link0";
    static constexpr const char *EEF_LINK = "tl_robot_link6";
};
