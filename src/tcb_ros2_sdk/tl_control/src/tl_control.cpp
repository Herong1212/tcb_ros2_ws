#include "tl_control/tl_control.hpp"

#include <memory>
#include <string>
#include <vector>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

TLControlNode::TLControlNode(const rclcpp::NodeOptions &options)
    : Node("tl_control", options)
{
    this->declare_parameter("arm_mode", "single");
    arm_mode_ = this->get_parameter("arm_mode").as_string();

    this->declare_parameter("planning_groups.single", "");
    this->declare_parameter("planning_groups.left", "");
    this->declare_parameter("planning_groups.right", "");

    RCLCPP_INFO(this->get_logger(), "Node constructed, waiting for initialize()...");
}

void TLControlNode::initialize()
{
    if (arms_initialized_)
        return;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (arm_mode_ == "single")
    {
        std::string group_name = this->get_parameter("planning_groups.single").as_string();
        if (group_name.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "planning_groups.single is empty!");
            return;
        }
        initArm("single", group_name);
        createSubscriptionsForArm("single");
    }
    else if (arm_mode_ == "dual")
    {
        std::string left_group = this->get_parameter("planning_groups.left").as_string();
        std::string right_group = this->get_parameter("planning_groups.right").as_string();
        if (left_group.empty() || right_group.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "planning_groups.left or .right is empty!");
            return;
        }
        initArm("left", left_group);
        initArm("right", right_group);
        createSubscriptionsForArm("left");
        createSubscriptionsForArm("right");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid arm_mode: %s", arm_mode_.c_str());
        return;
    }

    arms_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "tl_control node initialized with mode: %s", arm_mode_.c_str());
}

void TLControlNode::initArm(const std::string &arm_id, const std::string &group_name)
{
    // 创建 MoveGroupInterface
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), group_name);

    // 设置参考坐标系为规划框架
    std::string planning_frame = move_group->getPlanningFrame();
    move_group->setPoseReferenceFrame(planning_frame);

    // 设置规划参数（可根据需要参数化）
    move_group->setPlanningTime(3.0);                 // 设置运动规划器允许的最大规划时间（单位：秒）
    move_group->setNumPlanningAttempts(5);            // 设置规划尝试次数
    move_group->setMaxVelocityScalingFactor(0.5);     // 设置最大速度缩放因子（取值 0.0~1.0）
    move_group->setMaxAccelerationScalingFactor(0.5); // 设置最大加速度缩放因子（取值 0.0~1.0）

    // 获取关节名称列表
    std::vector<std::string> joint_names = move_group->getJointNames();

    move_groups_[arm_id] = move_group;
    joint_names_map_[arm_id] = joint_names;
    base_frame_map_[arm_id] = planning_frame;

    RCLCPP_INFO(this->get_logger(),
                "Initialized arm '%s' with group '%s', %zu joints, planning frame '%s'",
                arm_id.c_str(), group_name.c_str(), joint_names.size(), planning_frame.c_str());
}

void TLControlNode::createSubscriptionsForArm(const std::string &arm_id)
{
    // 根据 arm_id 生成话题前缀
    std::string prefix;
    if (arm_id == "left")
    {
        prefix = "armleft/";
    }
    else if (arm_id == "right")
    {
        prefix = "armright/";
    }
    else
    {
        prefix = ""; // single 模式无前缀
    }

    std::string joint_topic = "/tl_control/" + prefix + "joint_motion";
    std::string cartesian_topic = "/tl_control/" + prefix + "cartesian_motion";
    std::string line_topic = "/tl_control/" + prefix + "cartesian_linear_motion";

    joint_motion_subs_[arm_id] = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, 10,
        [this, arm_id](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            this->targetPosCallback(msg, arm_id);
        });

    cartesian_motion_subs_[arm_id] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        cartesian_topic, 10,
        [this, arm_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->targetPoseCallback(msg, arm_id);
        });

    cartesian_line_subs_[arm_id] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        line_topic, 10,
        [this, arm_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->cartesianLineCallback(msg, arm_id);
        });

    RCLCPP_INFO(this->get_logger(), "Subscribed to topics for arm '%s'", arm_id.c_str());
}

bool TLControlNode::isArmReady(const std::string &arm_id) const
{
    return arms_initialized_ && move_groups_.count(arm_id) > 0;
}

void TLControlNode::targetPosCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg,
    const std::string &arm_id)
{
    if (!isArmReady(arm_id))
    {
        RCLCPP_WARN(this->get_logger(), "Arm '%s' not initialized yet.", arm_id.c_str());
        return;
    }

    const auto &joint_names = joint_names_map_[arm_id];
    if (msg->position.size() != joint_names.size())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Target position size mismatch for arm '%s'. Expect %zu, got %zu",
            arm_id.c_str(), joint_names.size(), msg->position.size());
        return;
    }

    std::map<std::string, double> target_joint_values;
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        target_joint_values[joint_names[i]] = msg->position[i];
    }

    auto move_group = move_groups_[arm_id];
    move_group->setJointValueTarget(target_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group->plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Motion planning success for arm '%s'.", arm_id.c_str());
        move_group->execute(plan);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Motion planning failed for arm '%s'.", arm_id.c_str());
    }
}

void TLControlNode::targetPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    const std::string &arm_id)
{
    if (!isArmReady(arm_id))
    {
        RCLCPP_WARN(this->get_logger(), "Arm '%s' not initialized yet.", arm_id.c_str());
        return;
    }

    auto move_group = move_groups_[arm_id];
    std::string target_frame = move_group->getPlanningFrame();

    geometry_msgs::msg::PoseStamped transformed_pose;
    try
    {
        transformed_pose = tf_buffer_->transform(*msg, target_frame, tf2::durationFromSec(1.0));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to transform pose from '%s' to '%s' for arm '%s': %s",
                     msg->header.frame_id.c_str(), target_frame.c_str(), arm_id.c_str(), ex.what());
        return;
    }

    move_group->setPoseTarget(transformed_pose.pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group->plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Cartesian pose planning success for arm '%s'.", arm_id.c_str());
        move_group->execute(plan);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Cartesian pose planning failed for arm '%s'.", arm_id.c_str());
    }

    move_group->clearPoseTargets();
}

void TLControlNode::cartesianLineCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    const std::string &arm_id)
{
    if (!isArmReady(arm_id))
    {
        RCLCPP_WARN(this->get_logger(), "Arm '%s' not initialized yet.", arm_id.c_str());
        return;
    }

    auto move_group = move_groups_[arm_id];
    std::string target_frame = move_group->getPlanningFrame();

    geometry_msgs::msg::PoseStamped transformed_pose;
    try
    {
        transformed_pose = tf_buffer_->transform(*msg, target_frame, tf2::durationFromSec(1.0));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to transform pose from '%s' to '%s' for arm '%s': %s",
                     msg->header.frame_id.c_str(), target_frame.c_str(), arm_id.c_str(), ex.what());
        return;
    }

    geometry_msgs::msg::Pose start_pose = move_group->getCurrentPose().pose;
    geometry_msgs::msg::Pose target_pose = transformed_pose.pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;

    double fraction = move_group->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory, false);

    RCLCPP_INFO(
        this->get_logger(),
        "Cartesian path fraction for arm '%s': %.2f",
        arm_id.c_str(), fraction);

    if (fraction < 0.95)
    {
        RCLCPP_ERROR(this->get_logger(), "Cartesian path planning incomplete for arm '%s'.", arm_id.c_str());
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group->execute(plan);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TLControlNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}