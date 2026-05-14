#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TLControlNode : public rclcpp::Node
{
public:
  TLControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  
  // 新增：初始化函数，在节点构造完成后调用
  void initialize();

private:
  /* ---------- callbacks (with arm id) ---------- */
  void targetPosCallback(const sensor_msgs::msg::JointState::SharedPtr msg, const std::string& arm_id);
  void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string& arm_id);
  void cartesianLineCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, const std::string& arm_id);

  /* ---------- internal ---------- */
  void initArm(const std::string& arm_id, const std::string& group_name);
  void createSubscriptionsForArm(const std::string& arm_id);
  bool isArmReady(const std::string& arm_id) const;

private:
  /* ---------- ROS subscriptions (per arm) ---------- */
  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> joint_motion_subs_;
  std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> cartesian_motion_subs_;
  std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> cartesian_line_subs_;

  /* ---------- MoveIt interfaces (per arm) ---------- */
  std::map<std::string, std::shared_ptr<moveit::planning_interface::MoveGroupInterface>> move_groups_;

  /* ---------- arm info (per arm) ---------- */
  std::map<std::string, std::vector<std::string>> joint_names_map_;
  std::map<std::string, std::string> base_frame_map_;  // 仅为兼容保留，不再用于帧检查

  /* ---------- TF2 ---------- */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /* ---------- runtime state ---------- */
  bool arms_initialized_{false};

  // 存储从参数读取的 arm_mode，供 initialize 使用
  std::string arm_mode_;
};