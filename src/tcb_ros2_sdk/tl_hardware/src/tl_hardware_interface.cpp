#include "tl_hardware/tl_hardware_interface.hpp"

#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>

using namespace std::chrono_literals;

namespace tl_hardware {

TlHardwareInterface::TlHardwareInterface()
    : last_external_command_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
      using_external_in_auto_(false),
      node_(nullptr),
      node_thread_(),
      info_(),
      logger_(rclcpp::get_logger("tl_hardware_interface")),
      control_mode_(ControlMode::POSITION),
      hardware_connected_(false),
      hardware_powered_(false),
      new_joint_state_(false),
      data_mutex_(),
      last_joint_state_time_(rclcpp::Time(0, 0, RCL_ROS_TIME)),
      last_command_time_(rclcpp::Time(0, 0, RCL_ROS_TIME))
{
}

TlHardwareInterface::~TlHardwareInterface() {
    if (node_thread_.joinable()) {
        if (node_) {
            node_.reset();
        }
        node_thread_.join();
    }
}

hardware_interface::CallbackReturn TlHardwareInterface::on_init(
    const hardware_interface::HardwareInfo & info) {
    
    RCLCPP_INFO(logger_, "Initializing TL hardware interface");
    
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    info_ = info;
    
    size_t joint_count = info_.joints.size();
    
    joint_names_.clear();
    for (size_t i = 0; i < joint_count; ++i) {
        joint_names_.push_back(info_.joints[i].name);
    }

    RCLCPP_INFO(logger_, "Found %zu joints:", joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        RCLCPP_INFO(logger_, "  Joint %zu: %s", i, joint_names_[i].c_str());
    }
    
    joint_positions_.resize(joint_count, 0.0);
    joint_velocities_.resize(joint_count, 0.0);
    joint_efforts_.resize(joint_count, 0.0);
    joint_position_commands_.resize(joint_count, 0.0);
    joint_velocity_commands_.resize(joint_count, 0.0);
    joint_effort_commands_.resize(joint_count, 0.0);
    
    RCLCPP_INFO(logger_, "Initialized TL hardware interface with %zu joints", joint_count);
    for (size_t i = 0; i < joint_count; ++i) {
        RCLCPP_INFO(logger_, "  Joint %zu: %s", i + 1, joint_names_[i].c_str());
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TlHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    
    RCLCPP_INFO(logger_, "Configuring TL hardware interface");
    
    try {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        
        auto node_options = rclcpp::NodeOptions();
        node_options.allow_undeclared_parameters(true);
        node_options.automatically_declare_parameters_from_overrides(true);
        
        node_ = std::make_shared<rclcpp::Node>("tl_hardware", node_options);
        
        power_pub_ = node_->create_publisher<std_msgs::msg::String>(
            "tl_driver/cmd", 
            rclcpp::QoS(10).reliable());

        trajectory_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "tl_driver/joint_trajectory",
            rclcpp::QoS(10).reliable());
        
        // 这个是遥操或者外部控制的话题
        target_pos_sub_ =
            node_->create_subscription<sensor_msgs::msg::JointState>(
                "/teleop/target_position",
                rclcpp::QoS(1).reliable(),
                [this](const sensor_msgs::msg::JointState::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    
                    if (external_target_positions_.size() != joint_names_.size()) {
                        external_target_positions_.resize(joint_names_.size(), 0.0);
                    }
                    
                    bool all_found = true;
                    for (size_t i = 0; i < joint_names_.size(); ++i) {
                        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
                        if (it != msg->name.end()) {
                            size_t idx = std::distance(msg->name.begin(), it);
                            if (idx < msg->position.size()) {
                                external_target_positions_[i] = msg->position[idx];
                            } else {
                                RCLCPP_WARN(logger_, "JointState message missing position for joint %s", 
                                        joint_names_[i].c_str());
                                all_found = false;
                            }
                        } else {
                            std::string alt_name = "joint" + std::to_string(i + 1);
                            it = std::find(msg->name.begin(), msg->name.end(), alt_name);
                            if (it != msg->name.end()) {
                                size_t idx = std::distance(msg->name.begin(), it);
                                if (idx < msg->position.size()) {
                                    external_target_positions_[i] = msg->position[idx];
                                } else {
                                    RCLCPP_WARN(logger_, "JointState message missing position for joint %s (alternate name)", 
                                            alt_name.c_str());
                                    all_found = false;
                                }
                            } else {
                                RCLCPP_WARN(logger_, "JointState message does not contain joint %s", 
                                        joint_names_[i].c_str());
                                all_found = false;
                            }
                        }
                    }
                    
                    if (all_found) {
                        has_external_target_ = true;
                        external_ever_used_ = true;

                        if (node_) {
                            last_external_command_time_ = node_->now();
                            
                            // AUTO 模式下标记当前使用外部指令
                            if (trajectory_mode_ == TrajectoryMode::AUTO) {
                                using_external_in_auto_ = true;
                            }
                        }
                        
                        RCLCPP_INFO(logger_, "Received external joint target via JointState");
                    } else {
                        RCLCPP_WARN(logger_, "Incomplete joint target received, ignoring");
                        has_external_target_ = false;
                    }
                });

        auto joint_state_callback = [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
                if (it != msg->name.end()) {
                    size_t idx = std::distance(msg->name.begin(), it);
                    if (idx < msg->position.size()) {
                        joint_positions_[i] = msg->position[idx];
                    }
                    if (idx < msg->velocity.size()) {
                        joint_velocities_[i] = msg->velocity[idx];
                    }
                    if (idx < msg->effort.size()) {
                        joint_efforts_[i] = msg->effort[idx];
                    }
                } else {
                    std::string alt_name = "joint" + std::to_string(i + 1);
                    it = std::find(msg->name.begin(), msg->name.end(), alt_name);
                    if (it != msg->name.end()) {
                        size_t idx = std::distance(msg->name.begin(), it);
                        if (idx < msg->position.size()) {
                            joint_positions_[i] = msg->position[idx];
                        }
                        if (idx < msg->velocity.size()) {
                            joint_velocities_[i] = msg->velocity[idx];
                        }
                        if (idx < msg->effort.size()) {
                            joint_efforts_[i] = msg->effort[idx];
                        }
                    }
                }
            }
            
            new_joint_state_ = true;
            last_joint_state_time_ = node_->now();
        };

        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "tl_driver/current_joint_states",
            rclcpp::QoS(10).best_effort(),
            joint_state_callback);
        
        node_thread_ = std::thread([this]() {
            RCLCPP_INFO(logger_, "Starting ROS2 node thread for hardware interface");
            rclcpp::spin(node_);
            RCLCPP_INFO(logger_, "ROS2 node thread stopped");
        });
        
        if (info_.hardware_parameters.find("control_mode") != info_.hardware_parameters.end()) {
            std::string mode = info_.hardware_parameters.at("control_mode");
            if (mode == "position") {
                control_mode_ = ControlMode::POSITION;
                RCLCPP_INFO(logger_, "采用位置控制方式: position (目前仅支持位置控制)");
            } else if (mode == "velocity") {
                control_mode_ = ControlMode::VELOCITY;
            } else if (mode == "effort") {
                control_mode_ = ControlMode::EFFORT;
            }
        } else {
            control_mode_ = ControlMode::POSITION;
        }

        if (info_.hardware_parameters.find("trajectory_mode") != info_.hardware_parameters.end()) {
            const auto & mode = info_.hardware_parameters.at("trajectory_mode");

            if (mode == "external") {
                trajectory_mode_ = TrajectoryMode::EXTERNAL;
                RCLCPP_INFO(logger_, "采用外部点控制方式: external");
            } else if (mode == "controller") {
                trajectory_mode_ = TrajectoryMode::CONTROLLER;
                RCLCPP_INFO(logger_, "采用ros2 control 控制方式: controller");
            } else if (mode == "auto") {
                trajectory_mode_ = TrajectoryMode::AUTO;
                RCLCPP_INFO(logger_, "采用自动切换控制方式: auto (timeout: %ld ms)", 
                        auto_mode_timeout_.count());
            }
            RCLCPP_INFO(
                logger_,
                "Trajectory mode set to: %s",
                mode.c_str());
        } else {
            trajectory_mode_ = TrajectoryMode::CONTROLLER;
            RCLCPP_INFO(logger_, "Trajectory mode default: controller");
        }
             
        hardware_connected_ = true;
        // last_joint_state_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        last_joint_state_time_ = node_->now();
        
        // 初始化外部目标位置向量
        external_target_positions_.resize(joint_names_.size(), 0.0);
        has_external_target_ = false;
        using_external_in_auto_ = false;
        
        // 初始化保持位置向量
        hold_position_.resize(joint_names_.size(), 0.0);
        stale_controller_command_.resize(joint_names_.size(), 0.0);
        last_sent_commands_.resize(joint_names_.size(), 0.0);
        last_controller_commands_.resize(joint_names_.size(), 0.0);
        
        RCLCPP_INFO(logger_, "TL hardware interface configured successfully");
        RCLCPP_INFO(logger_, "关节数量: %zu", joint_names_.size());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to configure hardware interface: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TlHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    
    RCLCPP_INFO(logger_, "Activating TL hardware interface");
    
    if (!hardware_connected_ || !node_) {
        RCLCPP_ERROR(logger_, "Hardware not connected, cannot activate");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    power_on_robot();
    
    std::this_thread::sleep_for(2s);

    bool got_initial_state = false;
    auto start_time = std::chrono::steady_clock::now();
    
    while (!got_initial_state && 
           std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5) {
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (new_joint_state_) {
                // 使用当前的关节位置初始化所有命令
                for (size_t i = 0; i < joint_names_.size(); ++i) {
                    joint_position_commands_[i] = joint_positions_[i];
                }
                
                // 初始化各种命令记录为当前关节位置
                last_sent_commands_ = joint_positions_;
                last_controller_commands_ = joint_positions_;
                controller_hold_current_ = false;
                
                got_initial_state = true;
                RCLCPP_INFO(logger_, "Got initial joint positions and initialized commands");
            }
        }
        std::this_thread::sleep_for(100ms);
    }
    
    if (!got_initial_state) {
        RCLCPP_WARN(logger_, "Timeout waiting for initial joint states");
        // 使用默认值初始化
        last_sent_commands_ = joint_position_commands_;
        last_controller_commands_ = joint_position_commands_;
    }
    
    hardware_powered_ = true;
    RCLCPP_INFO(logger_, "TL hardware interface activated successfully");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TlHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    
    RCLCPP_INFO(logger_, "Deactivating TL hardware interface");
    
    if (hardware_connected_ && node_) {
        power_off_robot();
        std::this_thread::sleep_for(1s);
    }
    
    hardware_powered_ = false;
    RCLCPP_INFO(logger_, "TL hardware interface deactivated");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TlHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    
    RCLCPP_INFO(logger_, "Cleaning up TL hardware interface");
    
    if (node_) {
        node_.reset();
    }
    
    if (node_thread_.joinable()) {
        node_thread_.join();
    }
    
    hardware_connected_ = false;
    
    // 重置各种标志
    using_external_in_auto_ = false;
    has_external_target_ = false;
    last_external_command_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    hold_active_ = false;
    
    RCLCPP_INFO(logger_, "TL hardware interface cleaned up");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TlHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    
    if (node_) {
        auto now = node_->now();
        auto time_since_update = now - last_joint_state_time_;
        
        if (time_since_update.seconds() > 1.0) {
            static auto last_warning = std::chrono::steady_clock::now();
            auto now_time = std::chrono::steady_clock::now();
            
            if (std::chrono::duration_cast<std::chrono::seconds>(now_time - last_warning).count() >= 5) {
                RCLCPP_WARN(logger_, "No joint state update for %.1f seconds", 
                           time_since_update.seconds());
                last_warning = now_time;
            }
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type TlHardwareInterface::write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)period;

    if (!hardware_powered_ || !node_ ||
        control_mode_ != ControlMode::POSITION) {
        return hardware_interface::return_type::OK;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    if (first_write_) {
        last_sent_commands_ = joint_position_commands_;
        first_write_ = false;
        return hardware_interface::return_type::OK;
    }

    std::vector<double>* source_commands = nullptr;
    bool using_external = false;
    bool mode_changed = false;

    if (trajectory_mode_ == TrajectoryMode::EXTERNAL) {
        if (!has_external_target_) {
            return hardware_interface::return_type::OK;
        }
        source_commands = &external_target_positions_;
        using_external = true;
    }
    else if (trajectory_mode_ == TrajectoryMode::CONTROLLER) {
        source_commands = &joint_position_commands_;
        using_external = false;
    }
    else if (trajectory_mode_ == TrajectoryMode::AUTO) {
        auto now = time;

        bool external_available = has_external_target_;

        if (external_available) {
            auto time_since_last_external = now - last_external_command_time_;
            auto timeout_duration = rclcpp::Duration::from_nanoseconds(
                auto_mode_timeout_.count() * 1000000LL);

            if (time_since_last_external > timeout_duration) {
                external_available = false;

                if (using_external_in_auto_) {
                    using_external_in_auto_ = false;
                    mode_changed = true;
                    controller_hold_current_ = true;
                    // 记录保持位置与 controller 的旧值, 用于后续忽略 controller 的陈旧 pos1
                    hold_position_ = external_target_positions_;
                    hold_active_ = true;
                    stale_controller_command_ = joint_position_commands_;
                    RCLCPP_INFO(logger_,
                        "AUTO mode: External timeout, switching to controller (holding last external)");
                }
            }
        }

        bool first_external_use = external_available && !using_external_in_auto_;
        using_external_in_auto_ = external_available;

        if (first_external_use) {
            hold_active_ = false;  // 重新使用外部指令, 清除 hold 状态
        }

        source_commands = external_available ? &external_target_positions_ : &joint_position_commands_;
        using_external = external_available;

        mode_changed = mode_changed || first_external_use;
    }
    else {
        return hardware_interface::return_type::OK;
    }

    if (!source_commands) {
        return hardware_interface::return_type::OK;
    }

    // 若处于 hold 状态, 检测 controller 是否已发送新指令（用户通过 ros2 controller 发轨迹）
    if (hold_active_ && trajectory_mode_ == TrajectoryMode::AUTO && !using_external) {
        if (stale_controller_command_.size() == joint_position_commands_.size() &&
            commandChanged(joint_position_commands_, stale_controller_command_)) {
            hold_active_ = false;
        }
    }

    // 是否需要 publish、实际发送的命令
    const std::vector<double>* effective_commands = source_commands;
    if (hold_active_ && hold_position_.size() == joint_names_.size()) {
        effective_commands = &hold_position_;
    }
    bool should_publish = mode_changed || commandChanged(*effective_commands, last_sent_commands_);

    if (!should_publish) {
        return hardware_interface::return_type::OK;
    }

    std::vector<double> commands_to_send = *effective_commands;

    // 首次切换时同步 joint_position_commands_, 使 controller 的起点为 pos2
    if (mode_changed && controller_hold_current_ &&
        external_target_positions_.size() == joint_names_.size()) {
        joint_position_commands_ = external_target_positions_;
        controller_hold_current_ = false;
    }

    // 发布轨迹
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = time;
    traj.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = commands_to_send;
    point.time_from_start = rclcpp::Duration::from_seconds(0.2);
    traj.points.push_back(point);

    trajectory_pub_->publish(traj);

    if (mode_changed) {
        RCLCPP_INFO(logger_, "AUTO mode: Published transition trajectory");
    }

    last_sent_commands_ = commands_to_send;

    if (trajectory_mode_ == TrajectoryMode::EXTERNAL) {
        has_external_target_ = false;
    }

    last_controller_commands_ = joint_position_commands_;

    return hardware_interface::return_type::OK;
}

bool TlHardwareInterface::commandChanged(
    const std::vector<double>& a,
    const std::vector<double>& b,
    double eps)
{
    if (a.size() != b.size()) return true;

    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > eps) {
            return true;
        }
    }
    return false;
}

void TlHardwareInterface::power_on_robot() {
    if (!node_ || !power_pub_) {
        return;
    }
    
    auto msg = std_msgs::msg::String();
    msg.data = "arm_power_on"; // Power on command
    
    power_pub_->publish(msg);
    RCLCPP_INFO(logger_, "Published power ON command (data=arm_power_on)");
}

void TlHardwareInterface::power_off_robot() {
    if (!node_ || !power_pub_) {
        return;
    }
    
    auto msg = std_msgs::msg::String();
    msg.data = "arm_power_off"; // Power off command
    
    power_pub_->publish(msg);
    RCLCPP_INFO(logger_, "Published power OFF command (data=arm_power_off)");
}

void TlHardwareInterface::send_position_command(const rclcpp::Time& time) {
    if (!node_ || !trajectory_pub_) {
        return;
    }
    
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    trajectory_msg.header.stamp = time;  // 使用传入的时间
    trajectory_msg.header.frame_id = "";
    trajectory_msg.joint_names = joint_names_;
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(joint_names_.size());
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        point.positions[i] = joint_position_commands_[i];
    }
    
    point.time_from_start = rclcpp::Duration(0, 0);
    
    trajectory_msg.points.push_back(point);
    
    trajectory_pub_->publish(trajectory_msg);
    
    static size_t publish_count = 0;
    if (++publish_count % 50 == 0) {
        RCLCPP_DEBUG(logger_, "Published position command to %zu joints", joint_names_.size());
    }
}

std::vector<hardware_interface::StateInterface> TlHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint_names_[i], 
                hardware_interface::HW_IF_POSITION, 
                &joint_positions_[i]));
        
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint_names_[i], 
                hardware_interface::HW_IF_VELOCITY, 
                &joint_velocities_[i]));
        
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint_names_[i], 
                hardware_interface::HW_IF_EFFORT, 
                &joint_efforts_[i]));
    }
    
    RCLCPP_INFO(logger_, "Exported %zu state interfaces", state_interfaces.size());
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TlHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    if (control_mode_ == ControlMode::POSITION) {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    joint_names_[i], 
                    hardware_interface::HW_IF_POSITION, 
                    &joint_position_commands_[i]));
        }
    } else if (control_mode_ == ControlMode::VELOCITY) {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    joint_names_[i], 
                    hardware_interface::HW_IF_VELOCITY, 
                &joint_velocity_commands_[i]));
        }
    } else if (control_mode_ == ControlMode::EFFORT) {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    joint_names_[i], 
                    hardware_interface::HW_IF_EFFORT, 
                    &joint_effort_commands_[i]));
        }
    }
    
    RCLCPP_INFO(logger_, "Exported %zu command interfaces for %s control", 
                command_interfaces.size(),
                control_mode_ == ControlMode::POSITION ? "position" : 
                control_mode_ == ControlMode::VELOCITY ? "velocity" : "effort");
    
    return command_interfaces;
}

}  // namespace tl_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(tl_hardware::TlHardwareInterface, hardware_interface::SystemInterface)