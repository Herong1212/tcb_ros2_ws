/**
 * @file tl_hardware_interface.hpp
 * @brief TCB 机械臂 ros2_control 硬件接口
 *
 * 支持三种轨迹模式：
 * - CONTROLLER: 仅使用 ros2 controller 的关节指令
 * - EXTERNAL: 仅使用 /teleop/target_position 话题的外部指令
 * - AUTO: 自动切换，有外部指令时优先使用，超时后切回 controller
 */
#ifndef TL_HARDWARE_INTERFACE_HPP
#define TL_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

namespace tl_hardware {

/**
 * TCB 机械臂硬件接口，实现 ros2_control SystemInterface
 * 负责与 tl_driver 通信，发布轨迹到 tl_driver/joint_trajectory
 */
class TlHardwareInterface : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(TlHardwareInterface)

    TlHardwareInterface();
    virtual ~TlHardwareInterface();

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    /// 控制模式：目前仅使用 POSITION 位置控制
    enum class ControlMode {
        POSITION,
        VELOCITY,
        EFFORT
    };

    /// 轨迹指令来源模式
    enum class TrajectoryMode {
        CONTROLLER,  ///< 仅使用 ros2 controller 的 joint_position_commands_
        EXTERNAL,    ///< 仅使用 /teleop/target_position 话题的外部关节目标
        AUTO         ///< 有外部指令时用 EXTERNAL，超时后切回 CONTROLLER
    };

    /// AUTO 模式：上次收到 /teleop/target_position 的时间
    rclcpp::Time last_external_command_time_;
    /// AUTO 模式：外部指令超时时间（ms），超时后切回 controller
    std::chrono::milliseconds auto_mode_timeout_{200};
    /// AUTO 模式：当前周期是否使用外部指令
    bool using_external_in_auto_{false};
    /// 保存最后一次控制器写入的指令
    std::vector<double> last_controller_commands_;
    /// 外部超时切换瞬间的标记，用于一次性同步 joint_position_commands_
    bool controller_hold_current_{false};
    /// 是否曾经使用过外部指令
    bool external_ever_used_{false};

    /// AUTO 模式外部超时后：是否处于“保持 pos2”状态
    /// controller 会持续写入旧的 pos1，需忽略直至用户发送新轨迹
    bool hold_active_{false};
    /// 保持的目标关节角（/teleop/target_position 的最后一组）
    std::vector<double> hold_position_;
    /// 切换时的 controller 旧值（pos1），用于判断 controller 是否已发新指令
    std::vector<double> stale_controller_command_;

    // --- ROS2 节点与通信 ---
    std::shared_ptr<rclcpp::Node> node_;
    std::thread node_thread_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr power_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_version_sub_;
    /// 订阅 /teleop/target_position，EXTERNAL/AUTO 模式下作为关节目标
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_pos_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr stop_trajectory_pub_;

    /// 来自 /teleop/target_position 的关节目标
    std::vector<double> external_target_positions_;
    /// 是否已收到有效的外部目标
    bool has_external_target_{false};

    hardware_interface::HardwareInfo info_;
    rclcpp::Logger logger_;

    ControlMode control_mode_;
    TrajectoryMode trajectory_mode_{TrajectoryMode::CONTROLLER};

    /// 机械臂型号，用于确定关节数（如 TCB605_05 为 6 轴，TCB705_05 为 7 轴）
    std::string arm_version_;
    std::atomic<bool> arm_version_received_;

    // --- 关节数据 ---
    std::vector<double> joint_positions_;       ///< 来自 tl_driver/current_joint_states
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;
    std::vector<double> joint_position_commands_;  ///< controller 写入，write() 中发布到 driver
    std::vector<double> joint_velocity_commands_;
    std::vector<double> joint_effort_commands_;
    std::vector<std::string> joint_names_;

    std::atomic<bool> hardware_connected_;
    std::atomic<bool> hardware_powered_;
    std::atomic<bool> new_joint_state_;

    std::mutex data_mutex_;

    rclcpp::Time last_joint_state_time_;
    std::vector<double> last_sent_commands_;    ///< 上次发布到 driver 的关节指令
    rclcpp::Time last_command_time_;
    bool first_write_{true};

    // --- 辅助方法 ---
    void power_on_robot();
    void power_off_robot();
    void send_position_command(const rclcpp::Time& time);
    void reconfigure_joints_based_on_arm_version();
    /// 比较两向量是否不同（按 eps 容差）
    bool commandChanged(
            const std::vector<double>& a,
            const std::vector<double>& b,
            double eps = 1e-3);
};

}  // namespace tl_hardware

#endif  // TL_HARDWARE_INTERFACE_HPP