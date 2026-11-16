#ifndef ZROBOT_GZ_SIM__GAZEBO_MOTOR_BRIDGE_NODE_HPP_
#define ZROBOT_GZ_SIM__GAZEBO_MOTOR_BRIDGE_NODE_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "rs_interface/msg/joint_cmd.hpp"
#include "rs_interface/msg/joint_state.hpp"
#include "rs_interface/msg/motor_telemetry.hpp"

// Gazebo 仿真桥：与实机 bridge 使用同一套 /joint_cmd -> /joint_state 契约。
class GazeboMotorBridgeNode : public rclcpp::Node
{
public:
    GazeboMotorBridgeNode();

private:
    static constexpr size_t kNumMotors = 23;

    static std::vector<std::string> default_joint_names();

    double compute_tau(
        double target_q,
        double current_q,
        double target_dq,
        double current_dq,
        double kp,
        double kd,
        double torque_ff) const;

    void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
    void on_joint_cmd(const rs_interface::msg::JointCmd::SharedPtr msg);
    void control_loop();
    void publish_contract_state();
    void publish_telemetry();
    const char* safety_state_string() const;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
    rclcpp::Publisher<rs_interface::msg::JointState>::SharedPtr state_pub_;
    rclcpp::Publisher<rs_interface::msg::MotorTelemetry>::SharedPtr telemetry_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<rs_interface::msg::JointCmd>::SharedPtr joint_cmd_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, size_t> joint_name_to_index_;
    size_t active_joint_count_;

    std::mutex state_mutex_;

    // Gazebo 原始状态（未减零位偏移）
    std::array<float, kNumMotors> last_positions_;
    std::array<float, kNumMotors> last_velocities_;
    std::array<float, kNumMotors> last_efforts_;
    std::array<float, kNumMotors> zero_offsets_;
    bool has_joint_state_;

    // 命令缓存（位置为相对零位）
    bool enabled_;
    bool damping_mode_;
    bool has_mit_command_;
    std::array<float, kNumMotors> target_positions_;
    std::array<float, kNumMotors> target_velocities_;
    std::array<float, kNumMotors> target_torques_;
    std::array<float, kNumMotors> target_kps_;
    std::array<float, kNumMotors> target_kds_;
    std::chrono::steady_clock::time_point last_command_time_;

    std::vector<double> kp_;
    std::vector<double> kd_;
    float feedback_temperature_;
    double command_timeout_ms_;
    double damping_kd_;
    double telemetry_divider_;
    uint64_t state_count_;
    bool command_stale_;
};

#endif
