#ifndef ZROBOT_GZ_SIM__GAZEBO_MOTOR_BRIDGE_NODE_HPP_
#define ZROBOT_GZ_SIM__GAZEBO_MOTOR_BRIDGE_NODE_HPP_

#include <array>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "rs_interface/srv/get_positions.hpp"
#include "rs_interface/srv/rob_stride_msgs.hpp"
#include "rs_interface/srv/set_zeros.hpp"

class GazeboMotorBridgeNode : public rclcpp::Node
{
public:
    GazeboMotorBridgeNode();

private:
    static constexpr size_t kNumMotors = 23;

    static std::vector<std::string> default_joint_names();

    void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);

    void on_rob_stride_control(
        const std::shared_ptr<rs_interface::srv::RobStrideMsgs::Request> request,
        std::shared_ptr<rs_interface::srv::RobStrideMsgs::Response> response);

    void on_get_positions(
        const std::shared_ptr<rs_interface::srv::GetPositions::Request> request,
        std::shared_ptr<rs_interface::srv::GetPositions::Response> response);

    void on_set_zeros(
        const std::shared_ptr<rs_interface::srv::SetZeros::Request> request,
        std::shared_ptr<rs_interface::srv::SetZeros::Response> response);

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp::Service<rs_interface::srv::RobStrideMsgs>::SharedPtr rob_stride_service_;
    rclcpp::Service<rs_interface::srv::GetPositions>::SharedPtr get_positions_service_;
    rclcpp::Service<rs_interface::srv::SetZeros>::SharedPtr set_zeros_service_;

    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, size_t> joint_name_to_index_;
    size_t active_joint_count_;

    std::mutex state_mutex_;
    std::array<float, kNumMotors> zero_offsets_;
    std::array<float, kNumMotors> last_positions_;
    std::array<float, kNumMotors> last_velocities_;
    std::array<float, kNumMotors> last_efforts_;

    float feedback_temperature_;
    bool has_joint_state_;
};

#endif