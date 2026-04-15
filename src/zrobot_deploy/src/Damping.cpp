#include "zrobot_deploy/Damping.h"

#include <algorithm>
#include <cmath>
#include <vector>

Damping::Damping(std::shared_ptr<rclcpp::Node> node)
    : FSM(node)
    , is_initialized_(false)
    , feedback_seeded_(false)
    , kd_default_(0.08f)
    , velocity_deadband_(0.02f)
    , max_position_delta_(0.15f)
{
    current_state_ = FSMState::DAMPING;
    damping_gains_.fill(kd_default_);
    command_positions_.fill(0.0f);

    RCLCPP_INFO(node_->get_logger(), "Damping FSM created");
}

Damping::~Damping()
{
    RCLCPP_INFO(node_->get_logger(), "Damping FSM destroyed");
}

void Damping::loadParameters()
{
    auto get_or_declare_double = [this](const std::string & name, double default_value) {
        double value = default_value;
        if (node_->has_parameter(name)) {
            node_->get_parameter(name, value);
        } else {
            value = node_->declare_parameter<double>(name, default_value);
        }
        return static_cast<float>(value);
    };

    kd_default_ = get_or_declare_double("damping.kd_default", 0.08);
    velocity_deadband_ = get_or_declare_double("damping.velocity_deadband", 0.02);
    max_position_delta_ = get_or_declare_double("damping.max_position_delta", 0.15);

    std::vector<double> kd_per_joint;
    if (node_->has_parameter("damping.kd_per_joint")) {
        node_->get_parameter("damping.kd_per_joint", kd_per_joint);
    } else {
        kd_per_joint = node_->declare_parameter<std::vector<double>>("damping.kd_per_joint", std::vector<double>{});
    }

    damping_gains_.fill(kd_default_);
    if (kd_per_joint.empty()) {
        return;
    }

    if (kd_per_joint.size() != damping_gains_.size()) {
        RCLCPP_WARN(
            node_->get_logger(),
            "damping.kd_per_joint size is %zu, expected %zu. Fallback to kd_default for all channels.",
            kd_per_joint.size(), damping_gains_.size());
        return;
    }

    for (size_t i = 0; i < damping_gains_.size(); ++i) {
        damping_gains_[i] = static_cast<float>(kd_per_joint[i]);
    }
}

void Damping::initialize()
{
    FSM::initialize();
    loadParameters();

    std::array<float, 23> current_positions;
    if (!getCurrentPositions(current_positions)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to read current positions, fallback to zeros for damping init");
        current_positions.fill(0.0f);
    }

    command_positions_ = current_positions;
    current_motor_positions_ = current_positions;
    feedback_seeded_ = false;
    is_initialized_ = true;

    RCLCPP_INFO(
        node_->get_logger(),
        "Damping initialized: kd_default=%.3f, velocity_deadband=%.3f, max_position_delta=%.3f",
        kd_default_, velocity_deadband_, max_position_delta_);
}

void Damping::run()
{
    if (!is_initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "Damping not initialized! Call initialize() first.");
        return;
    }

    // 首次进入先发送一次当前姿态，确保反馈缓存被服务响应刷新。
    if (!feedback_seeded_) {
        feedback_seeded_ = sendMotorPositions(command_positions_);
        if (!feedback_seeded_) {
            RCLCPP_WARN(node_->get_logger(), "Unable to seed feedback in damping mode, retrying...");
        }
        return;
    }

    std::array<float, 23> feedback_positions;
    std::array<float, 23> feedback_velocities;
    std::array<float, 23> feedback_torques;
    std::array<float, 23> feedback_temperatures;
    getMotorFeedback(feedback_positions, feedback_velocities, feedback_torques, feedback_temperatures);

    for (size_t i = 0; i < command_positions_.size(); ++i) {
        float cmd = feedback_positions[i];
        const float velocity = feedback_velocities[i];

        if (std::fabs(velocity) >= velocity_deadband_) {
            cmd -= damping_gains_[i] * velocity;
        }

        const float lower = feedback_positions[i] - max_position_delta_;
        const float upper = feedback_positions[i] + max_position_delta_;
        command_positions_[i] = std::clamp(cmd, lower, upper);
    }

    current_motor_positions_ = command_positions_;

    if (!sendMotorPositions(command_positions_)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to send damping commands");
    }
}

void Damping::exit()
{
    is_initialized_ = false;
    feedback_seeded_ = false;
    RCLCPP_INFO(node_->get_logger(), "Exiting Damping FSM");
    FSM::exit();
}
