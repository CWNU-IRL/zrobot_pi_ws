#include "zrobot_deploy/Damping.h"

#include <algorithm>
#include <cmath>
#include <vector>

Damping::Damping(std::shared_ptr<rclcpp::Node> node)
    : FSM(node)
    , is_initialized_(false)
    , feedback_seeded_(false)
    , motor_kp_(0.0f)
    , motor_kd_(2.0f)
{
    current_state_ = FSMState::DAMPING;
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

    // 电机内部原生阻尼增益：kp=0 无位置刚度（纯阻尼），kd 为粘性阻尼系数
    motor_kp_ = get_or_declare_double("damping.motor_kp", 0.0);
    motor_kd_ = get_or_declare_double("damping.motor_kd", 2.0);
}

void Damping::initialize()
{
    FSM::initialize();
    loadParameters();

    std::array<float, 23> current_positions;
    if (!getCurrentPositions(current_positions)) {
        // 服务失败时回退到最近一次成功反馈（硬件约定，与指令空间一致），
        // 避免以全零位姿为基准阻尼导致机器人被拉向零位
        RCLCPP_WARN(node_->get_logger(),
                    "Failed to read current positions, falling back to cached motor feedback");
        std::array<float, 23> cached_velocities, cached_torques, cached_temperatures;
        getMotorFeedback(current_positions, cached_velocities, cached_torques, cached_temperatures);

        // 缓存反馈从未被成功刷新过（如桥接层刚启动）时不可信，
        // 以全 0 为阻尼基准会把电机拉向零位，此时拒绝初始化
        if (!feedback_valid_) {
            RCLCPP_ERROR(node_->get_logger(),
                         "No valid motor feedback cache. Damping will NOT start to avoid "
                         "pulling motors toward zero. Check /rob_stride_control service.");
            is_initialized_ = false;
            return;
        }
    }

    command_positions_ = current_positions;
    current_motor_positions_ = current_positions;
    feedback_seeded_ = false;
    is_initialized_ = true;

    // 电机原生阻尼：指令帧携带 kp=0/kd=motor_kd_，由电机内部速度环直接阻尼
    command_kps_.fill(motor_kp_);
    command_kds_.fill(motor_kd_);

    RCLCPP_INFO(
        node_->get_logger(),
        "Damping initialized: motor_kp=%.3f, motor_kd=%.3f (native motor damping)",
        motor_kp_, motor_kd_);
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

    // 原生阻尼：位置指令跟踪当前反馈（kp=0 时位置不产生力矩），
    // 阻尼力矩由电机内部 τ = -kd·dq 实时产生，无外环延迟
    std::array<float, 23> feedback_positions;
    std::array<float, 23> feedback_velocities;
    std::array<float, 23> feedback_torques;
    std::array<float, 23> feedback_temperatures;
    getMotorFeedback(feedback_positions, feedback_velocities, feedback_torques, feedback_temperatures);

    command_positions_ = feedback_positions;
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
