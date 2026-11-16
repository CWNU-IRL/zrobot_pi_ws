#include "zrobot_gz_sim/gazebo_motor_bridge_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>

namespace
{
constexpr double kDefaultKp = 40.0;
constexpr double kDefaultKd = 1.0;
} // namespace

GazeboMotorBridgeNode::GazeboMotorBridgeNode()
    : Node("gazebo_motor_bridge_node"),
      active_joint_count_(12),
      has_joint_state_(false),
      enabled_(false),
      damping_mode_(false),
      has_mit_command_(false),
      feedback_temperature_(35.0f),
      command_timeout_ms_(300.0),
      damping_kd_(1.0),
      telemetry_divider_(10.0),
      state_count_(0),
      command_stale_(false)
{
    joint_names_ = this->declare_parameter<std::vector<std::string>>(
        "joint_names", default_joint_names());
    if (joint_names_.size() != kNumMotors)
    {
        RCLCPP_WARN(get_logger(), "joint_names size=%zu, expected=%zu; using defaults",
                    joint_names_.size(), kNumMotors);
        joint_names_ = default_joint_names();
    }

    active_joint_count_ = static_cast<size_t>(std::clamp<int64_t>(
        this->declare_parameter<int>("active_joint_count", 12), 0,
        static_cast<int64_t>(kNumMotors)));

    kp_ = this->declare_parameter<std::vector<double>>(
        "kp", std::vector<double>(kNumMotors, kDefaultKp));
    kd_ = this->declare_parameter<std::vector<double>>(
        "kd", std::vector<double>(kNumMotors, kDefaultKd));
    kp_.resize(kNumMotors, kDefaultKp);
    kd_.resize(kNumMotors, kDefaultKd);

    feedback_temperature_ =
        static_cast<float>(this->declare_parameter<double>("feedback_temperature", 35.0));
    command_timeout_ms_ = static_cast<double>(this->declare_parameter<int>("command_timeout_ms", 300));
    damping_kd_ = this->declare_parameter<double>("damping_kd", 1.0);
    const double state_publish_hz = this->declare_parameter<double>("state_publish_hz", 100.0);
    const double telemetry_hz = this->declare_parameter<double>("telemetry_hz", 10.0);
    telemetry_divider_ = std::max(1.0, std::round(state_publish_hz / std::max(1.0, telemetry_hz)));

    zero_offsets_.fill(0.0f);
    last_positions_.fill(0.0f);
    last_velocities_.fill(0.0f);
    last_efforts_.fill(0.0f);
    target_positions_.fill(0.0f);
    target_velocities_.fill(0.0f);
    target_torques_.fill(0.0f);
    target_kps_.fill(static_cast<float>(kDefaultKp));
    target_kds_.fill(static_cast<float>(kDefaultKd));

    for (size_t i = 0; i < kNumMotors; ++i)
    {
        joint_name_to_index_[joint_names_[i]] = i;
    }

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/joint_group_effort_controller/commands", 10);
    state_pub_ = create_publisher<rs_interface::msg::JointState>(
        "/joint_state", rclcpp::QoS(1).best_effort());
    telemetry_pub_ = create_publisher<rs_interface::msg::MotorTelemetry>(
        "/motor_telemetry", rclcpp::QoS(1).best_effort());

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 50,
        std::bind(&GazeboMotorBridgeNode::on_joint_state, this, std::placeholders::_1));
    joint_cmd_sub_ = create_subscription<rs_interface::msg::JointCmd>(
        "/joint_cmd", rclcpp::QoS(1).reliable(),
        std::bind(&GazeboMotorBridgeNode::on_joint_cmd, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&GazeboMotorBridgeNode::control_loop, this));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, state_publish_hz)));
    state_timer_ = create_wall_timer(
        period, std::bind(&GazeboMotorBridgeNode::publish_contract_state, this));

    RCLCPP_INFO(get_logger(),
                "Gazebo motor bridge ready. active_joints=%zu, state_hz=%.1f",
                active_joint_count_, state_publish_hz);
}

std::vector<std::string> GazeboMotorBridgeNode::default_joint_names()
{
    return {
        "L_hip_roll_joint", "L_hip_yaw_joint", "L_hip_pitch_joint", "L_knee_joint",
        "L_foot_pitch_joint", "L_foot_roll_joint",
        "R_hip_roll_joint", "R_hip_yaw_joint", "R_hip_pitch_joint", "R_knee_joint",
        "R_foot_pitch_joint", "R_foot_roll_joint",
        "joint_12", "joint_13", "joint_14", "joint_15", "joint_16", "joint_17",
        "joint_18", "joint_19", "joint_20", "joint_21", "joint_22"};
}

double GazeboMotorBridgeNode::compute_tau(
    double target_q,
    double current_q,
    double target_dq,
    double current_dq,
    double kp,
    double kd,
    double torque_ff) const
{
    return (target_q - current_q) * kp + (target_dq - current_dq) * kd + torque_ff;
}

void GazeboMotorBridgeNode::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        const auto it = joint_name_to_index_.find(msg->name[i]);
        if (it == joint_name_to_index_.end())
        {
            continue;
        }
        const size_t idx = it->second;
        if (i < msg->position.size())
        {
            last_positions_[idx] = static_cast<float>(msg->position[i]);
        }
        if (i < msg->velocity.size())
        {
            last_velocities_[idx] = static_cast<float>(msg->velocity[i]);
        }
        if (i < msg->effort.size())
        {
            last_efforts_[idx] = static_cast<float>(msg->effort[i]);
        }
    }

    has_joint_state_ = true;
}

void GazeboMotorBridgeNode::on_joint_cmd(const rs_interface::msg::JointCmd::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    const std::string& command = msg->command;

    if (command == "mit")
    {
        if (!enabled_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "mit ignored: Gazebo bridge is not enabled");
            return;
        }
        if (msg->position.size() != kNumMotors ||
            (!msg->name.empty() && msg->name.size() != kNumMotors) ||
            (!msg->velocity.empty() && msg->velocity.size() != kNumMotors) ||
            (!msg->torque.empty() && msg->torque.size() != kNumMotors) ||
            (!msg->kp.empty() && msg->kp.size() != kNumMotors) ||
            (!msg->kd.empty() && msg->kd.size() != kNumMotors))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "mit rejected: array sizes must be empty or %zu", kNumMotors);
            return;
        }

        for (size_t i = 0; i < static_cast<size_t>(kNumMotors); ++i)
        {
            size_t idx = i;
            if (!msg->name.empty())
            {
                const auto it = joint_name_to_index_.find(msg->name[i]);
                if (it == joint_name_to_index_.end())
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "mit rejected: unknown joint '%s'", msg->name[i].c_str());
                    return;
                }
                idx = it->second;
            }
            if (idx >= active_joint_count_)
            {
                continue;
            }

            target_positions_[idx] = static_cast<float>(msg->position[i]);
            target_velocities_[idx] = msg->velocity.empty()
                                          ? 0.0f
                                          : static_cast<float>(msg->velocity[i]);
            target_torques_[idx] = msg->torque.empty()
                                       ? 0.0f
                                       : static_cast<float>(msg->torque[i]);
            target_kps_[idx] = msg->kp.empty()
                                   ? static_cast<float>(kp_[idx])
                                   : static_cast<float>(msg->kp[i]);
            target_kds_[idx] = msg->kd.empty()
                                   ? static_cast<float>(kd_[idx])
                                   : static_cast<float>(msg->kd[i]);
        }

        has_mit_command_ = true;
        damping_mode_ = false;
        last_command_time_ = std::chrono::steady_clock::now();
        return;
    }

    if (command == "enable")
    {
        if (!has_joint_state_)
        {
            RCLCPP_WARN(get_logger(), "enable ignored: no /joint_states from Gazebo yet");
            return;
        }
        for (size_t i = 0; i < active_joint_count_; ++i)
        {
            target_positions_[i] = last_positions_[i] - zero_offsets_[i];
        }
        enabled_ = true;
        damping_mode_ = true;
        has_mit_command_ = false;
        last_command_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "Enabling Gazebo bridge with soft hold");
        return;
    }

    if (command == "disable" || command == "stop")
    {
        enabled_ = false;
        damping_mode_ = false;
        has_mit_command_ = false;
        RCLCPP_INFO(get_logger(), "Gazebo bridge disabled");
        return;
    }

    if (command == "damp")
    {
        if (!enabled_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "damp ignored: Gazebo bridge is not enabled");
            return;
        }
        damping_mode_ = true;
        has_mit_command_ = false;
        last_command_time_ = std::chrono::steady_clock::now();
        return;
    }

    if (command == "set_zero")
    {
        if (!has_joint_state_)
        {
            RCLCPP_WARN(get_logger(), "set_zero ignored: no /joint_states from Gazebo yet");
            return;
        }
        for (size_t i = 0; i < active_joint_count_; ++i)
        {
            zero_offsets_[i] = last_positions_[i];
            target_positions_[i] = 0.0f;
        }
        RCLCPP_INFO(get_logger(), "Zero offsets captured from current Gazebo positions");
        return;
    }

    if (command == "status")
    {
        RCLCPP_INFO(get_logger(),
                    "status: enabled=%d damping=%d stale=%d has_mit=%d",
                    static_cast<int>(enabled_), static_cast<int>(damping_mode_),
                    static_cast<int>(command_stale_), static_cast<int>(has_mit_command_));
        return;
    }

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "unknown /joint_cmd command: '%s'", command.c_str());
}

void GazeboMotorBridgeNode::control_loop()
{
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(active_joint_count_, 0.0);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!has_joint_state_)
        {
            return;
        }

        const bool stale = has_mit_command_ &&
            (std::chrono::steady_clock::now() - last_command_time_) >
                std::chrono::duration<double, std::milli>(command_timeout_ms_);
        command_stale_ = stale;
        const bool use_damping = !enabled_ || damping_mode_ || stale || !has_mit_command_;

        for (size_t i = 0; i < active_joint_count_; ++i)
        {
            if (!enabled_)
            {
                cmd.data[i] = 0.0;
                continue;
            }

            double target_q = 0.0;
            double target_dq = 0.0;
            double torque_ff = 0.0;
            double kp = 0.0;
            double kd = 0.0;

            if (use_damping)
            {
                target_q = last_positions_[i];
                kd = damping_kd_;
            }
            else
            {
                target_q = target_positions_[i] + zero_offsets_[i];
                target_dq = target_velocities_[i];
                torque_ff = target_torques_[i];
                kp = target_kps_[i];
                kd = target_kds_[i];
            }

            cmd.data[i] = compute_tau(
                target_q, last_positions_[i], target_dq, last_velocities_[i], kp, kd, torque_ff);
        }
    }

    command_pub_->publish(cmd);
}

void GazeboMotorBridgeNode::publish_contract_state()
{
    rs_interface::msg::JointState msg;
    msg.stamp = now();
    msg.name.assign(joint_names_.begin(), joint_names_.end());
    msg.position.resize(kNumMotors, 0.0);
    msg.velocity.resize(kNumMotors, 0.0);
    msg.torque.resize(kNumMotors, 0.0);
    msg.temperature.resize(kNumMotors, 0.0);
    msg.valid.resize(kNumMotors, false);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t i = 0; i < kNumMotors; ++i)
        {
            msg.temperature[i] = feedback_temperature_;
            msg.valid[i] = has_joint_state_;
            if (i < active_joint_count_)
            {
                msg.position[i] = last_positions_[i] - zero_offsets_[i];
                msg.velocity[i] = last_velocities_[i];
                msg.torque[i] = last_efforts_[i];
            }
        }
    }

    state_pub_->publish(msg);

    if (++state_count_ % static_cast<uint64_t>(telemetry_divider_) == 0)
    {
        publish_telemetry();
    }
}

void GazeboMotorBridgeNode::publish_telemetry()
{
    rs_interface::msg::MotorTelemetry msg;
    msg.stamp = now();
    msg.name.assign(joint_names_.begin(), joint_names_.end());
    msg.error_code.assign(kNumMotors, 0);
    msg.feedback_age_ms.assign(kNumMotors, 0.0);
    msg.online.assign(kNumMotors, true);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        msg.command_stale = command_stale_;
        msg.safety_state = safety_state_string();
        for (size_t i = 0; i < kNumMotors; ++i)
        {
            msg.online[i] = has_joint_state_;
        }
    }

    telemetry_pub_->publish(msg);
}

const char* GazeboMotorBridgeNode::safety_state_string() const
{
    if (!enabled_)
    {
        return "disabled";
    }
    if (damping_mode_ || command_stale_ || !has_mit_command_)
    {
        return "damping";
    }
    return "active";
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboMotorBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
