#include "zrobot_deploy/FSM.h"

#include <algorithm>
#include <thread>
#include <vector>

namespace
{
template <typename T>
T getOrDeclareParameter(const std::shared_ptr<rclcpp::Node>& node,
                        const std::string& name,
                        const T& default_value)
{
    if (node->has_parameter(name))
    {
        T value{};
        if (node->get_parameter(name, value))
        {
            return value;
        }
        return default_value;
    }
    return node->declare_parameter<T>(name, default_value);
}

std::vector<std::string> default_joint_names()
{
    return {
        "L_hip_roll_joint", "L_hip_yaw_joint", "L_hip_pitch_joint", "L_knee_joint",
        "L_foot_pitch_joint", "L_foot_roll_joint",
        "R_hip_roll_joint", "R_hip_yaw_joint", "R_hip_pitch_joint", "R_knee_joint",
        "R_foot_pitch_joint", "R_foot_roll_joint",
        "joint_12", "joint_13", "joint_14", "joint_15", "joint_16", "joint_17",
        "joint_18", "joint_19", "joint_20", "joint_21", "joint_22"};
}
} // namespace

FSM::FSM(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
    , active_joint_count_(12)
    , state_timeout_ms_(100.0)
    , current_state_(FSMState::IDLE)
    , control_frequency_(100.0)  // 100Hz
{
    current_motor_positions_.fill(0.0f);
    feedback_positions_.fill(0.0f);
    feedback_velocities_.fill(0.0f);
    feedback_torques_.fill(0.0f);
    feedback_temperatures_.fill(0.0f);
    feedback_valid_.fill(false);
    state_received_ = false;

    const auto names = getOrDeclareParameter<std::vector<std::string>>(
        node_, "joint_names", default_joint_names());
    if (names.size() == joint_names_.size())
    {
        std::copy(names.begin(), names.end(), joint_names_.begin());
    }
    else
    {
        const auto defaults = default_joint_names();
        std::copy(defaults.begin(), defaults.end(), joint_names_.begin());
        RCLCPP_WARN(node_->get_logger(),
                    "joint_names size=%zu, expected=%zu; using defaults",
                    names.size(), joint_names_.size());
    }

    active_joint_count_ = std::clamp(
        getOrDeclareParameter<int>(node_, "active_joint_count", 12), 0, 23);
    state_timeout_ms_ = std::max(
        1.0, static_cast<double>(getOrDeclareParameter<int>(node_, "state_timeout_ms", 100)));

    const auto joint_cmd_topic = getOrDeclareParameter<std::string>(
        node_, "joint_cmd_topic", "/joint_cmd");
    joint_state_topic_ = getOrDeclareParameter<std::string>(
        node_, "joint_state_topic", "/joint_state");

    joint_cmd_pub_ = node_->create_publisher<rs_interface::msg::JointCmd>(
        joint_cmd_topic, rclcpp::QoS(1).reliable());

    RCLCPP_INFO(node_->get_logger(), "FSM initialized: cmd=%s state=%s",
                joint_cmd_topic.c_str(), joint_state_topic_.c_str());
}

FSM::~FSM()
{
    RCLCPP_INFO(node_->get_logger(), "FSM destroyed");
}

void FSM::initialize()
{
    if (!joint_state_sub_)
    {
        // 用 weak_ptr 绑定回调：executor 线程中的回调不会在 FSM 销毁后访问悬空 this。
        std::weak_ptr<FSM> weak_self = weak_from_this();
        joint_state_sub_ = node_->create_subscription<rs_interface::msg::JointState>(
            joint_state_topic_, rclcpp::QoS(1).best_effort(),
            [weak_self](rs_interface::msg::JointState::SharedPtr msg) {
                if (auto self = weak_self.lock())
                {
                    self->jointStateCallback(msg);
                }
            });
    }

    RCLCPP_INFO(node_->get_logger(), "Requesting motor enable and waiting for /joint_state...");
    publishCommand("enable");
    if (!waitForFirstState(5.0))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "No fresh /joint_state with %d valid joints within 5 s; "
                    "continuing, bridge safety logic still applies",
                    active_joint_count_);
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "/joint_state ready, active joints are valid");
    }
}

void FSM::exit()
{
    RCLCPP_INFO(node_->get_logger(), "FSM exiting");
}

void FSM::sendDampCommand()
{
    publishCommand("damp");
}

void FSM::publishCommand(const std::string& command)
{
    rs_interface::msg::JointCmd msg;
    msg.command = command;
    msg.stamp = node_->now();
    msg.name.assign(joint_names_.begin(), joint_names_.end());
    joint_cmd_pub_->publish(msg);
}

bool FSM::sendMotorPositions(const std::array<float, 23>& positions)
{
    rs_interface::msg::JointCmd msg;
    msg.command = "mit";
    msg.stamp = node_->now();
    msg.name.assign(joint_names_.begin(), joint_names_.end());
    msg.position.assign(positions.begin(), positions.end());
    // 第一阶段：velocity/torque/kp/kd 留空，由 bridge 使用 motor_config.yaml 增益。
    joint_cmd_pub_->publish(msg);
    return true;
}

bool FSM::getMotorFeedback(std::array<float, 23>& positions,
                           std::array<float, 23>& velocities,
                           std::array<float, 23>& torques,
                           std::array<float, 23>& temperatures)
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    positions = feedback_positions_;
    velocities = feedback_velocities_;
    torques = feedback_torques_;
    temperatures = feedback_temperatures_;
    return state_received_;
}

bool FSM::getCurrentPositions(std::array<float, 23>& positions)
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    positions = feedback_positions_;
    return state_received_;
}

bool FSM::stateFresh(double timeout_ms) const
{
    if (timeout_ms <= 0.0)
    {
        timeout_ms = state_timeout_ms_;
    }
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!state_received_)
    {
        return false;
    }
    const auto age = std::chrono::steady_clock::now() - last_state_time_;
    return age <= std::chrono::duration<double, std::milli>(timeout_ms);
}

bool FSM::activeJointsValid() const
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!state_received_)
    {
        return false;
    }
    for (int i = 0; i < active_joint_count_; ++i)
    {
        if (!feedback_valid_[static_cast<size_t>(i)])
        {
            return false;
        }
    }
    return true;
}

bool FSM::waitForFirstState(double timeout_sec)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_sec);
    auto last_enable = std::chrono::steady_clock::now() - std::chrono::seconds(1);

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
    {
        if (state_received_ && activeJointsValid())
        {
            return true;
        }
        if (std::chrono::steady_clock::now() - last_enable >= std::chrono::milliseconds(500))
        {
            publishCommand("enable");
            last_enable = std::chrono::steady_clock::now();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return state_received_ && activeJointsValid();
}

void FSM::jointStateCallback(const rs_interface::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);

    state_received_ = true;
    last_state_time_ = std::chrono::steady_clock::now();

    feedback_positions_.fill(0.0f);
    feedback_velocities_.fill(0.0f);
    feedback_torques_.fill(0.0f);
    feedback_temperatures_.fill(0.0f);
    feedback_valid_.fill(false);

    const size_t count = std::min(static_cast<size_t>(23), msg->position.size());
    for (size_t i = 0; i < count; ++i)
    {
        feedback_positions_[i] = static_cast<float>(msg->position[i]);
        if (i < msg->velocity.size())
        {
            feedback_velocities_[i] = static_cast<float>(msg->velocity[i]);
        }
        if (i < msg->torque.size())
        {
            feedback_torques_[i] = static_cast<float>(msg->torque[i]);
        }
        if (i < msg->temperature.size())
        {
            feedback_temperatures_[i] = static_cast<float>(msg->temperature[i]);
        }
        if (i < msg->valid.size())
        {
            feedback_valid_[i] = msg->valid[i];
        }
    }
}
