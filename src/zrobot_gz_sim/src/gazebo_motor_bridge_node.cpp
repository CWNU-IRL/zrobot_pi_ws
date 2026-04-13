#include "zrobot_gz_sim/gazebo_motor_bridge_node.hpp"

#include <cstdio>
#include <functional>

GazeboMotorBridgeNode::GazeboMotorBridgeNode()
    : Node("gazebo_motor_bridge_node"),
      active_joint_count_(kNumMotors),
      feedback_temperature_(35.0f),
      has_joint_state_(false)
{
    // 读取参数
    joint_names_ = this->declare_parameter<std::vector<std::string>>("joint_names", default_joint_names());
    feedback_temperature_ = static_cast<float>(this->declare_parameter<double>("feedback_temperature", 35.0));

    if (joint_names_.empty())
    {
        RCLCPP_WARN(
            get_logger(),
            "joint_names is empty. Falling back to default names.");
        joint_names_ = default_joint_names();
    }

    active_joint_count_ = joint_names_.size();

    if (active_joint_count_ > kNumMotors)
    {
        RCLCPP_WARN(
            get_logger(),
            "joint_names size is %zu, larger than %zu. Extra joints will be ignored.",
            active_joint_count_, kNumMotors);
        joint_names_.resize(kNumMotors);
        active_joint_count_ = kNumMotors;
    }

    if (active_joint_count_ < kNumMotors)
    {
        RCLCPP_WARN(
            get_logger(),
            "joint_names size is %zu, less than %zu. Remaining service channels will be zero-filled.",
            active_joint_count_, kNumMotors);
    }

    // Build the joint name to index mapping
    for (size_t i = 0; i < active_joint_count_; ++i)
    {
        joint_name_to_index_[joint_names_[i]] = i;
    }

    zero_offsets_.fill(0.0f);
    last_positions_.fill(0.0f);
    last_velocities_.fill(0.0f);
    last_efforts_.fill(0.0f);

    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/joint_group_position_controller/commands", 10);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 50,
        std::bind(&GazeboMotorBridgeNode::on_joint_state, this, std::placeholders::_1));

    rob_stride_service_ = create_service<rs_interface::srv::RobStrideMsgs>(
        "/rob_stride_control",
        std::bind(
            &GazeboMotorBridgeNode::on_rob_stride_control,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    get_positions_service_ = create_service<rs_interface::srv::GetPositions>(
        "/get_positions",
        std::bind(
            &GazeboMotorBridgeNode::on_get_positions,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    set_zeros_service_ = create_service<rs_interface::srv::SetZeros>(
        "/set_zeros",
        std::bind(
            &GazeboMotorBridgeNode::on_set_zeros,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Gazebo motor bridge is ready. Active joints: %zu", active_joint_count_);
}

std::vector<std::string> GazeboMotorBridgeNode::default_joint_names()
{
    std::vector<std::string> names;
    names.reserve(kNumMotors);
    for (size_t i = 1; i <= kNumMotors; ++i)
    {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "joint_%02zu", i);
        names.emplace_back(buf);
    }
    return names;
}

void GazeboMotorBridgeNode::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        auto it = joint_name_to_index_.find(msg->name[i]);
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

void GazeboMotorBridgeNode::on_rob_stride_control(
    const std::shared_ptr<rs_interface::srv::RobStrideMsgs::Request> request,
    std::shared_ptr<rs_interface::srv::RobStrideMsgs::Response> response)
{
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(active_joint_count_);

    std::array<float, kNumMotors> positions;
    std::array<float, kNumMotors> velocities;
    std::array<float, kNumMotors> efforts;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t i = 0; i < active_joint_count_; ++i)
        {
            cmd.data[i] = static_cast<double>(request->positions[i] + zero_offsets_[i]);
        }
        positions = last_positions_;
        velocities = last_velocities_;
        efforts = last_efforts_;
    }

    command_pub_->publish(cmd);

    for (size_t i = 0; i < kNumMotors; ++i)
    {
        response->feedback_positions[i] = positions[i] - zero_offsets_[i];
        response->feedback_velocities[i] = velocities[i];
        response->feedback_torques[i] = efforts[i];
        response->feedback_temperatures[i] = feedback_temperature_;
    }

    if (!has_joint_state_)
    {
        response->success = false;
        response->message = "No joint_states received yet";
        return;
    }

    response->success = true;
    response->message = "Command forwarded to joint_group_position_controller";
}

void GazeboMotorBridgeNode::on_get_positions(
    const std::shared_ptr<rs_interface::srv::GetPositions::Request>,
    std::shared_ptr<rs_interface::srv::GetPositions::Response> response)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (size_t i = 0; i < kNumMotors; ++i)
    {
        response->feedback_positions[i] = last_positions_[i] - zero_offsets_[i];
    }

    response->success = has_joint_state_;
    response->message = has_joint_state_ ? "OK" : "No joint_states received yet";
}

void GazeboMotorBridgeNode::on_set_zeros(
    const std::shared_ptr<rs_interface::srv::SetZeros::Request>,
    std::shared_ptr<rs_interface::srv::SetZeros::Response> response)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!has_joint_state_)
    {
        response->success = false;
        response->message = "No joint_states received yet";
        return;
    }

    for (size_t i = 0; i < active_joint_count_; ++i)
    {
        zero_offsets_[i] = last_positions_[i];
    }
    response->success = true;
    response->message = "Zero offsets captured from current joint positions";
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboMotorBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
