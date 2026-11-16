#include "zrobot_deploy/Damping.h"

Damping::Damping(std::shared_ptr<rclcpp::Node> node)
    : FSM(node)
    , is_initialized_(false)
{
    current_state_ = FSMState::DAMPING;
    RCLCPP_INFO(node_->get_logger(), "Damping FSM created");
}

Damping::~Damping()
{
    RCLCPP_INFO(node_->get_logger(), "Damping FSM destroyed");
}

void Damping::initialize()
{
    FSM::initialize();
    is_initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "Damping initialized: publishing damp commands");
}

void Damping::run()
{
    if (!is_initialized_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Damping not initialized! Call initialize() first.");
        return;
    }

    if (!stateFresh())
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "Damping: /joint_state is stale, damp command still published");
    }
    sendDampCommand();
}

void Damping::exit()
{
    is_initialized_ = false;
    RCLCPP_INFO(node_->get_logger(), "Exiting Damping FSM");
    FSM::exit();
}
