#include "zrobot_deploy/PTLocomotion.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace {
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

        RCLCPP_WARN(node->get_logger(),
                    "Parameter '%s' exists but type mismatch, using default value",
                    name.c_str());
        return default_value;
    }

    return node->declare_parameter<T>(name, default_value);
}

} // namespace

PTLocomotion::PTLocomotion(std::shared_ptr<rclcpp::Node> node)
    : FSM(node),
      thread_running_(false),
      imu_received_(false),
      require_imu_before_locomotion_(true),
      startup_hold_seconds_(0.5),
      dt_(0.01),
      phase_period_(0.64),
      frame_stack_(15),
      model_obs_dim_(NUM_SINGLE_OBS * 15),
      action_scale_(0.25f),
      obs_clip_(100.0f),
      act_clip_(100.0f),
      obs_scale_lin_vel_(1.0f),
      obs_scale_ang_vel_(1.0f),
      obs_scale_dof_pos_(1.0f),
      obs_scale_dof_vel_(1.0f),
      counter_(0),
      model_ready_(false)
{
    current_angular_velocity_.setZero();
    current_euler_.setZero();
    current_command_.setZero();

    dof_indices_ = {5, 4, 3, 2, 1, 0, 11, 10, 9, 8, 7, 6};
}

PTLocomotion::~PTLocomotion()
{
    exit();
    RCLCPP_INFO(node_->get_logger(), "PTLocomotion FSM destroyed");
}

void PTLocomotion::initialize()
{
    FSM::initialize();

    RCLCPP_INFO(node_->get_logger(), "PTLocomotion initializing...");
    initializeParameters();

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        getOrDeclareParameter<std::string>(node_, "imu_topic", "/imu/data"),
        rclcpp::SensorDataQoS(),
        std::bind(&PTLocomotion::imuCallback, this, std::placeholders::_1));

    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        getOrDeclareParameter<std::string>(node_, "cmd_topic", "cmd_vel"),
        10,
        std::bind(&PTLocomotion::cmdCallback, this, std::placeholders::_1));

    model_path_ = getOrDeclareParameter<std::string>(
        node_, "pt_model_path", "/home/bill/Codes/policy.pt");
    loadPolicy(model_path_);

    if (!model_ready_)
    {
        RCLCPP_ERROR(node_->get_logger(), "PTLocomotion model is not ready, inference thread will not start");
        return;
    }

    counter_ = 0;
    init_time_ = std::chrono::steady_clock::now();
    thread_running_ = true;
    inference_thread_ = std::thread(&PTLocomotion::inferenceLoop, this);

    current_state_ = FSMState::PT_LOCOMOTION;
    RCLCPP_INFO(node_->get_logger(), "PTLocomotion initialized and inference started");
}

void PTLocomotion::run()
{
    using Clock = std::chrono::steady_clock;

    if (!thread_running_)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "PTLocomotion inference thread not running");
        return;
    }

    const auto elapsed = std::chrono::duration<double>(Clock::now() - init_time_).count();
    if (elapsed < startup_hold_seconds_)
    {
        if (!sendMotorPositions(current_motor_positions_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to hold motor positions during PT startup warmup");
        }
        return;
    }

    if (require_imu_before_locomotion_ && !imu_received_)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "IMU data not received yet, holding current pose in PTLocomotion");
        if (!sendMotorPositions(current_motor_positions_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to hold motor positions while waiting IMU");
        }
        return;
    }

    {
        std::lock_guard<std::mutex> lock(action_mutex_);
        act_temp_ = act_scaled_;
    }

    std::array<float, 23> positions = current_motor_positions_;
    for (int i = 0; i < NUM_ACTIONS; ++i)
    {
        const int motor_idx = dof_indices_[i];
        if (motor_idx < 0 || motor_idx >= NUM_MOTORS)
        {
            continue;
        }
        positions[static_cast<size_t>(motor_idx)] = default_pose_(i) + act_temp_(i);
    }

    if (!sendMotorPositions(positions))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send motor positions in PTLocomotion");
    }
    else
    {
        current_motor_positions_ = positions;
    }
}

void PTLocomotion::exit()
{
    if (thread_running_)
    {
        thread_running_ = false;
        if (inference_thread_.joinable())
        {
            inference_thread_.join();
        }
    }
    FSM::exit();
}

void PTLocomotion::loadPolicy(const std::string& model_path)
{
    model_ready_ = false;

    try
    {
        policy_module_ = torch::jit::load(model_path);
        policy_module_.eval();

        auto warmup = torch::zeros({1, model_obs_dim_}, torch::TensorOptions().dtype(torch::kFloat32));
        (void)policy_module_.forward({warmup});

        model_ready_ = true;
        RCLCPP_INFO(node_->get_logger(),
                    "Loaded PT model: %s, obs_dim=%d, frame_stack=%d",
                    model_path.c_str(), model_obs_dim_, frame_stack_);
    }
    catch (const c10::Error& e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load PT model: %s", e.what_without_backtrace());
        model_ready_ = false;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load PT model: %s", e.what());
        model_ready_ = false;
    }
}

void PTLocomotion::initializeParameters()
{
    dt_ = getOrDeclareParameter<double>(node_, "control_dt", 0.01);
    phase_period_ = getOrDeclareParameter<double>(node_, "phase_period", 0.64);
    frame_stack_ = getOrDeclareParameter<int>(node_, "frame_stack", 15);
    model_obs_dim_ = getOrDeclareParameter<int>(node_, "model_obs_dim", NUM_SINGLE_OBS * frame_stack_);

    action_scale_ = static_cast<float>(getOrDeclareParameter<double>(node_, "action_scale", 0.25));
    obs_clip_ = static_cast<float>(getOrDeclareParameter<double>(node_, "clip_observations", 100.0));
    act_clip_ = static_cast<float>(getOrDeclareParameter<double>(node_, "clip_actions", 100.0));

    obs_scale_lin_vel_ = static_cast<float>(getOrDeclareParameter<double>(node_, "obs_scale_lin_vel", 1.0));
    obs_scale_ang_vel_ = static_cast<float>(getOrDeclareParameter<double>(node_, "obs_scale_ang_vel", 1.0));
    obs_scale_dof_pos_ = static_cast<float>(getOrDeclareParameter<double>(node_, "obs_scale_dof_pos", 1.0));
    obs_scale_dof_vel_ = static_cast<float>(getOrDeclareParameter<double>(node_, "obs_scale_dof_vel", 1.0));

    startup_hold_seconds_ = getOrDeclareParameter<double>(node_, "startup_hold_seconds", 0.5);
    require_imu_before_locomotion_ = getOrDeclareParameter<bool>(node_, "require_imu_before_locomotion", true);

    auto default_pose_param = getOrDeclareParameter<std::vector<double>>(
        node_, "default_pose", std::vector<double>(NUM_ACTIONS, 0.0));
    auto dof_indices_param = getOrDeclareParameter<std::vector<int64_t>>(
        node_, "dof_indices", std::vector<int64_t>{5, 4, 3, 2, 1, 0, 11, 10, 9, 8, 7, 6});

    if (frame_stack_ <= 0)
    {
        RCLCPP_WARN(node_->get_logger(), "Invalid frame_stack=%d, fallback to 15", frame_stack_);
        frame_stack_ = 15;
    }

    if (model_obs_dim_ != NUM_SINGLE_OBS * frame_stack_)
    {
        if (model_obs_dim_ > 0 && model_obs_dim_ % NUM_SINGLE_OBS == 0)
        {
            frame_stack_ = model_obs_dim_ / NUM_SINGLE_OBS;
            RCLCPP_WARN(node_->get_logger(),
                        "Adjusted frame_stack to %d based on model_obs_dim=%d",
                        frame_stack_, model_obs_dim_);
        }
        else
        {
            model_obs_dim_ = NUM_SINGLE_OBS * frame_stack_;
            RCLCPP_WARN(node_->get_logger(),
                        "Invalid model_obs_dim, fallback to %d",
                        model_obs_dim_);
        }
    }

    default_pose_ = Eigen::VectorXf::Zero(NUM_ACTIONS);
    if (default_pose_param.size() != NUM_ACTIONS)
    {
        RCLCPP_WARN(node_->get_logger(), "default_pose size=%zu, expected=%d, fallback to zeros",
                    default_pose_param.size(), NUM_ACTIONS);
    }
    else
    {
        for (int i = 0; i < NUM_ACTIONS; ++i)
        {
            default_pose_(i) = static_cast<float>(default_pose_param[static_cast<size_t>(i)]);
        }
    }

    if (dof_indices_param.size() != NUM_ACTIONS)
    {
        RCLCPP_WARN(node_->get_logger(), "dof_indices size=%zu, expected=%d, using defaults",
                    dof_indices_param.size(), NUM_ACTIONS);
        dof_indices_ = {5, 4, 3, 2, 1, 0, 11, 10, 9, 8, 7, 6};
    }
    else
    {
        for (int i = 0; i < NUM_ACTIONS; ++i)
        {
            dof_indices_[i] = static_cast<int>(dof_indices_param[static_cast<size_t>(i)]);
        }
    }

    obs_current_ = Eigen::VectorXf::Zero(NUM_SINGLE_OBS);
    policy_input_ = Eigen::VectorXf::Zero(model_obs_dim_);
    act_prev_ = Eigen::VectorXf::Zero(NUM_ACTIONS);
    act_scaled_ = Eigen::VectorXf::Zero(NUM_ACTIONS);
    act_temp_ = Eigen::VectorXf::Zero(NUM_ACTIONS);

    obs_history_.clear();
    for (int i = 0; i < frame_stack_; ++i)
    {
        obs_history_.push_back(Eigen::VectorXf::Zero(NUM_SINGLE_OBS));
    }

    RCLCPP_INFO(node_->get_logger(),
                "PTLocomotion params: dt=%.4f, frame_stack=%d, model_obs_dim=%d, action_scale=%.3f",
                dt_, frame_stack_, model_obs_dim_, action_scale_);
}

void PTLocomotion::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_received_ = true;
    RCLCPP_INFO_ONCE(node_->get_logger(), "Received IMU data, PTLocomotion IMU gate unlocked");

    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    if (q.length2() < 1e-12)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "Received invalid IMU orientation (near-zero quaternion), keeping previous attitude estimate");
    }

    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    current_angular_velocity_(0) = static_cast<float>(msg->angular_velocity.x);
    current_angular_velocity_(1) = static_cast<float>(msg->angular_velocity.y);
    current_angular_velocity_(2) = static_cast<float>(msg->angular_velocity.z);

    if (q.length2() >= 1e-12)
    {
        q.normalize();
        tf2::Matrix3x3 rot(q);

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        rot.getRPY(roll, pitch, yaw);
        current_euler_(0) = static_cast<float>(roll);
        current_euler_(1) = static_cast<float>(pitch);
        current_euler_(2) = static_cast<float>(yaw);
    }
}

void PTLocomotion::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    current_command_(0) = static_cast<float>(msg->linear.x);
    current_command_(1) = static_cast<float>(msg->linear.y);
    current_command_(2) = static_cast<float>(msg->angular.z);
}

void PTLocomotion::collectObservations()
{
    if (phase_period_ <= 1e-6)
    {
        phase_period_ = 0.64;
    }

    Eigen::Vector3f command;
    Eigen::Vector3f omega;
    Eigen::Vector3f euler;
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        command = current_command_;
        omega = current_angular_velocity_;
        euler = current_euler_;
    }

    const double phase = 2.0 * M_PI * static_cast<double>(counter_) * dt_ / phase_period_;
    obs_current_.setZero();
    obs_current_(0) = static_cast<float>(std::sin(phase));
    obs_current_(1) = static_cast<float>(std::cos(phase));
    obs_current_(2) = command(0) * obs_scale_lin_vel_;
    obs_current_(3) = command(1) * obs_scale_lin_vel_;
    obs_current_(4) = command(2) * obs_scale_ang_vel_;

    for (int i = 0; i < NUM_ACTIONS; ++i)
    {
        const int idx = dof_indices_[i];
        if (idx < 0 || idx >= NUM_MOTORS)
        {
            continue;
        }

        const float q_rel = feedback_positions_[static_cast<size_t>(idx)] - default_pose_(i);
        const float dq = feedback_velocities_[static_cast<size_t>(idx)];

        obs_current_(5 + i) = q_rel * obs_scale_dof_pos_;
        obs_current_(17 + i) = dq * obs_scale_dof_vel_;
        obs_current_(29 + i) = act_prev_(i);
    }

    obs_current_.segment(41, 3) = omega;
    obs_current_.segment(44, 3) = euler;

    for (int i = 0; i < NUM_SINGLE_OBS; ++i)
    {
        obs_current_(i) = std::clamp(obs_current_(i), -obs_clip_, obs_clip_);
    }

    obs_history_.push_back(obs_current_);
    while (static_cast<int>(obs_history_.size()) > frame_stack_)
    {
        obs_history_.pop_front();
    }
}

void PTLocomotion::runInference()
{
    if (!model_ready_)
    {
        return;
    }

    if (static_cast<int>(obs_history_.size()) < frame_stack_)
    {
        return;
    }

    if (policy_input_.size() != model_obs_dim_)
    {
        policy_input_ = Eigen::VectorXf::Zero(model_obs_dim_);
    }
    policy_input_.setZero();

    int offset = 0;
    for (int i = 0; i < frame_stack_ && i < static_cast<int>(obs_history_.size()); ++i)
    {
        if (offset + NUM_SINGLE_OBS > model_obs_dim_)
        {
            break;
        }
        policy_input_.segment(offset, NUM_SINGLE_OBS) = obs_history_[static_cast<size_t>(i)];
        offset += NUM_SINGLE_OBS;
    }

    auto input = torch::from_blob(
        policy_input_.data(),
        {1, model_obs_dim_},
        torch::TensorOptions().dtype(torch::kFloat32))
                     .clone();

    auto output_ivalue = policy_module_.forward({input});
    if (!output_ivalue.isTensor())
    {
        throw std::runtime_error("PT model output is not a tensor");
    }

    auto output_tensor = output_ivalue.toTensor().to(torch::kCPU).contiguous();
    if (output_tensor.numel() < NUM_ACTIONS)
    {
        throw std::runtime_error("PT model output dim is smaller than 12 actions");
    }

    const float* output_data = output_tensor.data_ptr<float>();
    Eigen::VectorXf raw_action = Eigen::VectorXf::Zero(NUM_ACTIONS);
    Eigen::VectorXf scaled_action = Eigen::VectorXf::Zero(NUM_ACTIONS);
    for (int i = 0; i < NUM_ACTIONS; ++i)
    {
        const float clipped = std::clamp(output_data[static_cast<size_t>(i)], -act_clip_, act_clip_);
        raw_action(i) = clipped;
        scaled_action(i) = clipped * action_scale_;
    }

    {
        std::lock_guard<std::mutex> lock(action_mutex_);
        act_scaled_ = scaled_action;
    }
    act_prev_ = raw_action;
}

void PTLocomotion::inferenceLoop()
{
    using Clock = std::chrono::steady_clock;
    while (thread_running_)
    {
        auto start = Clock::now();

        ++counter_;
        collectObservations();

        try
        {
            runInference();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "PT inference failed: %s", e.what());
            thread_running_ = false;
            break;
        }

        auto end = start + std::chrono::duration<double>(dt_);
        std::this_thread::sleep_until(end);
    }
}
