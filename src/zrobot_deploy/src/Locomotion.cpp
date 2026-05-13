#include "zrobot_deploy/Locomotion.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

Locomotion::Locomotion(std::shared_ptr<rclcpp::Node> node)
    : FSM(node),
      ort_env_(nullptr),
      ort_session_(nullptr),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
      thread_running_(false),
    imu_received_(false),
    motor_feedback_ready_(false),
    require_imu_before_locomotion_(true),
    enable_action_safety_(true),
    startup_hold_seconds_(0.5),
    action_delta_limit_(0.08f),
    action_abs_limit_(1.5f),
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
        current_gravity_vector_ << 0.0f, 0.0f, -1.0f;
        current_euler_.setZero();
        current_command_.setZero();

        dof_indices_ = {5, 4, 3, 2, 1, 0, 11, 10, 9, 8, 7, 6};
}

Locomotion::~Locomotion()
{
    exit();
    RCLCPP_INFO(node_->get_logger(), "Locomotion FSM destroyed");
}

// 初始化
void Locomotion::initialize()
{
    FSM::initialize();

    RCLCPP_INFO(node_->get_logger(), "Locomotion initializing...");

    // 初始化参数
    initializeParameters();

    // 订阅 IMU 和遥控命令
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        node_->declare_parameter<std::string>("imu_topic", "/imu/data"),
        rclcpp::SensorDataQoS(),
        std::bind(&Locomotion::imuCallback, this, std::placeholders::_1));

    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        node_->declare_parameter<std::string>("cmd_topic", "cmd_vel"),
        10,
        std::bind(&Locomotion::cmdCallback, this, std::placeholders::_1));

    // 加载 ONNX 模型
    const auto model_path = node_->declare_parameter<std::string>(
        "onnx_model_path", "/home/c112/Codes/policy/policy.onnx");
    loadPolicy(model_path);

    if (!model_ready_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Locomotion model is not ready, inference thread will not start");
        return;
    }
    
    // 启动推理线程
    counter_ = 0;
    init_time_ = std::chrono::steady_clock::now();
    thread_running_ = true;
    inference_thread_ = std::thread(&Locomotion::inferenceLoop, this);

    current_state_ = FSMState::IDLE;
    RCLCPP_INFO(node_->get_logger(), "Locomotion initialized and inference started");
}

void Locomotion::run()
{
    using Clock = std::chrono::steady_clock;

    if (!thread_running_)
    {
        RCLCPP_WARN(node_->get_logger(), "Locomotion inference thread not running");
        return;
    }

    const auto elapsed = std::chrono::duration<double>(Clock::now() - init_time_).count();
    if (elapsed < startup_hold_seconds_)
    {
        if (!sendMotorPositions(current_motor_positions_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to hold motor positions during startup warmup");
        }
        return;
    }

    if (require_imu_before_locomotion_ && !imu_received_)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "IMU data not received yet, holding current pose in Locomotion");
        if (!sendMotorPositions(current_motor_positions_))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to hold motor positions while waiting IMU");
        }
        return;
    }

    // 读取推理输出
    {   // 使用lock_guard的互斥锁，lock 对象离开作用域时自动解锁
        std::lock_guard<std::mutex> lock(action_mutex_);
        act_temp_ = act_scaled_;
    }

    // 发送到电机
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
        RCLCPP_ERROR(node_->get_logger(), "Failed to send motor positions in Locomotion");
    }
    else
    {
        current_motor_positions_ = positions;
    }
}

void Locomotion::exit()
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

void Locomotion::loadPolicy(const std::string &model_path)
{
    model_ready_ = false;

    try
    {
        // 初始化ONNX Runtime环境
        ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "locomotion");
        session_options_.SetIntraOpNumThreads(2);
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);

        ort_session_ = std::make_unique<Ort::Session>(*ort_env_, model_path.c_str(), session_options_);

        const size_t input_count = ort_session_->GetInputCount();
        const size_t output_count = ort_session_->GetOutputCount();
        if (input_count == 0 || output_count == 0)
        {
            throw std::runtime_error("ONNX model has empty input/output");
        }

        input_names_storage_.clear();
        output_names_storage_.clear();
        input_names_.clear();
        output_names_.clear();

        // 使用默认的内存分配器
        Ort::AllocatorWithDefaultOptions allocator;
        auto input_name = ort_session_->GetInputNameAllocated(0, allocator);
        auto output_name = ort_session_->GetOutputNameAllocated(0, allocator);
        input_names_storage_.emplace_back(input_name.get());
        output_names_storage_.emplace_back(output_name.get());

        for (const auto &name : input_names_storage_)
        {
            input_names_.push_back(name.c_str());
        }
        for (const auto &name : output_names_storage_)
        {
            output_names_.push_back(name.c_str());
        }

        // Store TypeInfo first to keep the underlying OrtTypeInfo alive.
        // GetTensorTypeAndShapeInfo() returns a non-owning wrapper whose
        // pointer is invalidated when the TypeInfo temporary is destroyed.
        auto input_type_info = ort_session_->GetInputTypeInfo(0);
        auto output_type_info = ort_session_->GetOutputTypeInfo(0);
        const auto input_info = input_type_info.GetTensorTypeAndShapeInfo();
        const auto output_info = output_type_info.GetTensorTypeAndShapeInfo();
        const auto input_shape = input_info.GetShape();
        const auto output_shape = output_info.GetShape();

        if (!input_shape.empty() && input_shape.back() > 0 &&
            input_shape.back() != static_cast<int64_t>(model_obs_dim_))
        {
            throw std::runtime_error("Model input dim does not match model_obs_dim parameter");
        }

        if (!output_shape.empty() && output_shape.back() > 0 &&
            output_shape.back() < static_cast<int64_t>(NUM_ACTIONS))
        {
            throw std::runtime_error("Model output dim is smaller than NUM_ACTIONS");
        }

        model_ready_ = true;
        RCLCPP_INFO(node_->get_logger(),
                    "Loaded ONNX model: %s, obs_dim=%d, frame_stack=%d",
                    model_path.c_str(), model_obs_dim_, frame_stack_);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load ONNX model: %s", e.what());
        model_ready_ = false;
    }
}

void Locomotion::initializeParameters()
{
    dt_ = node_->declare_parameter<double>("control_dt", 0.01);
    phase_period_ = node_->declare_parameter<double>("phase_period", 0.64);
    frame_stack_ = node_->declare_parameter<int>("frame_stack", 15);
    model_obs_dim_ = node_->declare_parameter<int>("model_obs_dim", NUM_SINGLE_OBS * frame_stack_);

    action_scale_ = static_cast<float>(node_->declare_parameter<double>("action_scale", 0.25));
    obs_clip_ = static_cast<float>(node_->declare_parameter<double>("clip_observations", 100.0));
    act_clip_ = static_cast<float>(node_->declare_parameter<double>("clip_actions", 100.0));

    obs_scale_lin_vel_ = static_cast<float>(node_->declare_parameter<double>("obs_scale_lin_vel", 1.0));
    obs_scale_ang_vel_ = static_cast<float>(node_->declare_parameter<double>("obs_scale_ang_vel", 1.0));
    obs_scale_dof_pos_ = static_cast<float>(node_->declare_parameter<double>("obs_scale_dof_pos", 1.0));
    obs_scale_dof_vel_ = static_cast<float>(node_->declare_parameter<double>("obs_scale_dof_vel", 1.0));

    auto default_pose_param = node_->declare_parameter<std::vector<double>>(
        "default_pose", std::vector<double>(NUM_ACTIONS, 0.0));
    auto dof_indices_param = node_->declare_parameter<std::vector<int64_t>>(
        "dof_indices", std::vector<int64_t>{5, 4, 3, 2, 1, 0, 11, 10, 9, 8, 7, 6});

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
                "Locomotion params: dt=%.4f, frame_stack=%d, model_obs_dim=%d, action_scale=%.3f",
                dt_, frame_stack_, model_obs_dim_, action_scale_);
}

void Locomotion::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_received_ = true;
    RCLCPP_INFO_ONCE(node_->get_logger(), "Received IMU data, Locomotion IMU gate unlocked");

    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    // Skip invalid IMU orientation to avoid propagating NaN into observations.
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

    // 通过姿态计算重力向量在机体坐标系下的投影
    if (q.length2() >= 1e-12)
    {
        q.normalize();
        tf2::Matrix3x3 rot(q);
        tf2::Vector3 gravity_world(0.0, 0.0, -1.0);
        tf2::Vector3 gravity_body = rot.transpose() * gravity_world;
        current_gravity_vector_(0) = static_cast<float>(gravity_body.x());
        current_gravity_vector_(1) = static_cast<float>(gravity_body.y());
        current_gravity_vector_(2) = static_cast<float>(gravity_body.z());

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        rot.getRPY(roll, pitch, yaw);
        current_euler_(0) = static_cast<float>(roll);
        current_euler_(1) = static_cast<float>(pitch);
        current_euler_(2) = static_cast<float>(yaw);
    }
}

void Locomotion::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    current_command_(0) = static_cast<float>(msg->linear.x);
    current_command_(1) = static_cast<float>(msg->linear.y);
    current_command_(2) = static_cast<float>(msg->angular.z);
}

void Locomotion::collectObservations()
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

void Locomotion::runInference()
{
    if (!model_ready_ || !ort_session_)
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

    std::vector<float> input_data(static_cast<size_t>(model_obs_dim_));
    for (int i = 0; i < model_obs_dim_; ++i)
    {
        input_data[static_cast<size_t>(i)] = policy_input_(i);
    }

    std::vector<int64_t> input_shape = {1, static_cast<int64_t>(model_obs_dim_)};
    auto input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, input_data.data(), input_data.size(), input_shape.data(), input_shape.size());

    auto output_tensors = ort_session_->Run(
        Ort::RunOptions{nullptr},
        input_names_.data(),
        &input_tensor,
        1,
        output_names_.data(),
        1);

    if (output_tensors.empty() || !output_tensors[0].IsTensor())
    {
        RCLCPP_WARN(node_->get_logger(), "ONNX inference returned empty output");
        return;
    }

    const auto output_info = output_tensors[0].GetTensorTypeAndShapeInfo();
    const size_t output_count = output_info.GetElementCount();
    if (output_count < static_cast<size_t>(NUM_ACTIONS))
    {
        RCLCPP_WARN(node_->get_logger(), "ONNX output count=%zu, expected >=%d", output_count, NUM_ACTIONS);
        return;
    }

    const float *output_data = output_tensors[0].GetTensorData<float>();
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

void Locomotion::inferenceLoop()
{
    using Clock = std::chrono::steady_clock;
    while (thread_running_)
    {
        auto start = Clock::now();

        ++counter_;
        collectObservations(); // 收集观测数据
        try
        {
            runInference(); // 推理
        }
        catch (const Ort::Exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "ONNX inference failed: %s", e.what());
            thread_running_ = false;
            break;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Inference failed: %s", e.what());
            thread_running_ = false;
            break;
        }

        auto end = start + std::chrono::duration<double>(dt_);
        std::this_thread::sleep_until(end);
    }
}
