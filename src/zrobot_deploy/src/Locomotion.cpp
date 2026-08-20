#include "zrobot_deploy/Locomotion.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace {
// 参数已存在时直接读取，避免重复 declare_parameter 抛出
// ParameterAlreadyDeclaredException（FSM 会被反复创建/销毁）
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
    startup_hold_seconds_(3.0),
    action_delta_limit_(0.08f),
    imu_timeout_(0.5),
      dt_(0.01),
            phase_period_(0.64),
            frame_stack_(15),
            model_obs_dim_(NUM_SINGLE_OBS * 15),
            action_scale_{0.15f, 0.25f, 0.25f, 0.25f, 0.1f, 0.1f, 0.15f, 0.25f, 0.25f, 0.25f, 0.1f, 0.1f},
            obs_clip_(18.0f),
            act_clip_(18.0f),
            obs_scale_lin_vel_(2.0f),
            obs_scale_ang_vel_(1.0f),
            obs_scale_dof_pos_(1.0f),
            obs_scale_dof_vel_(0.05f),
            counter_(0),
            model_ready_(false)
{
        current_angular_velocity_.setZero();
        current_gravity_vector_ << 0.0f, 0.0f, -1.0f;
        current_euler_.setZero();
        current_command_.setZero();

        dof_indices_ = {5, 4, 3, 2, 0, 1, 11, 10, 9, 8, 6, 7};
        joint_signs_.fill(1.0f);
        joint_signs_[0] = -1.0f;  // L_hip_roll
        joint_signs_[2] = -1.0f;  // L_hip_pitch
        joint_signs_[6] = -1.0f;  // R_hip_roll
        joint_signs_[8] = -1.0f;  // R_hip_pitch
        joint_signs_[10] = -1.0f; // R_foot_pitch（右踝非镜像机构方向与 URDF 相反）
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
        getOrDeclareParameter<std::string>(node_, "imu_topic", "/imu/data"),
        rclcpp::SensorDataQoS(),
        std::bind(&Locomotion::imuCallback, this, std::placeholders::_1));

    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        getOrDeclareParameter<std::string>(node_, "cmd_topic", "cmd_vel"),
        10,
        std::bind(&Locomotion::cmdCallback, this, std::placeholders::_1));

    // 加载 ONNX 模型
    const auto model_path = getOrDeclareParameter<std::string>(
        node_, "onnx_model_path", std::string(POLICY_DIR) + "/policy.onnx");
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
        std::array<float, 23> hold_positions{};
        for (int i = 0; i < NUM_ACTIONS; ++i)
        {
            if (i == IDX_L_FOOT_PITCH || i == IDX_L_FOOT_ROLL ||
                i == IDX_R_FOOT_PITCH || i == IDX_R_FOOT_ROLL)
            {
                continue;
            }
            const int motor_idx = dof_indices_[i];
            if (motor_idx >= 0 && motor_idx < NUM_MOTORS)
            {
                hold_positions[static_cast<size_t>(motor_idx)] = joint_signs_[i] * default_pose_(i);
            }
        }
        applyAnkleIK(hold_positions, IDX_L_FOOT_PITCH, IDX_L_FOOT_ROLL);
        applyAnkleIK(hold_positions, IDX_R_FOOT_PITCH, IDX_R_FOOT_ROLL);
        if (!sendMotorPositions(hold_positions))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to hold default pose during startup warmup");
        }
        return;
    }

    // IMU 新鲜度检查：未收到或数据超时（imu_timeout_ 秒）都保持默认姿态
    std::chrono::steady_clock::time_point last_imu_time;
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        last_imu_time = last_imu_time_;
    }
    const double imu_elapsed = std::chrono::duration<double>(Clock::now() - last_imu_time).count();
    if (require_imu_before_locomotion_ && (!imu_received_ || imu_elapsed > imu_timeout_))
    {
        if (imu_received_)
        {
            // IMU 失联（超时）: 停止推理并请求切换为 Damping（围绕当前姿态阻尼，安全停机）
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                2000,
                "IMU data stale (%.1fs > %.1fs), switching to Damping in Locomotion",
                imu_elapsed, imu_timeout_);
            requestSwitch(FSMState::DAMPING);
            exit();  // 立即停止推理线程，等待主循环接管切换
        }
        else
        {
            // 从未收到 IMU（启动阶段）: 保持默认姿态等待
            std::array<float, 23> hold_positions{};
            for (int i = 0; i < NUM_ACTIONS; ++i)
            {
                if (i == IDX_L_FOOT_PITCH || i == IDX_L_FOOT_ROLL ||
                    i == IDX_R_FOOT_PITCH || i == IDX_R_FOOT_ROLL)
                {
                    continue;
                }
                const int motor_idx = dof_indices_[i];
                if (motor_idx >= 0 && motor_idx < NUM_MOTORS)
                {
                    hold_positions[static_cast<size_t>(motor_idx)] = joint_signs_[i] * default_pose_(i);
                }
            }
            applyAnkleIK(hold_positions, IDX_L_FOOT_PITCH, IDX_L_FOOT_ROLL);
            applyAnkleIK(hold_positions, IDX_R_FOOT_PITCH, IDX_R_FOOT_ROLL);
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                2000,
                "IMU data not received yet, holding default pose in Locomotion");
            if (!sendMotorPositions(hold_positions))
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to hold default pose while waiting IMU");
            }
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
        // 踝关节 pitch/roll 由 IK 统一处理
        if (i == IDX_L_FOOT_PITCH || i == IDX_L_FOOT_ROLL ||
            i == IDX_R_FOOT_PITCH || i == IDX_R_FOOT_ROLL)
        {
            continue;
        }
        const int motor_idx = dof_indices_[i];
        if (motor_idx < 0 || motor_idx >= NUM_MOTORS)
        {
            continue;
        }
        positions[static_cast<size_t>(motor_idx)] = joint_signs_[i] * (default_pose_(i) + act_temp_(i));
    }

    // 通过并联踝关节 IK 将 pitch/roll 转为电机角度
    applyAnkleIK(positions, IDX_L_FOOT_PITCH, IDX_L_FOOT_ROLL);
    applyAnkleIK(positions, IDX_R_FOOT_PITCH, IDX_R_FOOT_ROLL);

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
    dt_ = getOrDeclareParameter<double>(node_, "control_dt", 0.01);
    phase_period_ = getOrDeclareParameter<double>(node_, "phase_period", 0.64);
    frame_stack_ = getOrDeclareParameter<int>(node_, "frame_stack", 15);
    model_obs_dim_ = getOrDeclareParameter<int>(node_, "model_obs_dim", NUM_SINGLE_OBS * frame_stack_);

    auto action_scale_param = getOrDeclareParameter<std::vector<double>>(
        node_, "action_scale", std::vector<double>{0.15, 0.25, 0.25, 0.25, 0.1, 0.1, 0.15, 0.25, 0.25, 0.25, 0.1, 0.1});
    if (action_scale_param.size() == NUM_ACTIONS)
    {
        for (int i = 0; i < NUM_ACTIONS; ++i)
        {
            action_scale_[i] = static_cast<float>(action_scale_param[static_cast<size_t>(i)]);
        }
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(),
                    "action_scale size=%zu, expected=%d, using defaults",
                    action_scale_param.size(), NUM_ACTIONS);
    }
    obs_clip_ = static_cast<float>(getOrDeclareParameter<double>(node_, "clip_observations", 18.0));
    act_clip_ = static_cast<float>(getOrDeclareParameter<double>(node_, "clip_actions", 18.0));

    obs_scale_lin_vel_ = static_cast<float>(getOrDeclareParameter<double>(node_, "obs_scale_lin_vel", 2.0));
    obs_scale_ang_vel_ = static_cast<float>(getOrDeclareParameter<double>(node_, "obs_scale_ang_vel", 1.0));
    obs_scale_dof_pos_ = static_cast<float>(getOrDeclareParameter<double>(node_, "obs_scale_dof_pos", 1.0));
    obs_scale_dof_vel_ = static_cast<float>(getOrDeclareParameter<double>(node_, "obs_scale_dof_vel", 0.05));

    startup_hold_seconds_ = getOrDeclareParameter<double>(node_, "startup_hold_seconds", 3.0);
    require_imu_before_locomotion_ = getOrDeclareParameter<bool>(node_, "require_imu_before_locomotion", true);
    imu_timeout_ = getOrDeclareParameter<double>(node_, "imu_timeout", 0.5);

    enable_action_safety_ = getOrDeclareParameter<bool>(node_, "enable_action_safety", true);
    action_delta_limit_ = static_cast<float>(getOrDeclareParameter<double>(node_, "action_delta_limit", 0.08));

    auto default_pose_param = getOrDeclareParameter<std::vector<double>>(
        node_, "default_pose", std::vector<double>{
            0.0, 0.0, -0.35, -0.65, 0.3, 0.0,   // L: hip_roll, hip_yaw, hip_pitch, knee, foot_pitch, foot_roll
            0.0, 0.0, 0.35, 0.65, -0.3, 0.0});  // R: hip_roll, hip_yaw, hip_pitch, knee, foot_pitch, foot_roll
    auto dof_indices_param = getOrDeclareParameter<std::vector<int64_t>>(
        node_, "dof_indices", std::vector<int64_t>{5, 4, 3, 2, 0, 1, 11, 10, 9, 8, 6, 7});
    auto joint_signs_param = getOrDeclareParameter<std::vector<double>>(
        node_, "joint_signs", std::vector<double>{
            -1.0, 1.0, -1.0, 1.0, 1.0, 1.0,   // L: hip_roll, hip_yaw, hip_pitch, knee, foot_pitch, foot_roll
            -1.0, 1.0, -1.0, 1.0, -1.0, 1.0}); // R: hip_roll, hip_yaw, hip_pitch, knee, foot_pitch(机构反向), foot_roll

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
        dof_indices_ = {5, 4, 3, 2, 0, 1, 11, 10, 9, 8, 6, 7};
    }
    else
    {
        for (int i = 0; i < NUM_ACTIONS; ++i)
        {
            dof_indices_[i] = static_cast<int>(dof_indices_param[static_cast<size_t>(i)]);
        }
    }

    if (joint_signs_param.size() != NUM_ACTIONS)
    {
        RCLCPP_WARN(node_->get_logger(), "joint_signs size=%zu, expected=%d, using defaults",
                    joint_signs_param.size(), NUM_ACTIONS);
        joint_signs_.fill(1.0f);
        joint_signs_[0] = -1.0f;  // L_hip_roll
        joint_signs_[2] = -1.0f;  // L_hip_pitch
        joint_signs_[6] = -1.0f;  // R_hip_roll
        joint_signs_[8] = -1.0f;  // R_hip_pitch
        joint_signs_[10] = -1.0f; // R_foot_pitch（右踝非镜像机构方向与 URDF 相反）
    }
    else
    {
        for (int i = 0; i < NUM_ACTIONS; ++i)
        {
            joint_signs_[i] = static_cast<float>(joint_signs_param[static_cast<size_t>(i)]);
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
                "Locomotion params: dt=%.4f, frame_stack=%d, model_obs_dim=%d, "
                "action_scale=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f|%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                dt_, frame_stack_, model_obs_dim_,
                action_scale_[0], action_scale_[1], action_scale_[2], action_scale_[3],
                action_scale_[4], action_scale_[5],
                action_scale_[6], action_scale_[7], action_scale_[8], action_scale_[9],
                action_scale_[10], action_scale_[11]);
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
    last_imu_time_ = std::chrono::steady_clock::now();  // 记录新鲜度时间戳
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
    // 限幅到训练指令分布 (zrobot_config.py commands.ranges):
    // vx ∈ [-0.3, 0.6], vy ∈ [-0.3, 0.3],
    // wz ∈ [-1.0, 1.0] (训练中 heading 模式下由航向误差重算并 clip 到 ±1)
    current_command_(0) = std::clamp(static_cast<float>(msg->linear.x), -0.3f, 0.6f);
    current_command_(1) = std::clamp(static_cast<float>(msg->linear.y), -0.3f, 0.3f);
    current_command_(2) = std::clamp(static_cast<float>(msg->angular.z), -1.0f, 1.0f);
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
        // 踝关节由下文 FK 统一处理，此处跳过
        if (i == IDX_L_FOOT_PITCH || i == IDX_L_FOOT_ROLL ||
            i == IDX_R_FOOT_PITCH || i == IDX_R_FOOT_ROLL)
        {
            continue;
        }

        const int idx = dof_indices_[i];
        if (idx < 0 || idx >= NUM_MOTORS)
        {
            continue;
        }

        float q, dq;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            q = feedback_positions_[static_cast<size_t>(idx)];
            dq = feedback_velocities_[static_cast<size_t>(idx)];
        }
        const float sign = joint_signs_[i];
        const float q_rel = q * sign - default_pose_(i);

        obs_current_(5 + i) = q_rel * obs_scale_dof_pos_;
        obs_current_(17 + i) = dq * sign * obs_scale_dof_vel_;
        obs_current_(29 + i) = act_prev_(i);
    }

    // 踝关节通过正运动学将电机角度 (θ₁,θ₂) 转回 (pitch,roll) 再构建观测
    {
        float t1, t2, dt1, dt2;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            // 左脚: motor[0]=θ₁(pitch 电机), motor[1]=θ₂(roll 电机)
            t1  = feedback_positions_[static_cast<size_t>(dof_indices_[IDX_L_FOOT_PITCH])];
            t2  = feedback_positions_[static_cast<size_t>(dof_indices_[IDX_L_FOOT_ROLL])];
            dt1 = feedback_velocities_[static_cast<size_t>(dof_indices_[IDX_L_FOOT_PITCH])];
            dt2 = feedback_velocities_[static_cast<size_t>(dof_indices_[IDX_L_FOOT_ROLL])];
        }
        const auto fwL = ankle_ik_.fw(static_cast<double>(prev_L_pitch_),
                                       static_cast<double>(prev_L_roll_), t1, t2);
        if (fwL.error_state == 0) {
            prev_L_pitch_ = static_cast<float>(fwL.pitch);
            prev_L_roll_  = static_cast<float>(fwL.roll);
        }
        const float s4 = joint_signs_[4], s5 = joint_signs_[5];
        // IK 约定: pitch→策略索引4(L_foot_pitch), roll→策略索引5(L_foot_roll)
        // 与主循环一致: obs = 测量×sign - default（只翻转测量，不翻转 default）
        obs_current_(5 + 4)  = (prev_L_pitch_ * s4 - default_pose_(4)) * obs_scale_dof_pos_;
        obs_current_(5 + 5)  = (prev_L_roll_  * s5 - default_pose_(5)) * obs_scale_dof_pos_;
        // 速度: 经雅可比将电机速度转为机构关节速度（与位置 FK 对齐）
        double L_pitch_vel = 0.0, L_roll_vel = 0.0;
        if (!ankle_ik_.motorVelToJointVel(static_cast<double>(prev_L_pitch_),
                                          static_cast<double>(prev_L_roll_),
                                          t1, t2, dt1, dt2,
                                          L_pitch_vel, L_roll_vel)) {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "Left ankle velocity Jacobian failed, using motor velocity approximation");
            L_pitch_vel = dt1;
            L_roll_vel  = dt2;
        }
        obs_current_(17 + 4) = static_cast<float>(L_pitch_vel * s4) * obs_scale_dof_vel_;
        obs_current_(17 + 5) = static_cast<float>(L_roll_vel  * s5) * obs_scale_dof_vel_;
        obs_current_(29 + 4) = act_prev_(4);
        obs_current_(29 + 5) = act_prev_(5);
    }
    {
        float t1, t2, dt1, dt2;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            // 右脚: motor[6]=θ₁(pitch 电机), motor[7]=θ₂(roll 电机)
            t1  = feedback_positions_[static_cast<size_t>(dof_indices_[IDX_R_FOOT_PITCH])];
            t2  = feedback_positions_[static_cast<size_t>(dof_indices_[IDX_R_FOOT_ROLL])];
            dt1 = feedback_velocities_[static_cast<size_t>(dof_indices_[IDX_R_FOOT_PITCH])];
            dt2 = feedback_velocities_[static_cast<size_t>(dof_indices_[IDX_R_FOOT_ROLL])];
        }
        const auto fwR = ankle_ik_.fw(static_cast<double>(prev_R_pitch_),
                                       static_cast<double>(prev_R_roll_), t1, t2);
        if (fwR.error_state == 0) {
            prev_R_pitch_ = static_cast<float>(fwR.pitch);   // 机构约定
            prev_R_roll_  = static_cast<float>(fwR.roll);
        }
        const float s10 = joint_signs_[10], s11 = joint_signs_[11];
        // 右踝机构方向翻转已由 joint_signs_[10] = -1 吸收，公式与左踝一致
        // obs 按 URDF 约定: obs = 测量×sign - default
        obs_current_(5 + 10)  = (prev_R_pitch_ * s10 - default_pose_(10)) * obs_scale_dof_pos_;
        obs_current_(5 + 11)  = (prev_R_roll_  * s11 - default_pose_(11)) * obs_scale_dof_pos_;
        // 速度: 经雅可比将电机速度转为机构关节速度（与位置 FK 对齐）
        double R_pitch_vel = 0.0, R_roll_vel = 0.0;
        if (!ankle_ik_.motorVelToJointVel(static_cast<double>(prev_R_pitch_),
                                          static_cast<double>(prev_R_roll_),
                                          t1, t2, dt1, dt2,
                                          R_pitch_vel, R_roll_vel)) {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "Right ankle velocity Jacobian failed, using motor velocity approximation");
            R_pitch_vel = dt1;
            R_roll_vel  = dt2;
        }
        obs_current_(17 + 10) = static_cast<float>(R_pitch_vel * s10) * obs_scale_dof_vel_;
        obs_current_(17 + 11) = static_cast<float>(R_roll_vel  * s11) * obs_scale_dof_vel_;
        obs_current_(29 + 10) = act_prev_(10);
        obs_current_(29 + 11) = act_prev_(11);
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
        float clipped = std::clamp(output_data[static_cast<size_t>(i)], -act_clip_, act_clip_);
        if (enable_action_safety_)
        {
            // 增量限幅：限制相邻帧动作突变（100Hz 控制率下的平滑约束）
            const float delta = clipped - act_prev_(i);
            clipped = act_prev_(i) + std::clamp(delta, -action_delta_limit_, action_delta_limit_);
        }
        raw_action(i) = clipped;
        scaled_action(i) = clipped * action_scale_[i];
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

void Locomotion::applyAnkleIK(std::array<float, 23>& positions,
                               int policy_idx_pitch, int policy_idx_roll)
{
    const int motor_idx_pitch = dof_indices_[policy_idx_pitch];
    const int motor_idx_roll  = dof_indices_[policy_idx_roll];

    if (motor_idx_pitch < 0 || motor_idx_pitch >= NUM_MOTORS ||
        motor_idx_roll  < 0 || motor_idx_roll  >= NUM_MOTORS)
    {
        return;
    }

    // 与主循环一致: joint_signs_ 乘以整个 (default + act)（硬件/机构方向翻转）
    // 右踝非镜像机构的方向翻转已由 joint_signs_[10] = -1 吸收
    double pitch = joint_signs_[policy_idx_pitch]
                   * static_cast<double>(default_pose_(policy_idx_pitch) + act_temp_(policy_idx_pitch));
    double roll  = joint_signs_[policy_idx_roll]
                   * static_cast<double>(default_pose_(policy_idx_roll) + act_temp_(policy_idx_roll));

    // 钳位到机械限位范围
    if (pitch < ParallelAnkle::PITCH_MIN || pitch > ParallelAnkle::PITCH_MAX ||
        roll  < ParallelAnkle::ROLL_MIN  || roll  > ParallelAnkle::ROLL_MAX)
    {
        const double raw_pitch = pitch, raw_roll = roll;
        pitch = std::clamp(pitch, ParallelAnkle::PITCH_MIN, ParallelAnkle::PITCH_MAX);
        roll  = std::clamp(roll,  ParallelAnkle::ROLL_MIN,  ParallelAnkle::ROLL_MAX);
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "Ankle pitch/roll clamped: pitch %.3f→%.3f, roll %.3f→%.3f",
            raw_pitch, pitch, raw_roll, roll);
    }

    const auto ik_result = ankle_ik_.ik(pitch, roll);

    positions[static_cast<size_t>(motor_idx_pitch)] = static_cast<float>(ik_result.theta1);
    positions[static_cast<size_t>(motor_idx_roll)]  = static_cast<float>(ik_result.theta2);

    if (ik_result.error_state != 0)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "Ankle IK warning: pitch=%.3f, roll=%.3f, error_state=%d",
            pitch, roll, ik_result.error_state);
    }
}
