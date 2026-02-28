#include "zrobot_deploy/Locomotion.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>
#include <cmath>

Locomotion::Locomotion(std::shared_ptr<rclcpp::Node> node)
    : FSM(node),
      ort_env_(nullptr),
      ort_session_(nullptr),
      memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
      thread_running_(false),
      dt_(0.01),
      phase_period_(0.8),
      counter_(0)
{
    current_state_ = FSMState::IDLE;
    // 观测数据
    obs_current_.setZero(NUM_OBSERVATIONS);
    obs_scaled_.setZero(NUM_OBSERVATIONS);
    obs_mean_.setZero(NUM_OBSERVATIONS);
    obs_scales_.setZero(NUM_OBSERVATIONS);
    // 动作数据
    act_prev_.setZero(NUM_ACTIONS);
    act_scaled_.setZero(NUM_ACTIONS);
    act_temp_.setZero(NUM_ACTIONS);
    act_mean_.setZero(NUM_ACTIONS);
    act_scales_.setZero(NUM_ACTIONS);
    // 控制参数
    stiffness_.setZero(NUM_ACTIONS);
    damping_.setZero(NUM_ACTIONS);
    // 传感器数据初始化
    current_angular_velocity_.setZero();
    current_gravity_vector_.setZero();
    current_command_.setZero();

    RCLCPP_INFO(node_->get_logger(), "Locomotion FSM created");
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
        node_->declare_parameter<std::string>("imu_topic", "imu/data"),
        10,
        std::bind(&Locomotion::imuCallback, this, std::placeholders::_1));

    cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        node_->declare_parameter<std::string>("cmd_topic", "cmd_vel"),
        10,
        std::bind(&Locomotion::cmdCallback, this, std::placeholders::_1));

    // 加载 ONNX 模型
    const auto model_path = node_->declare_parameter<std::string>(
        "onnx_model_path", "/root/codes/zrobot_pi_ws/models/policy.onnx");
    loadPolicy(model_path);
    
    // 启动推理线程
    counter_ = 0;
    thread_running_ = true;
    inference_thread_ = std::thread(&Locomotion::inferenceLoop, this);

    current_state_ = FSMState::IDLE;
    RCLCPP_INFO(node_->get_logger(), "Locomotion initialized and inference started");
}

void Locomotion::run()
{
    if (!thread_running_)
    {
        RCLCPP_WARN(node_->get_logger(), "Locomotion inference thread not running");
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
        positions[i] = act_temp_(i);
    }

    if (!sendMotorPositions(positions))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send motor positions in Locomotion");
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
    // 初始化ONNX Runtime环境
    ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "locomotion");
    session_options_.SetIntraOpNumThreads(2);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);

    ort_session_ = std::make_unique<Ort::Session>(*ort_env_, model_path.c_str(), session_options_);
    // 使用默认的内存分配器
    Ort::AllocatorWithDefaultOptions allocator;
    char *input_name = ort_session_->GetInputName(0, allocator);
    char *output_name = ort_session_->GetOutputName(0, allocator);
    input_names_storage_.emplace_back(input_name);
    output_names_storage_.emplace_back(output_name);
    allocator.Free(input_name);
    allocator.Free(output_name);

    input_names_.clear();
    output_names_.clear();
    for (const auto &name : input_names_storage_)
    {
        input_names_.push_back(name.c_str());
    }
    for (const auto &name : output_names_storage_)
    {
        output_names_.push_back(name.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "Loaded ONNX model: %s", model_path.c_str());
}

void Locomotion::initializeParameters()
{
    // 读取参数
    dt_ = node_->declare_parameter<double>("control_dt", 0.01);
    phase_period_ = node_->declare_parameter<double>("phase_period", 0.8);

    std::vector<double> default_angles = node_->declare_parameter<std::vector<double>>(
        "default_angles", std::vector<double>(NUM_ACTIONS, 0.0));

    std::vector<double> kps = node_->declare_parameter<std::vector<double>>(
        "kps", std::vector<double>(NUM_ACTIONS, 30.0));
    std::vector<double> kds = node_->declare_parameter<std::vector<double>>(
        "kds", std::vector<double>(NUM_ACTIONS, 1.0));
    
    obs_mean_.setZero(NUM_OBSERVATIONS);
    default_angles.resize(NUM_ACTIONS, 0.0);
    kps.resize(NUM_ACTIONS, 30.0);
    kds.resize(NUM_ACTIONS, 1.0);
    for (int i = 0; i < NUM_ACTIONS; ++i)
    {
        obs_mean_(9 + i) = static_cast<float>(default_angles[i]);
    }

    // 缩放系数、读取自配置文件
    double obs_scale_ang_vel = node_->declare_parameter<double>("ang_vel_scale", 1.0);
    double obs_scale_dof_pos = node_->declare_parameter<double>("dof_pos_scale", 1.0);
    double obs_scale_dof_vel = node_->declare_parameter<double>("dof_vel_scale", 1.0);
    double action_scale = node_->declare_parameter<double>("action_scale", 1.0);

    obs_scales_.setZero(NUM_OBSERVATIONS);
    obs_scales_.segment<3>(0) = Eigen::Vector3f::Ones() * static_cast<float>(obs_scale_ang_vel);
    obs_scales_.segment<3>(3) = Eigen::Vector3f::Ones();
    obs_scales_.segment<3>(6) = Eigen::Vector3f::Ones();
    obs_scales_.segment<12>(9) = Eigen::Vector<float, 12>::Ones() * static_cast<float>(obs_scale_dof_pos);
    obs_scales_.segment<12>(21) = Eigen::Vector<float, 12>::Ones() * static_cast<float>(obs_scale_dof_vel);
    obs_scales_.segment<12>(33) = Eigen::Vector<float, 12>::Ones();
    obs_scales_.segment<2>(45) = Eigen::Vector2f::Ones();

    // 动作均值和缩放
    act_mean_.setZero(NUM_ACTIONS);
    for (int i = 0; i < NUM_ACTIONS; ++i)
    {
        act_mean_(i) = static_cast<float>(default_angles[i]);
    }

    act_scales_.setZero(NUM_ACTIONS);
    act_scales_.segment<12>(0) = Eigen::Vector<float, 12>::Ones() * static_cast<float>(action_scale);

    for (int i = 0; i < NUM_ACTIONS; ++i)
    {
        stiffness_(i) = static_cast<float>(kps[i]);
        damping_(i) = static_cast<float>(kds[i]);
    }

    RCLCPP_INFO(node_->get_logger(), "Locomotion parameters initialized");
}

void Locomotion::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    current_angular_velocity_(0) = static_cast<float>(msg->angular_velocity.x);
    current_angular_velocity_(1) = static_cast<float>(msg->angular_velocity.y);
    current_angular_velocity_(2) = static_cast<float>(msg->angular_velocity.z);

    // 通过姿态计算重力向量在机体坐标系下的投影
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf2::Matrix3x3 rot(q);
    tf2::Vector3 gravity_world(0.0, 0.0, -1.0);
    tf2::Vector3 gravity_body = rot.transpose() * gravity_world;
    current_gravity_vector_(0) = static_cast<float>(gravity_body.x());
    current_gravity_vector_(1) = static_cast<float>(gravity_body.y());
    current_gravity_vector_(2) = static_cast<float>(gravity_body.z());
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
    obs_current_.setZero(NUM_OBSERVATIONS);

    // 读取传感器数据
    {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        obs_current_.segment<3>(0) = current_angular_velocity_;
        obs_current_.segment<3>(3) = current_gravity_vector_;
        obs_current_.segment<3>(6) = current_command_;
    }

    // 读取电机反馈
    if (getMotorFeedback(feedback_positions_, feedback_velocities_, feedback_torques_, feedback_temperatures_))
    {
        current_motor_positions_ = feedback_positions_;
        for (int j = 0; j < NUM_ACTIONS; ++j)
        {
            obs_current_(9 + j) = feedback_positions_[j];
            obs_current_(21 + j) = feedback_velocities_[j];
        }
    }

    // 历史动作
    obs_current_.segment<12>(33) = act_prev_;

    // 步态周期相位
    double count = static_cast<double>(counter_) * dt_;
    double phase = std::fmod(count, phase_period_) / phase_period_;
    float sin_phase = static_cast<float>(std::sin(2.0 * M_PI * phase));
    float cos_phase = static_cast<float>(std::cos(2.0 * M_PI * phase));
    obs_current_(45) = sin_phase;
    obs_current_(46) = cos_phase;

    // 标准化，obs_mean_和obs_scales_都是来自配置文件
    obs_scaled_ = (obs_current_ - obs_mean_).cwiseProduct(obs_scales_); // 逐元素乘法
}

void Locomotion::runInference()
{
    std::vector<float> input_data(NUM_OBSERVATIONS, 0.0f);
    for (int i = 0; i < NUM_OBSERVATIONS; ++i)
    {
        input_data[i] = obs_scaled_(i);
    }

    // 输入和输出的形状张量
    std::array<int64_t, 2> input_shape{1, NUM_OBSERVATIONS};
    std::array<int64_t, 2> output_shape{1, NUM_ACTIONS};

    // 创建输入和输出张量
    auto input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_,         // 内存信息，指定了张量数据的存储位置和分配器
        input_data.data(),    // 数据指针，指向实际数据缓冲区的指针，即内存起始位置
        input_data.size(),    // 数据大小
        input_shape.data(),   // 形状指针，指向描述张量维度的整数数组
        input_shape.size());  // 形状大小

    std::vector<float> output_data(NUM_ACTIONS, 0.0f);
    auto output_tensor = Ort::Value::CreateTensor<float>(
        memory_info_,
        output_data.data(),
        output_data.size(),
        output_shape.data(),
        output_shape.size());

    // 执行推理
    ort_session_->Run(Ort::RunOptions{nullptr}, // 默认配置
                      input_names_.data(),      // 输入的节点名称数组
                      &input_tensor, 1,         // 输入张量指针和数量
                      output_names_.data(),     // 输出的节点名称数组
                      &output_tensor, 1);       // 输出张量指针和数量

    for (int i = 0; i < NUM_ACTIONS; ++i)
    {
        act_prev_(i) = output_data[i];
    }
    
    // 使用互斥锁确保对共享变量的安全访问
    std::lock_guard<std::mutex> lock(action_mutex_);
    act_scaled_ = act_prev_.cwiseProduct(act_scales_) + act_mean_;
}

void Locomotion::inferenceLoop()
{
    using Clock = std::chrono::steady_clock;
    while (thread_running_)
    {
        auto start = Clock::now();

        ++counter_;
        collectObservations(); // 收集观测数据
        runInference(); // 推理

        auto end = start + std::chrono::duration<double>(dt_);
        std::this_thread::sleep_until(end);
    }
}
