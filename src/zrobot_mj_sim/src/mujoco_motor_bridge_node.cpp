#include "zrobot_mj_sim/mujoco_motor_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <cmath>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "mujoco/mujoco.h"
#include <GL/freeglut.h>

namespace
{
constexpr double kDefaultControlFrequency = 200.0;
constexpr double kDefaultKp = 40.0;
constexpr double kDefaultKd = 1.0;
}

MujocoMotorBridgeNode* MujocoMotorBridgeNode::render_instance_ = nullptr;

/**
 * @brief 构造函数。初始化参数、加载 MuJoCo 模型、建立关节索引映射、
 *        注册三个 ROS 2 服务、创建话题发布器、启动控制定时器与 GLUT 渲染线程。
 */
MujocoMotorBridgeNode::MujocoMotorBridgeNode()
    : Node("mujoco_motor_bridge_node"),
      active_joint_count_(kNumMotors),
      control_frequency_(kDefaultControlFrequency),
      sim_substeps_(1),
      feedback_temperature_(35.0f),
      publish_joint_states_(true),
      publish_imu_(true),
      publish_clock_(true),
      has_state_(false),
      is_target_initialized_(false),
      control_mode_(ControlMode::kTorquePd),
      model_(nullptr),
      data_(nullptr)
{
    render_instance_ = this;
    joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", default_joint_names());

    std::vector<double> default_kp(kNumMotors, kDefaultKp);
    std::vector<double> default_kd(kNumMotors, kDefaultKd);
    kp_ = declare_parameter<std::vector<double>>("kp", default_kp);
    kd_ = declare_parameter<std::vector<double>>("kd", default_kd);

    control_mode_param_ = declare_parameter<std::string>("control_mode", "torque_pd");
    control_frequency_ = declare_parameter<double>("control_frequency", kDefaultControlFrequency);
    feedback_temperature_ = static_cast<float>(declare_parameter<double>("feedback_temperature", 35.0));
    publish_joint_states_ = declare_parameter<bool>("publish_joint_states", true);
    publish_imu_ = declare_parameter<bool>("publish_imu", true);
    publish_clock_ = declare_parameter<bool>("publish_clock", true);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu");
    model_path_ = declare_parameter<std::string>("model_path", "");
    position_model_path_ = declare_parameter<std::string>("position_model_path", "");

    if (joint_names_.empty())
    {
        RCLCPP_WARN(get_logger(), "joint_names is empty. Falling back to defaults.");
        joint_names_ = default_joint_names();
    }

    if (joint_names_.size() > kNumMotors)
    {
        RCLCPP_WARN(get_logger(), "joint_names size is %zu, truncating to %zu.", joint_names_.size(), kNumMotors);
        joint_names_.resize(kNumMotors);
    }

    active_joint_count_ = joint_names_.size();

    if (kp_.size() < active_joint_count_)
    {
        RCLCPP_WARN(get_logger(), "kp size (%zu) is less than active joints (%zu). Padding with %.1f.",
                    kp_.size(), active_joint_count_, kDefaultKp);
        kp_.resize(active_joint_count_, kDefaultKp);
    }
    if (kd_.size() < active_joint_count_)
    {
        RCLCPP_WARN(get_logger(), "kd size (%zu) is less than active joints (%zu). Padding with %.1f.",
                    kd_.size(), active_joint_count_, kDefaultKd);
        kd_.resize(active_joint_count_, kDefaultKd);
    }

    if (control_mode_param_ == "position")
    {
        control_mode_ = ControlMode::kPosition;
    }
    else if (control_mode_param_ == "torque_pd")
    {
        control_mode_ = ControlMode::kTorquePd;
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Unknown control_mode '%s', defaulting to torque_pd.", control_mode_param_.c_str());
        control_mode_ = ControlMode::kTorquePd;
    }

    std::string resolved_model_path = model_path_;
    if (resolved_model_path.empty())
    {
        const std::string pkg_share = ament_index_cpp::get_package_share_directory("zrobot_mj_sim");
        if (control_mode_ == ControlMode::kPosition)
        {
            if (position_model_path_.empty())
            {
                resolved_model_path = pkg_share + "/resources/zrobot/mjcf/zrobot_position.xml";
            }
            else
            {
                resolved_model_path = position_model_path_;
            }
        }
        else
        {
            resolved_model_path = pkg_share + "/resources/zrobot/mjcf/zrobot.xml";
        }
    }

    load_model(resolved_model_path);
    build_joint_mappings();

    zero_offsets_.fill(0.0f);
    last_positions_.fill(0.0f);
    last_velocities_.fill(0.0f);
    last_efforts_.fill(0.0f);
    target_positions_.fill(0.0f);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        update_state_from_sim_locked();
    }

    orientation_sensor_ = find_sensor_handle("orientation");
    gyro_sensor_ = find_sensor_handle("angular-velocity");
    accel_sensor_ = find_sensor_handle("linear-acceleration");

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

    rob_stride_service_ = create_service<rs_interface::srv::RobStrideMsgs>(
        "/rob_stride_control",
        std::bind(&MujocoMotorBridgeNode::on_rob_stride_control, this, std::placeholders::_1, std::placeholders::_2));

    get_positions_service_ = create_service<rs_interface::srv::GetPositions>(
        "/get_positions",
        std::bind(&MujocoMotorBridgeNode::on_get_positions, this, std::placeholders::_1, std::placeholders::_2));

    set_zeros_service_ = create_service<rs_interface::srv::SetZeros>(
        "/set_zeros",
        std::bind(&MujocoMotorBridgeNode::on_set_zeros, this, std::placeholders::_1, std::placeholders::_2));

    if (control_frequency_ <= 1e-6)
    {
        RCLCPP_WARN(get_logger(), "control_frequency too small; defaulting to %.1f Hz.", kDefaultControlFrequency);
        control_frequency_ = kDefaultControlFrequency;
    }

    const double sim_dt = (model_ && model_->opt.timestep > 0.0) ? model_->opt.timestep : 0.001;
    const double desired_dt = 1.0 / control_frequency_;
    sim_substeps_ = std::max<size_t>(1, static_cast<size_t>(std::round(desired_dt / sim_dt)));

    const auto period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&MujocoMotorBridgeNode::control_loop, this));

    start_render_thread();

    RCLCPP_INFO(get_logger(), "MuJoCo motor bridge ready. Active joints: %zu", active_joint_count_);
}

/**
 * @brief 析构函数。停止渲染线程，释放 MuJoCo 的 model 和 data 资源。
 */
MujocoMotorBridgeNode::~MujocoMotorBridgeNode()
{
    render_running_ = false;
    if (render_thread_.joinable())
    {
        render_thread_.join();
    }

    if (data_)
    {
        mj_deleteData(data_);
        data_ = nullptr;
    }
    if (model_)
    {
        mj_deleteModel(model_);
        model_ = nullptr;
    }
}

/**
 * @brief 返回默认的 12 个腿部关节名称（左腿 6 + 右腿 6）。
 * @return 关节名称列表
 */
std::vector<std::string> MujocoMotorBridgeNode::default_joint_names()
{
    return {
        "L_hip_roll_joint",
        "L_hip_yaw_joint",
        "L_hip_pitch_joint",
        "L_knee_joint",
        "L_foot_pitch_joint",
        "L_foot_roll_joint",
        "R_hip_roll_joint",
        "R_hip_yaw_joint",
        "R_hip_pitch_joint",
        "R_knee_joint",
        "R_foot_pitch_joint",
        "R_foot_roll_joint"};
}

/**
 * @brief 加载 MJCF/XML 模型文件，创建 data 并执行一次正向动力学计算。
 * @param model_path MJCF 文件路径
 * @throw std::runtime_error 模型加载或 data 分配失败时抛出
 */
void MujocoMotorBridgeNode::load_model(const std::string &model_path)
{
    char error[1024] = {0};
    model_ = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!model_)
    {
        throw std::runtime_error(std::string("Failed to load MuJoCo model: ") + error);
    }

    data_ = mj_makeData(model_);
    if (!data_)
    {
        mj_deleteModel(model_);
        model_ = nullptr;
        throw std::runtime_error("Failed to allocate MuJoCo data");
    }

    mj_forward(model_, data_);
}

/**
 * @brief 将 ROS 关节名称映射到 MuJoCo 内部索引（qpos 地址、dof 地址、执行器 ID）。
 *        未找到的关节或执行器标记为 invalid，后续控制循环会跳过。
 */
void MujocoMotorBridgeNode::build_joint_mappings()
{
    joint_handles_.assign(kNumMotors, JointHandle{-1, -1, -1, -1, false});

    for (size_t i = 0; i < active_joint_count_; ++i)
    {
        const std::string &name = joint_names_[i];
        const int joint_id = mj_name2id(model_, mjOBJ_JOINT, name.c_str());
        if (joint_id < 0)
        {
            RCLCPP_WARN(get_logger(), "Joint '%s' not found in MuJoCo model.", name.c_str());
            continue;
        }

        const int actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, name.c_str());
        if (actuator_id < 0)
        {
            RCLCPP_WARN(get_logger(), "Actuator '%s' not found in MuJoCo model.", name.c_str());
        }

        joint_handles_[i] = JointHandle{
            joint_id,
            model_->jnt_qposadr[joint_id],
            model_->jnt_dofadr[joint_id],
            actuator_id,
            true};
    }
}

/**
 * @brief 按名称查找 MuJoCo 传感器，返回其在 sensordata 数组中的地址和维度。
 * @param name 传感器名称（如 "orientation"）
 * @return SensorHandle，valid 为 false 表示未找到
 */
MujocoMotorBridgeNode::SensorHandle MujocoMotorBridgeNode::find_sensor_handle(const std::string &name) const
{
    const int sensor_id = mj_name2id(model_, mjOBJ_SENSOR, name.c_str());
    if (sensor_id < 0)
    {
        RCLCPP_WARN(get_logger(), "Sensor '%s' not found in MuJoCo model.", name.c_str());
        return SensorHandle{-1, 0, 0, false};
    }

    return SensorHandle{
        sensor_id,
        model_->sensor_adr[sensor_id],
        model_->sensor_dim[sensor_id],
        true};
}

/**
 * @brief 从 MuJoCo data 中读取关节位置、速度、力矩到内部缓存数组。
 * @note 调用方需持有 state_mutex_ 锁。
 *       位置会减去 zero_offsets_，使反馈值相对于用户设定的零位。
 */
void MujocoMotorBridgeNode::update_state_from_sim_locked()
{
    if (!model_ || !data_)
    {
        return;
    }

    for (size_t i = 0; i < kNumMotors; ++i)
    {
        if (i < active_joint_count_ && joint_handles_[i].valid)
        {
            const JointHandle &handle = joint_handles_[i];
            last_positions_[i] = static_cast<float>(data_->qpos[handle.qpos_adr] - zero_offsets_[i]);
            last_velocities_[i] = static_cast<float>(data_->qvel[handle.dof_adr]);
            if (handle.actuator_id >= 0 && handle.actuator_id < model_->nu)
            {
                last_efforts_[i] = static_cast<float>(data_->actuator_force[handle.actuator_id]);
            }
            else
            {
                last_efforts_[i] = 0.0f;
            }
        }
        else
        {
            last_positions_[i] = 0.0f;
            last_velocities_[i] = 0.0f;
            last_efforts_[i] = 0.0f;
        }
    }

    has_state_ = true;
}

/**
 * @brief 将目标位置转换为 MuJoCo 控制输入写入 data_->ctrl。
 *        力矩模式下通过 PD 计算力矩；位置模式下直接写入目标位置。
 * @note 调用方需持有 state_mutex_ 锁。
 */
void MujocoMotorBridgeNode::apply_control_locked()
{
    if (!model_ || !data_)
    {
        return;
    }

    if (model_->nu > 0 && data_->ctrl)
    {
        std::fill(data_->ctrl, data_->ctrl + model_->nu, 0.0);
    }

    for (size_t i = 0; i < active_joint_count_; ++i)
    {
        const JointHandle &handle = joint_handles_[i];
        if (!handle.valid || handle.actuator_id < 0)
        {
            continue;
        }

        const double target_q = target_positions_[i];
        const double current_q = data_->qpos[handle.qpos_adr];
        const double current_dq = data_->qvel[handle.dof_adr];
        const double target_dq = 0.0;

        if (control_mode_ == ControlMode::kTorquePd)
        {
            const double tau = compute_tau(target_q, current_q, target_dq, current_dq, kp_[i], kd_[i]);
            data_->ctrl[handle.actuator_id] = tau;
        }
        else
        {
            data_->ctrl[handle.actuator_id] = target_q;
        }
    }
}

/**
 * @brief PD 控制律：tau = (target_q - current_q) * kp + (target_dq - current_dq) * kd。
 * @param target_q  目标位置（rad）
 * @param current_q 当前实际位置（rad）
 * @param target_dq 目标速度（通常为 0）
 * @param current_dq 当前实际速度（rad/s）
 * @param kp 比例增益
 * @param kd 微分增益
 * @return 计算得到的力矩值（Nm）
 */
double MujocoMotorBridgeNode::compute_tau(
    double target_q,
    double current_q,
    double target_dq,
    double current_dq,
    double kp,
    double kd) const
{
    return (target_q - current_q) * kp + (target_dq - current_dq) * kd;
}

/**
 * @brief 控制定时器回调。按 control_frequency_ 频率执行：
 *        加锁 → apply_control_locked → mj_step（多次）→ update_state_from_sim_locked → 发布话题。
 */
void MujocoMotorBridgeNode::control_loop()
{
    if (!model_ || !data_)
    {
        return;
    }

    std::array<float, kNumMotors> positions;
    std::array<float, kNumMotors> velocities;
    std::array<float, kNumMotors> efforts;

    SensorHandle orientation = orientation_sensor_;
    SensorHandle gyro = gyro_sensor_;
    SensorHandle accel = accel_sensor_;

    rclcpp::Time stamp;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (is_target_initialized_)
        {
            apply_control_locked();
            for (size_t step = 0; step < sim_substeps_; ++step)
            {
                mj_step(model_, data_);
            }
            update_state_from_sim_locked();
        }

        positions = last_positions_;
        velocities = last_velocities_;
        efforts = last_efforts_;
        stamp = current_sim_time_locked();
    }

    if (publish_clock_)
    {
        publish_clock(stamp);
    }
    if (publish_joint_states_)
    {
        publish_joint_states(stamp, positions, velocities, efforts);
    }
    if (publish_imu_)
    {
        publish_imu(stamp, orientation, gyro, accel);
    }
}

/**
 * @brief 处理 /rob_stride_control 服务请求。
 *        将 23 个目标位置（加零位偏移）存入 target_positions_，
 *        并返回当前关节的反馈（位置、速度、力矩、模拟温度）。
 */
void MujocoMotorBridgeNode::on_rob_stride_control(
    const std::shared_ptr<rs_interface::srv::RobStrideMsgs::Request> request,
    std::shared_ptr<rs_interface::srv::RobStrideMsgs::Response> response)
{
    std::array<float, kNumMotors> positions;
    std::array<float, kNumMotors> velocities;
    std::array<float, kNumMotors> efforts;
    bool has_state = false;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!model_ || !data_)
        {
            response->success = false;
            response->message = "MuJoCo model not initialized";
            return;
        }

        for (size_t i = 0; i < active_joint_count_; ++i)
        {
            if (!joint_handles_[i].valid)
            {
                continue;
            }
            target_positions_[i] = request->positions[i] + zero_offsets_[i];
        }
        is_target_initialized_ = true;

        update_state_from_sim_locked();
        positions = last_positions_;
        velocities = last_velocities_;
        efforts = last_efforts_;
        has_state = has_state_;
    }

    for (size_t i = 0; i < kNumMotors; ++i)
    {
        response->feedback_positions[i] = positions[i];
        response->feedback_velocities[i] = velocities[i];
        response->feedback_torques[i] = efforts[i];
        response->feedback_temperatures[i] = feedback_temperature_;
    }

    response->success = has_state;
    response->message = has_state ? "Command stored" : "No simulation state available";
}

/**
 * @brief 处理 /get_positions 服务请求。返回当前各关节的位置（已减去零位偏移）。
 */
void MujocoMotorBridgeNode::on_get_positions(
    const std::shared_ptr<rs_interface::srv::GetPositions::Request>,
    std::shared_ptr<rs_interface::srv::GetPositions::Response> response)
{
    std::array<float, kNumMotors> positions;
    bool has_state = false;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        update_state_from_sim_locked();
        positions = last_positions_;
        has_state = has_state_;
    }

    for (size_t i = 0; i < kNumMotors; ++i)
    {
        response->feedback_positions[i] = positions[i];
    }

    response->success = has_state;
    response->message = has_state ? "OK" : "No simulation state available";
}

/**
 * @brief 处理 /set_zeros 服务请求。将当前关节 qpos 值记录为零位偏移，
 *        此后反馈位置 = qpos - zero_offset，使当前姿态成为"零位"。
 */
void MujocoMotorBridgeNode::on_set_zeros(
    const std::shared_ptr<rs_interface::srv::SetZeros::Request>,
    std::shared_ptr<rs_interface::srv::SetZeros::Response> response)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!model_ || !data_)
        {
            response->success = false;
            response->message = "MuJoCo model not initialized";
            return;
        }

        for (size_t i = 0; i < active_joint_count_; ++i)
        {
            if (!joint_handles_[i].valid)
            {
                continue;
            }
            zero_offsets_[i] = static_cast<float>(data_->qpos[joint_handles_[i].qpos_adr]);
        }
        update_state_from_sim_locked();
    }

    response->success = true;
    response->message = "Zero offsets captured from current joint positions";
}

/**
 * @brief 发布 /joint_states 话题，仅包含 active_joint_count_ 个活动关节的数据。
 * @param stamp     时间戳
 * @param positions  23 个关节的位置数组
 * @param velocities 23 个关节的速度数组
 * @param efforts    23 个关节的力矩数组
 */
void MujocoMotorBridgeNode::publish_joint_states(
    const rclcpp::Time &stamp,
    const std::array<float, kNumMotors> &positions,
    const std::array<float, kNumMotors> &velocities,
    const std::array<float, kNumMotors> &efforts)
{
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = base_frame_;
    msg.name = joint_names_;
    msg.position.resize(active_joint_count_);
    msg.velocity.resize(active_joint_count_);
    msg.effort.resize(active_joint_count_);

    for (size_t i = 0; i < active_joint_count_; ++i)
    {
        msg.position[i] = positions[i];
        msg.velocity[i] = velocities[i];
        msg.effort[i] = efforts[i];
    }

    joint_state_pub_->publish(msg);
}

/**
 * @brief 发布 /imu/data 话题。从 MuJoCo sensordata 中读取四元数、角速度和线加速度。
 * @param stamp       时间戳
 * @param orientation 四元数传感器句柄
 * @param gyro        陀螺仪传感器句柄
 * @param accel       加速度计传感器句柄
 */
void MujocoMotorBridgeNode::publish_imu(
    const rclcpp::Time &stamp,
    const SensorHandle &orientation,
    const SensorHandle &gyro,
    const SensorHandle &accel)
{
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = imu_frame_;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!model_ || !data_)
        {
            return;
        }

        if (orientation.valid && orientation.dim >= 4)
        {
            const double *quat = data_->sensordata + orientation.adr;
            msg.orientation.w = quat[0];
            msg.orientation.x = quat[1];
            msg.orientation.y = quat[2];
            msg.orientation.z = quat[3];
        }
        else
        {
            msg.orientation.w = 1.0;
            msg.orientation.x = 0.0;
            msg.orientation.y = 0.0;
            msg.orientation.z = 0.0;
        }

        if (gyro.valid && gyro.dim >= 3)
        {
            const double *gyro_data = data_->sensordata + gyro.adr;
            msg.angular_velocity.x = gyro_data[0];
            msg.angular_velocity.y = gyro_data[1];
            msg.angular_velocity.z = gyro_data[2];
        }

        if (accel.valid && accel.dim >= 3)
        {
            const double *accel_data = data_->sensordata + accel.adr;
            msg.linear_acceleration.x = accel_data[0];
            msg.linear_acceleration.y = accel_data[1];
            msg.linear_acceleration.z = accel_data[2];
        }
    }

    imu_pub_->publish(msg);
}

/**
 * @brief 发布 /clock 话题，将 MuJoCo 仿真时间传递给 ROS 2 时间系统。
 * @param stamp 当前仿真时间
 */
void MujocoMotorBridgeNode::publish_clock(const rclcpp::Time &stamp)
{
    rosgraph_msgs::msg::Clock msg;
    msg.clock = stamp;
    clock_pub_->publish(msg);
}

/**
 * @brief 获取当前 MuJoCo 仿真时间。若 data 不可用则返回 ROS 系统时间。
 * @note 调用方需持有 state_mutex_ 锁。
 * @return rclcpp::Time 格式的仿真时间
 */
rclcpp::Time MujocoMotorBridgeNode::current_sim_time_locked() const
{
    if (!data_)
    {
        return this->get_clock()->now();
    }

    const int64_t nanos = static_cast<int64_t>(data_->time * 1e9);
    return rclcpp::Time(nanos, RCL_ROS_TIME);
}

/**
 * @brief 重置仿真到初始状态：mj_resetData → mj_forward，清除零位偏移，
 *        并将目标位置设为复位后的关节位置，防止复位后瞬间跳变。
 */
void MujocoMotorBridgeNode::reset_simulation()
{
    if (!model_ || !data_)
    {
        RCLCPP_WARN(get_logger(), "Cannot reset: model not loaded.");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        mj_resetData(model_, data_);
        mj_forward(model_, data_);

        zero_offsets_.fill(0.0f);
        is_target_initialized_ = false;

        for (size_t i = 0; i < kNumMotors; ++i)
        {
            if (i < active_joint_count_ && joint_handles_[i].valid)
            {
                target_positions_[i] = static_cast<float>(data_->qpos[joint_handles_[i].qpos_adr]);
            }
            else
            {
                target_positions_[i] = 0.0f;
            }
        }

        update_state_from_sim_locked();
    }

    RCLCPP_INFO(get_logger(), "Simulation reset to initial state.");
}

/**
 * @brief 在独立线程中启动 GLUT 渲染循环。
 */
void MujocoMotorBridgeNode::start_render_thread()
{
    render_running_ = true;
    render_thread_ = std::thread(&MujocoMotorBridgeNode::render_loop, this);
}

/**
 * @brief GLUT 渲染线程主循环。初始化窗口、注册回调、以 ~60 FPS 运行事件循环。
 *        窗口关闭时清理场景和上下文资源。
 */
void MujocoMotorBridgeNode::render_loop()
{
    int glut_argc = 0;
    glutInit(&glut_argc, nullptr);

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(render_width_, render_height_);
    glutCreateWindow("zrobot - MuJoCo Viewer");

    glutDisplayFunc(glut_display);
    glutReshapeFunc(glut_reshape);
    glutMouseFunc(glut_mouse);
    glutMotionFunc(glut_motion);
    glutKeyboardFunc(glut_keyboard);

    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);

    mjv_defaultCamera(&render_cam_);
    mjv_defaultFreeCamera(model_, &render_cam_);
    mjv_defaultOption(&render_opt_);
    mjv_defaultPerturb(&render_pert_);

    mjv_defaultScene(&render_scene_);
    mjv_makeScene(model_, &render_scene_, 2000);

    mjr_defaultContext(&render_context_);
    mjr_makeContext(model_, &render_context_, mjFONTSCALE_150);

    render_initialized_ = true;
    RCLCPP_INFO(get_logger(), "MuJoCo renderer initialized (GLUT).");

    while (render_running_)
    {
        glutMainLoopEvent();
        if (!render_running_)
        {
            break;
        }

        int win = glutGetWindow();
        if (win == 0)
        {
            render_running_ = false;
            break;
        }

        render_scene();

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    mjr_freeContext(&render_context_);
    mjv_freeScene(&render_scene_);

    int win = glutGetWindow();
    if (win != 0)
    {
        glutDestroyWindow(win);
    }

    render_initialized_ = false;
    RCLCPP_INFO(get_logger(), "MuJoCo renderer stopped.");
}

/**
 * @brief 更新并渲染一帧 MuJoCo 场景。在持锁状态下调用 mjv_updateScene，
 *        然后通过 mjr_render 绘制到 GLUT 窗口。
 */
void MujocoMotorBridgeNode::render_scene()
{
    if (!render_initialized_ || !model_ || !data_)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        mjv_updateScene(model_, data_, &render_opt_, &render_pert_,
                        &render_cam_, mjCAT_ALL, &render_scene_);
    }

    mjrRect viewport = {0, 0, render_width_, render_height_};
    mjr_render(viewport, &render_scene_, &render_context_);
    glutSwapBuffers();
}

/**
 * @brief GLUT 显示回调。委托给 render_instance_ 的 render_scene()。
 */
/* static */ void MujocoMotorBridgeNode::glut_display()
{
    if (render_instance_)
    {
        render_instance_->render_scene();
    }
}

/**
 * @brief GLUT 窗口大小变化回调。更新视口尺寸。
 */
/* static */ void MujocoMotorBridgeNode::glut_reshape(int width, int height)
{
    if (!render_instance_)
    {
        return;
    }
    render_instance_->render_width_ = width;
    render_instance_->render_height_ = height;
    glViewport(0, 0, width, height);
}

/**
 * @brief GLUT 鼠标按键回调。将按钮映射到 MuJoCo 相机动作（平移/缩放/旋转）。
 */
/* static */ void MujocoMotorBridgeNode::glut_mouse(int button, int state, int x, int y)
{
    if (!render_instance_)
    {
        return;
    }

    render_instance_->last_mouse_x_ = x;
    render_instance_->last_mouse_y_ = y;

    if (state == GLUT_DOWN)
    {
        if (button == GLUT_LEFT_BUTTON)
        {
            render_instance_->mouse_action_left_ = mjMOUSE_MOVE_V;
        }
        else if (button == GLUT_RIGHT_BUTTON)
        {
            render_instance_->mouse_action_left_ = mjMOUSE_ZOOM;
        }
        else if (button == GLUT_MIDDLE_BUTTON)
        {
            render_instance_->mouse_action_left_ = mjMOUSE_ROTATE_V;
        }
    }
    else
    {
        render_instance_->mouse_action_left_ = mjMOUSE_NONE;
    }
}

/**
 * @brief GLUT 鼠标拖拽回调。计算鼠标位移并传递给 MuJoCo 相机控制。
 */
/* static */ void MujocoMotorBridgeNode::glut_motion(int x, int y)
{
    if (!render_instance_ || render_instance_->mouse_action_left_ == mjMOUSE_NONE)
    {
        return;
    }

    double dx = static_cast<double>(x - render_instance_->last_mouse_x_);
    double dy = static_cast<double>(render_instance_->last_mouse_y_ - y);
    render_instance_->last_mouse_x_ = x;
    render_instance_->last_mouse_y_ = y;

    constexpr double kSensitivity = 0.001;
    dx *= kSensitivity;
    dy *= kSensitivity;

    {
        std::lock_guard<std::mutex> lock(render_instance_->state_mutex_);
        mjv_moveCamera(render_instance_->model_, render_instance_->mouse_action_left_,
                       dx, dy, &render_instance_->render_scene_, &render_instance_->render_cam_);
    }
}

/**
 * @brief GLUT 键盘回调。ESC 退出渲染，r 重置仿真，c 重置相机视角。
 */
/* static */ void MujocoMotorBridgeNode::glut_keyboard(unsigned char key, int, int)
{
    if (!render_instance_)
    {
        return;
    }

    switch (key)
    {
    case 27:
        render_instance_->render_running_ = false;
        break;
    case 'r':
    case 'R':
        render_instance_->reset_simulation();
        break;
    case 'c':
    case 'C':
        mjv_defaultFreeCamera(render_instance_->model_, &render_instance_->render_cam_);
        break;
    default:
        break;
    }
}

/**
 * @brief 将 GLUT 鼠标按钮和状态映射为 MuJoCo 相机动作类型。
 * @param button GLUT 鼠标按钮（LEFT/RIGHT/MIDDLE）
 * @param state  GLUT 状态（DOWN/UP）
 * @return mjtMouse 动作枚举（平移/缩放/旋转/无动作）
 */
mjtMouse MujocoMotorBridgeNode::map_button_to_action(int button, int state) const
{
    if (state != GLUT_DOWN)
    {
        return mjMOUSE_NONE;
    }

    switch (button)
    {
    case GLUT_LEFT_BUTTON:
        return mjMOUSE_MOVE_V;
    case GLUT_RIGHT_BUTTON:
        return mjMOUSE_ZOOM;
    case GLUT_MIDDLE_BUTTON:
        return mjMOUSE_ROTATE_V;
    default:
        return mjMOUSE_NONE;
    }
}

/**
 * @brief 程序入口。初始化 ROS 2，创建并运行 MujocoMotorBridgeNode，异常时打印错误信息。
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(std::make_shared<MujocoMotorBridgeNode>());
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("mujoco_motor_bridge_node"), "Startup failed: %s", ex.what());
    }

    rclcpp::shutdown();
    return 0;
}
