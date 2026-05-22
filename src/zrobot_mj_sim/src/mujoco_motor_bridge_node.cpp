#include "zrobot_mj_sim/mujoco_motor_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <cmath>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "mujoco/mujoco.h"

namespace
{
constexpr double kDefaultControlFrequency = 200.0;
constexpr double kDefaultKp = 40.0;
constexpr double kDefaultKd = 1.0;
}

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

    RCLCPP_INFO(get_logger(), "MuJoCo motor bridge ready. Active joints: %zu", active_joint_count_);
}

MujocoMotorBridgeNode::~MujocoMotorBridgeNode()
{
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

            if (!is_target_initialized_)
            {
                target_positions_[i] = static_cast<float>(data_->qpos[handle.qpos_adr]);
            }
        }
        else
        {
            last_positions_[i] = 0.0f;
            last_velocities_[i] = 0.0f;
            last_efforts_[i] = 0.0f;
            if (!is_target_initialized_)
            {
                target_positions_[i] = 0.0f;
            }
        }
    }

    has_state_ = true;
    if (!is_target_initialized_)
    {
        is_target_initialized_ = true;
    }
}

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
        apply_control_locked();
        for (size_t step = 0; step < sim_substeps_; ++step)
        {
            mj_step(model_, data_);
        }
        update_state_from_sim_locked();

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

void MujocoMotorBridgeNode::publish_clock(const rclcpp::Time &stamp)
{
    rosgraph_msgs::msg::Clock msg;
    msg.clock = stamp;
    clock_pub_->publish(msg);
}

rclcpp::Time MujocoMotorBridgeNode::current_sim_time_locked() const
{
    if (!data_)
    {
        return this->get_clock()->now();
    }

    const int64_t nanos = static_cast<int64_t>(data_->time * 1e9);
    return rclcpp::Time(nanos, RCL_ROS_TIME);
}

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
