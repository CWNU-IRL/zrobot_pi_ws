#include "zrobot_bridge/motor_controller.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <utility>

namespace
{
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

const char* MotorControllerNode::maintenance_name(MaintenanceType type)
{
    switch (type)
    {
    case MaintenanceType::kEnable:
        return "enable";
    case MaintenanceType::kDisable:
        return "disable";
    case MaintenanceType::kSetZero:
        return "set_zero";
    }
    return "unknown";
}

MotorControllerNode::MotorControllerNode()
    : rclcpp::Node("motor_controller_node"),
      logger_(this->get_logger())
{
    load_parameters();
    initialize_motors();

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    setup_ros_interfaces();

    RCLCPP_INFO(logger_,
                "motor_controller_node ready: live=%d control=%.1fHz telemetry=%.1fHz "
                "cmd_timeout=%dms feedback_timeout=%dms",
                static_cast<int>(live_), control_frequency_hz_, telemetry_frequency_hz_,
                command_timeout_ms_, feedback_timeout_ms_);
}

MotorControllerNode::~MotorControllerNode()
{
    if (!live_)
    {
        return;
    }

    if (safety_state_ == SafetyState::kDisabled)
    {
        return;
    }

    RCLCPP_INFO(logger_, "shutdown: sending zero target and disabling motors");
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        if (!motors_[i])
        {
            continue;
        }
        motors_[i]->send_motion_command(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
        motors_[i]->Disenable_Motor(0);
    }
}

void MotorControllerNode::load_parameters()
{
    live_ = this->declare_parameter<bool>("live", true);
    master_id_ = static_cast<uint8_t>(this->declare_parameter<int>("master_id", 0xFD));

    const auto can_ids = this->declare_parameter<std::vector<int64_t>>(
        "motor_can_ids", std::vector<int64_t>(NUM_MOTORS, 0));
    const auto types = this->declare_parameter<std::vector<int64_t>>(
        "motor_types", std::vector<int64_t>(NUM_MOTORS, 2));
    const auto interfaces = this->declare_parameter<std::vector<std::string>>(
        "motor_can_interfaces", std::vector<std::string>(NUM_MOTORS, "can0"));
    const auto kps = this->declare_parameter<std::vector<double>>(
        "motor_kps", std::vector<double>(NUM_MOTORS, 1.0));
    const auto kds = this->declare_parameter<std::vector<double>>(
        "motor_kds", std::vector<double>(NUM_MOTORS, 0.5));
    const auto names = this->declare_parameter<std::vector<std::string>>(
        "joint_names", default_joint_names());

    damping_kd_ = static_cast<float>(this->declare_parameter<double>("damping_kd", 1.0));
    control_frequency_hz_ = this->declare_parameter<double>("control_frequency_hz", 100.0);
    telemetry_frequency_hz_ = this->declare_parameter<double>("telemetry_frequency_hz", 10.0);
    command_timeout_ms_ = this->declare_parameter<int>("command_timeout_ms", 300);
    feedback_timeout_ms_ = this->declare_parameter<int>("feedback_timeout_ms", 100);
    auto_disable_after_ms_ = this->declare_parameter<int>("auto_disable_after_ms", 0);
    joint_cmd_topic_ = this->declare_parameter<std::string>("joint_cmd_topic", "/joint_cmd");
    joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/joint_state");
    telemetry_topic_ = this->declare_parameter<std::string>("telemetry_topic", "/motor_telemetry");

    if (control_frequency_hz_ <= 1.0)
    {
        RCLCPP_WARN(logger_, "control_frequency_hz=%.2f invalid, fallback to 100 Hz", control_frequency_hz_);
        control_frequency_hz_ = 100.0;
    }
    if (telemetry_frequency_hz_ <= 1.0)
    {
        RCLCPP_WARN(logger_, "telemetry_frequency_hz=%.2f invalid, fallback to 10 Hz", telemetry_frequency_hz_);
        telemetry_frequency_hz_ = 10.0;
    }
    command_timeout_ms_ = std::max(command_timeout_ms_, 1);
    feedback_timeout_ms_ = std::max(feedback_timeout_ms_, 1);
    auto_disable_after_ms_ = std::max(auto_disable_after_ms_, 0);
    telemetry_divider_ = static_cast<uint64_t>(
        std::max(1.0, std::round(control_frequency_hz_ / telemetry_frequency_hz_)));

    const auto require_size = [this](const char* name, size_t size) {
        if (size != static_cast<size_t>(NUM_MOTORS))
        {
            RCLCPP_WARN(logger_, "%s size=%zu, expected=%d; using defaults", name, size, NUM_MOTORS);
            return false;
        }
        return true;
    };

    if (require_size("motor_can_ids", can_ids.size()))
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            motor_can_ids_[i] = static_cast<uint8_t>(can_ids[static_cast<size_t>(i)]);
        }
    }
    else
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            motor_can_ids_[i] = static_cast<uint8_t>(i);
        }
    }

    if (require_size("motor_types", types.size()))
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            const auto type = static_cast<int>(types[static_cast<size_t>(i)]);
            if (type < static_cast<int>(ActuatorType::ROBSTRIDE_00) ||
                type > static_cast<int>(ActuatorType::ROBSTRIDE_06))
            {
                RCLCPP_WARN(logger_, "motor_types[%d]=%d invalid, fallback to 0", i, type);
                motor_types_[i] = 0;
            }
            else
            {
                motor_types_[i] = type;
            }
        }
    }
    else
    {
        motor_types_.fill(0);
    }

    if (require_size("motor_can_interfaces", interfaces.size()))
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            motor_can_interfaces_[i] = interfaces[static_cast<size_t>(i)];
        }
    }
    else
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            motor_can_interfaces_[i] = "can0";
        }
    }

    if (require_size("motor_kps", kps.size()))
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            motor_kps_[i] = static_cast<float>(kps[static_cast<size_t>(i)]);
        }
    }
    else
    {
        motor_kps_.fill(1.0f);
    }

    if (require_size("motor_kds", kds.size()))
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            motor_kds_[i] = static_cast<float>(kds[static_cast<size_t>(i)]);
        }
    }
    else
    {
        motor_kds_.fill(0.5f);
    }

    if (require_size("joint_names", names.size()))
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            joint_names_[i] = names[static_cast<size_t>(i)];
        }
    }
    else
    {
        const auto defaults = default_joint_names();
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            joint_names_[i] = defaults[static_cast<size_t>(i)];
        }
    }
}

void MotorControllerNode::initialize_motors()
{
    std::lock_guard<std::mutex> lock(motors_mutex_);

    if (!live_)
    {
        RCLCPP_WARN(logger_, "live=false: CAN sockets are disabled, /joint_state echoes the last command");
        return;
    }

    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        try
        {
            motors_[i] = std::make_shared<RobStrideMotor>(
                motor_can_interfaces_[i], master_id_, motor_can_ids_[i], motor_types_[i]);
            RCLCPP_DEBUG(logger_, "motor %d socket ready (%s, id=%u, type=%d)",
                         i, motor_can_interfaces_[i].c_str(), motor_can_ids_[i], motor_types_[i]);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "motor %d init failed: %s", i, e.what());
            motors_[i] = nullptr;
        }
    }
    RCLCPP_INFO(logger_, "motor sockets initialized; motors stay disabled until /joint_cmd enable");
}

void MotorControllerNode::setup_ros_interfaces()
{
    joint_state_pub_ = this->create_publisher<rs_interface::msg::JointState>(
        joint_state_topic_, rclcpp::QoS(1).best_effort());
    telemetry_pub_ = this->create_publisher<rs_interface::msg::MotorTelemetry>(
        telemetry_topic_, rclcpp::QoS(1).best_effort());

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    joint_cmd_sub_ = this->create_subscription<rs_interface::msg::JointCmd>(
        joint_cmd_topic_, rclcpp::QoS(1).reliable(),
        std::bind(&MotorControllerNode::handle_joint_cmd, this, std::placeholders::_1),
        sub_options);

    set_zeros_service_ = this->create_service<rs_interface::srv::SetZeros>(
        "/set_zeros",
        std::bind(&MotorControllerNode::handle_set_zeros_service, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_frequency_hz_));
    control_timer_ = this->create_wall_timer(
        period, std::bind(&MotorControllerNode::control_tick, this), callback_group_);

    RCLCPP_INFO(logger_, "topics: %s -> %s, %s; service /set_zeros",
                joint_cmd_topic_.c_str(), joint_state_topic_.c_str(),
                telemetry_topic_.c_str());
}

void MotorControllerNode::handle_joint_cmd(const rs_interface::msg::JointCmd::SharedPtr msg)
{
    const std::string& command = msg->command;

    if (command == "mit")
    {
        MotionCommand parsed;
        if (!parse_mit_command(*msg, parsed))
        {
            return;
        }

        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_.command = parsed;
        pending_command_.valid = true;
        pending_command_.received_at = std::chrono::steady_clock::now();
        last_command_time_ = pending_command_.received_at;
        has_command_time_ = true;
        damping_mode_ = false;
        auto_disable_queued_ = false;

        if (safety_state_ == SafetyState::kDamping)
        {
            safety_state_ = SafetyState::kActive;
        }
        else if (safety_state_ != SafetyState::kActive)
        {
            RCLCPP_WARN_THROTTLE(
                logger_, *this->get_clock(), 2000,
                "mit cached but motors are not enabled (state=%s); send enable first",
                safety_state_string());
        }
        return;
    }

    if (command == "enable")
    {
        if (maintenance_active_ || safety_state_ == SafetyState::kPreparing ||
            safety_state_ == SafetyState::kMaintenance)
        {
            RCLCPP_INFO(logger_, "enable ignored: maintenance in progress");
            return;
        }

        if (safety_state_ == SafetyState::kDisabled)
        {
            enqueue_maintenance(MaintenanceType::kEnable);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            pending_command_.valid = false;
            damping_mode_ = false;
            has_command_time_ = true;
            last_command_time_ = std::chrono::steady_clock::now();
            auto_disable_queued_ = false;
        }
        safety_state_ = SafetyState::kActive;
        RCLCPP_INFO(logger_, "enable: motors already ready, starting soft hold");
        return;
    }

    if (command == "disable" || command == "stop")
    {
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            pending_command_.valid = false;
            damping_mode_ = false;
            has_command_time_ = false;
        }

        bool maintenance_pending = false;
        {
            std::lock_guard<std::mutex> lock(maintenance_mutex_);
            maintenance_pending = maintenance_active_ || !maintenance_queue_.empty();
        }
        if (safety_state_ == SafetyState::kDisabled && !maintenance_pending)
        {
            RCLCPP_INFO(logger_, "%s ignored: motors already disabled", command.c_str());
            return;
        }
        enqueue_maintenance(MaintenanceType::kDisable);
        return;
    }

    if (command == "damp")
    {
        if (safety_state_ != SafetyState::kActive && safety_state_ != SafetyState::kDamping)
        {
            RCLCPP_WARN_THROTTLE(
                logger_, *this->get_clock(), 2000,
                "damp ignored: motors are not enabled (state=%s)", safety_state_string());
            return;
        }

        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_.valid = false;
        damping_mode_ = true;
        has_command_time_ = true;
        last_command_time_ = std::chrono::steady_clock::now();
        safety_state_ = SafetyState::kDamping;
        return;
    }

    if (command == "set_zero")
    {
        enqueue_maintenance(MaintenanceType::kSetZero);
        return;
    }

    if (command == "status")
    {
        RCLCPP_INFO(logger_,
                    "status: safety_state=%s live=%d pending_mit=%d damping=%d stale=%d "
                    "tick=%lu",
                    safety_state_string(), static_cast<int>(live_),
                    static_cast<int>(pending_command_.valid), static_cast<int>(damping_mode_),
                    static_cast<int>(command_stale_), static_cast<unsigned long>(tick_count_));
        return;
    }

    RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                         "unknown /joint_cmd command: '%s'", command.c_str());
}

void MotorControllerNode::handle_set_zeros_service(
    const std::shared_ptr<rs_interface::srv::SetZeros::Request>,
    std::shared_ptr<rs_interface::srv::SetZeros::Response> response)
{
    if (!live_)
    {
        response->success = false;
        response->message = "live=false: set_zeros is unavailable";
        return;
    }

    enqueue_maintenance(MaintenanceType::kSetZero);
    response->success = true;
    response->message = "set_zeros queued; calibration runs in the maintenance slot";
}

void MotorControllerNode::enqueue_maintenance(MaintenanceType type)
{
    std::lock_guard<std::mutex> lock(maintenance_mutex_);
    maintenance_queue_.push_back(MaintenanceRequest{type});
    RCLCPP_INFO(logger_, "maintenance queued: %s (queue=%zu)",
                maintenance_name(type), maintenance_queue_.size());
}

void MotorControllerNode::control_tick()
{
    const auto now = std::chrono::steady_clock::now();

    // 先排空上一周期迟到的反馈，再开始本周期。
    drain_feedback(0);

    if (!maintenance_active_)
    {
        begin_next_maintenance();
    }

    if (maintenance_active_)
    {
        process_maintenance_step();
    }
    else if (safety_state_ != SafetyState::kDisabled)
    {
        command_stale_ = command_timed_out(now);

        if (damping_mode_ || command_stale_)
        {
            safety_state_ = SafetyState::kDamping;
        }
        else
        {
            safety_state_ = SafetyState::kActive;
        }

        if (auto_disable_after_ms_ > 0 && !damping_mode_ &&
            has_command_time_ && !auto_disable_queued_ &&
            (now - last_command_time_) >= std::chrono::milliseconds(auto_disable_after_ms_))
        {
            auto_disable_queued_ = true;
            enqueue_maintenance(MaintenanceType::kDisable);
            RCLCPP_WARN(logger_, "auto_disable_after_ms=%d reached, disabling motors",
                        auto_disable_after_ms_);
        }

        MotionCommand command;
        if (safety_state_ == SafetyState::kDamping)
        {
            command = build_damping_command();
        }
        else
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command = pending_command_.valid ? pending_command_.command : build_hold_command();
        }

        if (live_)
        {
            for (int i = 0; i < NUM_MOTORS; ++i)
            {
                if (!motors_[i])
                {
                    continue;
                }
                motors_[i]->send_motion_command(
                    command.torque[static_cast<size_t>(i)],
                    command.position[static_cast<size_t>(i)],
                    command.velocity[static_cast<size_t>(i)],
                    command.kp[static_cast<size_t>(i)],
                    command.kd[static_cast<size_t>(i)],
                    false);
            }
        }
        else
        {
            // dry-run: 用命令值回显状态，方便在笔记本上联调 deploy。
            for (int i = 0; i < NUM_MOTORS; ++i)
            {
                auto& snapshot = snapshots_[static_cast<size_t>(i)];
                snapshot.position = command.position[static_cast<size_t>(i)];
                snapshot.velocity = command.velocity[static_cast<size_t>(i)];
                snapshot.torque = command.torque[static_cast<size_t>(i)];
                snapshot.temperature = 25.0f;
                snapshot.error_code = 0;
                snapshot.ever_feedback = true;
                snapshot.last_feedback = now;
            }
        }
    }

    publish_joint_state();
    if (++tick_count_ % telemetry_divider_ == 0)
    {
        publish_telemetry();
    }
}

void MotorControllerNode::drain_feedback(int timeout_ms)
{
    if (!live_)
    {
        return;
    }

    std::array<struct pollfd, NUM_MOTORS> fds{};
    std::array<int, NUM_MOTORS> motor_indices{};
    size_t count = 0;

    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        if (!motors_[i])
        {
            continue;
        }
        fds[count].fd = motors_[i]->fd();
        fds[count].events = POLLIN;
        motor_indices[count] = i;
        ++count;
    }

    if (count == 0)
    {
        return;
    }

    const int rc = ::poll(fds.data(), count, timeout_ms);
    if (rc <= 0)
    {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    for (size_t k = 0; k < count; ++k)
    {
        if ((fds[k].revents & POLLIN) == 0)
        {
            continue;
        }
        const int motor_index = motor_indices[k];
        if (motors_[motor_index]->poll_status(0))
        {
            update_snapshot_from_motor(static_cast<size_t>(motor_index), now);
        }
    }
}

void MotorControllerNode::update_snapshot_from_motor(
    size_t index, std::chrono::steady_clock::time_point now)
{
    auto& motor = *motors_[index];
    auto& snapshot = snapshots_[index];
    snapshot.position = motor.position_;
    snapshot.velocity = motor.velocity_;
    snapshot.torque = motor.torque_;
    snapshot.temperature = motor.temperature_;
    snapshot.error_code = motor.error_code;
    snapshot.ever_feedback = true;
    snapshot.last_feedback = now;
}

void MotorControllerNode::begin_next_maintenance()
{
    MaintenanceType type;
    {
        std::lock_guard<std::mutex> lock(maintenance_mutex_);
        if (maintenance_queue_.empty())
        {
            return;
        }
        type = maintenance_queue_.front().type;
        maintenance_queue_.pop_front();
    }

    maintenance_active_ = true;
    active_maintenance_ = type;
    maintenance_index_ = 0;

    if (type == MaintenanceType::kEnable)
    {
        safety_state_ = SafetyState::kPreparing;
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_.valid = false;
        damping_mode_ = false;
        has_command_time_ = false;
        auto_disable_queued_ = false;
    }
    else
    {
        safety_state_ = SafetyState::kMaintenance;
    }

    RCLCPP_INFO(logger_, "maintenance start: %s", maintenance_name(type));
}

void MotorControllerNode::process_maintenance_step()
{
    if (!live_)
    {
        if (active_maintenance_ == MaintenanceType::kEnable)
        {
            finish_enable();
        }
        else if (active_maintenance_ == MaintenanceType::kDisable)
        {
            finish_disable();
        }
        else
        {
            finish_set_zero();
        }
        return;
    }

    switch (active_maintenance_)
    {
    case MaintenanceType::kEnable:
        if (maintenance_index_ >= static_cast<size_t>(NUM_MOTORS))
        {
            finish_enable();
        }
        else
        {
            prepare_single_motor(maintenance_index_++);
        }
        break;

    case MaintenanceType::kDisable:
        if (maintenance_index_ >= static_cast<size_t>(NUM_MOTORS))
        {
            finish_disable();
        }
        else
        {
            disable_single_motor(maintenance_index_++);
        }
        break;

    case MaintenanceType::kSetZero:
        if (maintenance_index_ >= static_cast<size_t>(NUM_MOTORS))
        {
            finish_set_zero();
        }
        else
        {
            set_zero_single_motor(maintenance_index_++);
        }
        break;
    }
}

bool MotorControllerNode::prepare_single_motor(size_t index)
{
    if (!motors_[index])
    {
        return false;
    }

    const uint64_t feedback_before = motors_[index]->feedback_count();

    // 逐个使能：先取一次反馈/pattern，再触发运控模式准备并软保持，最后确保使能。
    motors_[index]->enable_motor();
    const float hold_position = motors_[index]->position_;
    motors_[index]->send_motion_command(
        0.0f, hold_position, 0.0f, 0.0f, damping_kd_, true);
    motors_[index]->enable_motor();

    if (motors_[index]->feedback_count() != feedback_before)
    {
        update_snapshot_from_motor(index, std::chrono::steady_clock::now());
    }

    RCLCPP_DEBUG(logger_, "motor %zu prepared, hold=%.3f rad", index, hold_position);
    return true;
}

void MotorControllerNode::disable_single_motor(size_t index)
{
    if (!motors_[index])
    {
        return;
    }

    const uint64_t feedback_before = motors_[index]->feedback_count();
    motors_[index]->send_motion_command(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
    motors_[index]->Disenable_Motor(0);

    if (motors_[index]->feedback_count() != feedback_before)
    {
        update_snapshot_from_motor(index, std::chrono::steady_clock::now());
    }
}

void MotorControllerNode::set_zero_single_motor(size_t index)
{
    if (!motors_[index])
    {
        return;
    }

    const uint64_t feedback_before = motors_[index]->feedback_count();
    motors_[index]->Set_ZeroPos();

    if (motors_[index]->feedback_count() != feedback_before)
    {
        update_snapshot_from_motor(index, std::chrono::steady_clock::now());
    }
}

void MotorControllerNode::finish_enable()
{
    maintenance_active_ = false;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_.valid = false;
        damping_mode_ = false;
        has_command_time_ = true;
        last_command_time_ = std::chrono::steady_clock::now();
        auto_disable_queued_ = false;
    }
    safety_state_ = SafetyState::kActive;
    command_stale_ = false;
    RCLCPP_INFO(logger_, "motors enabled; waiting for /joint_cmd mit commands");
}

void MotorControllerNode::finish_disable()
{
    maintenance_active_ = false;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_.valid = false;
        damping_mode_ = false;
        has_command_time_ = false;
    }
    safety_state_ = SafetyState::kDisabled;
    command_stale_ = false;
    RCLCPP_INFO(logger_, "motors disabled");
}

void MotorControllerNode::finish_set_zero()
{
    RCLCPP_INFO(logger_, "zero calibration completed; re-preparing motors");

    maintenance_active_ = true;
    active_maintenance_ = MaintenanceType::kEnable;
    maintenance_index_ = 0;
    safety_state_ = SafetyState::kPreparing;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_.valid = false;
        damping_mode_ = false;
        has_command_time_ = false;
        auto_disable_queued_ = false;
    }
}

bool MotorControllerNode::parse_mit_command(
    const rs_interface::msg::JointCmd& msg, MotionCommand& command)
{
    if (msg.position.size() != static_cast<size_t>(NUM_MOTORS))
    {
        RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                             "mit rejected: position size=%zu, expected=%d",
                             msg.position.size(), NUM_MOTORS);
        return false;
    }

    std::array<size_t, NUM_MOTORS> target_index{};
    if (msg.name.empty())
    {
        for (size_t i = 0; i < target_index.size(); ++i)
        {
            target_index[i] = i;
        }
    }
    else
    {
        if (msg.name.size() != static_cast<size_t>(NUM_MOTORS))
        {
            RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                                 "mit rejected: name size=%zu, expected=%d",
                                 msg.name.size(), NUM_MOTORS);
            return false;
        }
        for (size_t i = 0; i < target_index.size(); ++i)
        {
            const auto it = std::find(joint_names_.begin(), joint_names_.end(), msg.name[i]);
            if (it == joint_names_.end())
            {
                RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                                     "mit rejected: unknown joint name '%s'", msg.name[i].c_str());
                return false;
            }
            target_index[i] = static_cast<size_t>(std::distance(joint_names_.begin(), it));
        }
    }

    const auto copy_field = [&target_index](
                                const std::vector<double>& source,
                                std::array<float, NUM_MOTORS>& destination,
                                float fill_value) {
        if (source.empty())
        {
            destination.fill(fill_value);
            return true;
        }
        if (source.size() != static_cast<size_t>(NUM_MOTORS))
        {
            return false;
        }
        for (size_t i = 0; i < target_index.size(); ++i)
        {
            destination[target_index[i]] = static_cast<float>(source[i]);
        }
        return true;
    };

    command = MotionCommand{};
    if (!copy_field(msg.position, command.position, 0.0f) ||
        !copy_field(msg.velocity, command.velocity, 0.0f) ||
        !copy_field(msg.torque, command.torque, 0.0f))
    {
        RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                             "mit rejected: position/velocity/torque must be empty or %d", NUM_MOTORS);
        return false;
    }

    command.kp = motor_kps_;
    command.kd = motor_kds_;
    if (!msg.kp.empty())
    {
        if (msg.kp.size() != static_cast<size_t>(NUM_MOTORS))
        {
            RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                                 "mit rejected: kp size=%zu, expected=%d", msg.kp.size(), NUM_MOTORS);
            return false;
        }
        for (size_t i = 0; i < target_index.size(); ++i)
        {
            command.kp[target_index[i]] = static_cast<float>(msg.kp[i]);
        }
    }
    if (!msg.kd.empty())
    {
        if (msg.kd.size() != static_cast<size_t>(NUM_MOTORS))
        {
            RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                                 "mit rejected: kd size=%zu, expected=%d", msg.kd.size(), NUM_MOTORS);
            return false;
        }
        for (size_t i = 0; i < target_index.size(); ++i)
        {
            command.kd[target_index[i]] = static_cast<float>(msg.kd[i]);
        }
    }

    for (size_t i = 0; i < static_cast<size_t>(NUM_MOTORS); ++i)
    {
        if (!std::isfinite(command.position[i]) || !std::isfinite(command.velocity[i]) ||
            !std::isfinite(command.torque[i]) || !std::isfinite(command.kp[i]) ||
            !std::isfinite(command.kd[i]))
        {
            RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                                 "mit rejected: non-finite value at motor %zu", i);
            return false;
        }

        const auto& limits = ACTUATOR_OPERATION_MAPPING.at(
            static_cast<ActuatorType>(motor_types_[i]));
        command.position[i] = std::clamp<float>(
            command.position[i],
            static_cast<float>(-limits.position),
            static_cast<float>(limits.position));
        command.velocity[i] = std::clamp<float>(
            command.velocity[i],
            static_cast<float>(-limits.velocity),
            static_cast<float>(limits.velocity));
        command.torque[i] = std::clamp<float>(
            command.torque[i],
            static_cast<float>(-limits.torque),
            static_cast<float>(limits.torque));
        command.kp[i] = std::clamp<float>(
            command.kp[i], 0.0f, static_cast<float>(limits.kp));
        command.kd[i] = std::clamp<float>(
            command.kd[i], 0.0f, static_cast<float>(limits.kd));
    }

    return true;
}

MotorControllerNode::MotionCommand MotorControllerNode::build_hold_command() const
{
    MotionCommand command;
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        const auto& snapshot = snapshots_[static_cast<size_t>(i)];
        command.position[static_cast<size_t>(i)] =
            snapshot.ever_feedback ? snapshot.position : 0.0f;
        command.kd[static_cast<size_t>(i)] = damping_kd_;
    }
    return command;
}

MotorControllerNode::MotionCommand MotorControllerNode::build_damping_command() const
{
    MotionCommand command;
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        const auto& snapshot = snapshots_[static_cast<size_t>(i)];
        command.position[static_cast<size_t>(i)] =
            snapshot.ever_feedback ? snapshot.position : 0.0f;
        command.kd[static_cast<size_t>(i)] = damping_kd_;
    }
    return command;
}

bool MotorControllerNode::command_timed_out(std::chrono::steady_clock::time_point now) const
{
    if (damping_mode_ || !has_command_time_)
    {
        return false;
    }
    return (now - last_command_time_) > std::chrono::milliseconds(command_timeout_ms_);
}

void MotorControllerNode::publish_joint_state()
{
    rs_interface::msg::JointState msg;
    msg.stamp = this->now();
    msg.name.assign(joint_names_.begin(), joint_names_.end());
    msg.position.resize(NUM_MOTORS);
    msg.velocity.resize(NUM_MOTORS);
    msg.torque.resize(NUM_MOTORS);
    msg.temperature.resize(NUM_MOTORS);
    msg.valid.resize(NUM_MOTORS);

    const auto now = std::chrono::steady_clock::now();
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        const auto& snapshot = snapshots_[static_cast<size_t>(i)];
        const bool valid = !live_
                               ? true
                               : (snapshot.ever_feedback &&
                                  (now - snapshot.last_feedback) <=
                                      std::chrono::milliseconds(feedback_timeout_ms_));
        msg.position[static_cast<size_t>(i)] = snapshot.position;
        msg.velocity[static_cast<size_t>(i)] = snapshot.velocity;
        msg.torque[static_cast<size_t>(i)] = snapshot.torque;
        msg.temperature[static_cast<size_t>(i)] = snapshot.temperature;
        msg.valid[static_cast<size_t>(i)] = valid;
    }

    joint_state_pub_->publish(msg);
}

void MotorControllerNode::publish_telemetry()
{
    rs_interface::msg::MotorTelemetry msg;
    msg.stamp = this->now();
    msg.name.assign(joint_names_.begin(), joint_names_.end());
    msg.error_code.resize(NUM_MOTORS);
    msg.feedback_age_ms.resize(NUM_MOTORS);
    msg.online.resize(NUM_MOTORS);

    const auto now = std::chrono::steady_clock::now();
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        const auto& snapshot = snapshots_[static_cast<size_t>(i)];
        const bool has_feedback = snapshot.ever_feedback;
        const double age_ms = has_feedback
                                  ? std::chrono::duration<double, std::milli>(
                                        now - snapshot.last_feedback).count()
                                  : -1.0;
        const bool online = !live_
                                ? true
                                : (has_feedback &&
                                   age_ms <= static_cast<double>(feedback_timeout_ms_));
        msg.error_code[static_cast<size_t>(i)] = snapshot.error_code;
        msg.feedback_age_ms[static_cast<size_t>(i)] = live_ ? age_ms : 0.0;
        msg.online[static_cast<size_t>(i)] = online;
    }

    msg.command_stale = command_stale_;
    msg.safety_state = safety_state_string();
    telemetry_pub_->publish(msg);
}

const char* MotorControllerNode::safety_state_string() const
{
    switch (safety_state_)
    {
    case SafetyState::kDisabled:
        return "disabled";
    case SafetyState::kActive:
        return "active";
    case SafetyState::kDamping:
        return "damping";
    case SafetyState::kPreparing:
        return "preparing";
    case SafetyState::kMaintenance:
        return "maintenance";
    }
    return "unknown";
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
