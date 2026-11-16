#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rs_interface/msg/joint_cmd.hpp"
#include "rs_interface/msg/joint_state.hpp"
#include "rs_interface/msg/motor_telemetry.hpp"
#include "rs_interface/srv/set_zeros.hpp"
#include "zrobot_bridge/motor_cfg.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

/**
 * @class MotorControllerNode
 * @brief 命令流电机控制器：100 Hz tick 内完成 23 路 CAN 写与状态收包。
 *
 * /joint_cmd 回调只更新缓存；/joint_state 与 /motor_telemetry 由 tick 发布；
 * enable/disable/set_zero 进入维护队列，维护期间暂停电机写但继续发布状态。
 */
class MotorControllerNode : public rclcpp::Node
{
public:
    MotorControllerNode();
    ~MotorControllerNode() override;

private:
    static constexpr int NUM_MOTORS = 23;

    enum class SafetyState
    {
        kDisabled,
        kActive,
        kDamping,
        kPreparing,
        kMaintenance,
    };

    enum class MaintenanceType
    {
        kEnable,
        kDisable,
        kSetZero,
    };

    struct MotorSnapshot
    {
        float position = 0.0f;
        float velocity = 0.0f;
        float torque = 0.0f;
        float temperature = 0.0f;
        uint8_t error_code = 0;
        bool ever_feedback = false;
        std::chrono::steady_clock::time_point last_feedback{};
    };

    struct MotionCommand
    {
        std::array<float, NUM_MOTORS> position{};
        std::array<float, NUM_MOTORS> velocity{};
        std::array<float, NUM_MOTORS> torque{};
        std::array<float, NUM_MOTORS> kp{};
        std::array<float, NUM_MOTORS> kd{};
    };

    struct PendingCommand
    {
        bool valid = false;
        MotionCommand command;
        std::chrono::steady_clock::time_point received_at{};
    };

    struct MaintenanceRequest
    {
        MaintenanceType type;
    };

    /* ----------- 初始化 ----------- */
    void load_parameters();
    void initialize_motors();
    void setup_ros_interfaces();

    /* ----------- 回调 ----------- */
    void handle_joint_cmd(const rs_interface::msg::JointCmd::SharedPtr msg);
    void handle_set_zeros_service(
        const std::shared_ptr<rs_interface::srv::SetZeros::Request> request,
        std::shared_ptr<rs_interface::srv::SetZeros::Response> response);
    void enqueue_maintenance(MaintenanceType type);

    /* ----------- tick ----------- */
    void control_tick();
    void drain_feedback(int timeout_ms);
    void update_snapshot_from_motor(size_t index,
                                    std::chrono::steady_clock::time_point now);

    void begin_next_maintenance();
    void process_maintenance_step();
    bool prepare_single_motor(size_t index);
    void disable_single_motor(size_t index);
    void set_zero_single_motor(size_t index);
    void finish_enable();
    void finish_disable();
    void finish_set_zero();

    /* ----------- 命令构建 ----------- */
    bool parse_mit_command(const rs_interface::msg::JointCmd& msg,
                           MotionCommand& command);
    MotionCommand build_hold_command() const;
    MotionCommand build_damping_command() const;
    bool command_timed_out(std::chrono::steady_clock::time_point now) const;

    /* ----------- 发布 ----------- */
    void publish_joint_state();
    void publish_telemetry();
    const char* safety_state_string() const;
    static const char* maintenance_name(MaintenanceType type);

    /* ----------- 参数 ----------- */
    bool live_ = true;
    uint8_t master_id_ = 0xFD;
    std::array<uint8_t, NUM_MOTORS> motor_can_ids_{};
    std::array<int, NUM_MOTORS> motor_types_{};
    std::array<std::string, NUM_MOTORS> motor_can_interfaces_{};
    std::array<float, NUM_MOTORS> motor_kps_{};
    std::array<float, NUM_MOTORS> motor_kds_{};
    std::array<std::string, NUM_MOTORS> joint_names_{};
    float damping_kd_ = 1.0f;
    double control_frequency_hz_ = 100.0;
    double telemetry_frequency_hz_ = 10.0;
    int command_timeout_ms_ = 300;
    int feedback_timeout_ms_ = 100;
    int auto_disable_after_ms_ = 0;
    std::string joint_cmd_topic_ = "/joint_cmd";
    std::string joint_state_topic_ = "/joint_state";
    std::string telemetry_topic_ = "/motor_telemetry";

    /* ----------- 电机与缓存 ----------- */
    std::array<std::shared_ptr<RobStrideMotor>, NUM_MOTORS> motors_;
    std::array<MotorSnapshot, NUM_MOTORS> snapshots_;
    mutable std::mutex motors_mutex_;

    std::mutex command_mutex_;
    PendingCommand pending_command_;
    bool damping_mode_ = false;
    bool has_command_time_ = false;
    std::chrono::steady_clock::time_point last_command_time_{};

    std::mutex maintenance_mutex_;
    std::deque<MaintenanceRequest> maintenance_queue_;
    bool maintenance_active_ = false;
    MaintenanceType active_maintenance_ = MaintenanceType::kEnable;
    size_t maintenance_index_ = 0;

    SafetyState safety_state_ = SafetyState::kDisabled;
    bool command_stale_ = false;
    uint64_t tick_count_ = 0;
    uint64_t telemetry_divider_ = 10;
    bool auto_disable_queued_ = false;

    /* ----------- ROS 接口 ----------- */
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Publisher<rs_interface::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<rs_interface::msg::MotorTelemetry>::SharedPtr telemetry_pub_;
    rclcpp::Subscription<rs_interface::msg::JointCmd>::SharedPtr joint_cmd_sub_;
    rclcpp::Service<rs_interface::srv::SetZeros>::SharedPtr set_zeros_service_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Logger logger_;
};
