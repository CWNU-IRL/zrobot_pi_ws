#ifndef ZROBOT_DEPLOY_FSM_H
#define ZROBOT_DEPLOY_FSM_H

#include <rclcpp/rclcpp.hpp>
#include <rs_interface/msg/joint_cmd.hpp>
#include <rs_interface/msg/joint_state.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

// 状态机状态枚举
enum class FSMState {
    IDLE,           // 空闲状态
    FIX_STAND,      // 固定站立状态
    LOCOMOTION,     // 运动状态
    PT_LOCOMOTION,  // TorchScript(.pt)运动状态
    DAMPING,        // 阻尼状态
};

// FSM基类：控制意图通过 /joint_cmd 发布，反馈从 /joint_state 缓存读取。
class FSM : public std::enable_shared_from_this<FSM>
{
public:
    FSM(std::shared_ptr<rclcpp::Node> node);
    virtual ~FSM();

    virtual void initialize();
    virtual void run() = 0;
    FSMState getState() const { return current_state_; }
    virtual void exit();

    // 退出程序时由 main 调用：发布一次 damp 命令。
    void sendDampCommand();

protected:
    // 发布 23 路 MIT 位置命令；kp/kd 留空，由 bridge 使用 motor_config.yaml 增益。
    bool sendMotorPositions(const std::array<float, 23>& positions);

    // 读取最近一帧 /joint_state 缓存。
    bool getMotorFeedback(std::array<float, 23>& positions,
                          std::array<float, 23>& velocities,
                          std::array<float, 23>& torques,
                          std::array<float, 23>& temperatures);

    bool getCurrentPositions(std::array<float, 23>& positions);

    // 状态是否新鲜（默认使用 state_timeout_ms 参数）。
    bool stateFresh(double timeout_ms = -1.0) const;

    // 12 个主动关节是否全部 valid。
    bool activeJointsValid() const;

    // 等待首帧有效状态；期间周期性重发 enable，避免 bridge 后启动时丢命令。
    bool waitForFirstState(double timeout_sec);

    void publishCommand(const std::string& command);

    // ROS2节点
    std::shared_ptr<rclcpp::Node> node_;

    // 命令发布与状态订阅
    rclcpp::Publisher<rs_interface::msg::JointCmd>::SharedPtr joint_cmd_pub_;
    rclcpp::Subscription<rs_interface::msg::JointState>::SharedPtr joint_state_sub_;

    // 权威关节顺序（23）
    std::array<std::string, 23> joint_names_;
    int active_joint_count_;
    std::string joint_state_topic_;

    // 状态新鲜度参数
    double state_timeout_ms_;

    // 当前状态
    FSMState current_state_;

    // 控制频率（Hz）
    double control_frequency_;

    // 当前电机位置
    std::array<float, 23> current_motor_positions_;

    // 电机反馈缓存
    std::array<float, 23> feedback_positions_;
    std::array<float, 23> feedback_velocities_;
    std::array<float, 23> feedback_torques_;
    std::array<float, 23> feedback_temperatures_;
    std::array<bool, 23> feedback_valid_;
    std::atomic<bool> state_received_;
    std::chrono::steady_clock::time_point last_state_time_;

    mutable std::mutex feedback_mutex_;

private:
    void jointStateCallback(const rs_interface::msg::JointState::SharedPtr msg);
};

#endif //ZROBOT_DEPLOY_FSM_H
