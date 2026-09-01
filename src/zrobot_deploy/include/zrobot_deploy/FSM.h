#ifndef ZROBOT_DEPLOY_FSM_H
#define ZROBOT_DEPLOY_FSM_H

#include <rclcpp/rclcpp.hpp>
#include <rs_interface/srv/rob_stride_msgs.hpp>
#include <rs_interface/srv/get_positions.hpp>
#include <memory>
#include <mutex>
#include <atomic>
#include <vector>
#include <array>

// 状态机状态枚举
enum class FSMState {
    IDLE,           // 空闲状态（同时用作"无切换请求"哨兵值）
    FIX_STAND,      // 固定站立状态
    LOCOMOTION,     // 运动状态
    PT_LOCOMOTION,  // TorchScript(.pt)运动状态
    DAMPING,        // 阻尼状态（仿真近似）
};

// FSM基类
class FSM : public std::enable_shared_from_this<FSM>
{
public:
    FSM(std::shared_ptr<rclcpp::Node> node);
    virtual ~FSM();
    
    virtual void initialize();
    virtual void run() = 0;
    FSMState getState() const { return current_state_; }
    virtual void exit();

    // 初始化是否成功（默认成功；FixStand 等可能因读取位置失败而拒绝启动）
    virtual bool isReady() const { return true; }

    // 状态切换请求（供 FSM 内部触发，如安全保护：IMU 超时 → DAMPING）
    // 一次性语义：IDLE 表示无请求；takeSwitchRequest 取出后自动清除
    void requestSwitch(FSMState target);
    bool takeSwitchRequest(FSMState& target);
    
protected:
    // 向底层发送电机位置控制指令
    bool sendMotorPositions(const std::array<float, 23>& positions);
    
    // 获取电机反馈
    bool getMotorFeedback(std::array<float, 23>& positions,
                         std::array<float, 23>& velocities,
                         std::array<float, 23>& torques,
                         std::array<float, 23>& temperatures);
    
    // 获取当前电机位置
    bool getCurrentPositions(std::array<float, 23>& positions);
    
    // ROS2节点
    std::shared_ptr<rclcpp::Node> node_;
    
    // 服务客户端
    rclcpp::Client<rs_interface::srv::RobStrideMsgs>::SharedPtr control_client_;
    rclcpp::Client<rs_interface::srv::GetPositions>::SharedPtr get_positions_client_;
    
    // 当前状态
    FSMState current_state_;
    
    // 状态切换请求（IDLE = 无请求）
    std::atomic<FSMState> switch_target_{FSMState::IDLE};
    
    // 控制频率（Hz）
    double control_frequency_;
    
    // 当前电机位置
    std::array<float, 23> current_motor_positions_;
    
    // 电机反馈缓存（static 跨 FSM 实例共享：状态机切换时新实例可继承最近一次
    // 成功反馈，避免 Damping 在位置读取失败时无有效缓存可用）
    static std::array<float, 23> feedback_positions_;
    static std::array<float, 23> feedback_velocities_;
    static std::array<float, 23> feedback_torques_;
    static std::array<float, 23> feedback_temperatures_;

    // 是否曾经成功接收过电机反馈（Damping 用它判断缓存反馈能否作为阻尼基准）
    static std::atomic<bool> feedback_valid_;

    static std::mutex feedback_mutex_;
};

#endif //ZROBOT_DEPLOY_FSM_H