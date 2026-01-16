#ifndef ZROBOT_DEPLOY_FIXSTAND_H
#define ZROBOT_DEPLOY_FIXSTAND_H

#include "zrobot_deploy/FSM.h"
#include <chrono>

// FixStand
class FixStand : public FSM
{
public:
    FixStand(std::shared_ptr<rclcpp::Node> node);
    ~FixStand() override;
    
    void initialize() override;
    void run() override;
    void exit() override;
    
private:
    // 内部状态
    enum class InternalState {
        INIT,           // 初始化阶段
        MOVING,         // 移动到零位阶段
        STANDING        // 站立保持阶段
    };
    
    InternalState internal_state_;
    
    // 目标位置（机械零位，所有电机都是0）
    std::array<float, 23> target_positions_;
    
    // 初始位置
    std::array<float, 23> initial_positions_;
    
    // 插值参数
    double interpolation_time_;     // 插值总时间（秒）
    double elapsed_time_;            // 已经过时间（秒）
    
    // 时间记录
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_time_;
    
    // 是否已初始化
    bool is_initialized_;
    
    // 线性插值函数
    float lerp(float start, float end, float t);
};

#endif //ZROBOT_DEPLOY_FIXSTAND_H
