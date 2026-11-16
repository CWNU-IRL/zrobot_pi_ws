#ifndef ZROBOT_DEPLOY_DAMPING_H
#define ZROBOT_DEPLOY_DAMPING_H

#include "zrobot_deploy/FSM.h"

#include <array>

// Damping 状态机
// 电机原生阻尼模式：向电机下发 kp≈0、kd>0 的运控指令，
// 由电机内部速度环直接产生 τ = -kd·dq（无外环延迟，不会振荡）；
// 位置指令保持当前值（不再用外环 q_cmd = q_fb - k_d * dq 模拟阻尼）
class Damping : public FSM
{
public:
    explicit Damping(std::shared_ptr<rclcpp::Node> node);
    ~Damping() override;

    void initialize() override;
    void run() override;
    void exit() override;
    bool isReady() const override { return is_initialized_; }

private:
    void loadParameters();

    bool is_initialized_;
    bool feedback_seeded_;

    // 电机内部阻尼增益（随每次指令帧下发）
    float motor_kp_;   // 默认 0：无位置刚度，纯阻尼
    float motor_kd_;   // 默认 2.0 Nm·s/rad

    std::array<float, 23> command_positions_;
};

#endif // ZROBOT_DEPLOY_DAMPING_H
