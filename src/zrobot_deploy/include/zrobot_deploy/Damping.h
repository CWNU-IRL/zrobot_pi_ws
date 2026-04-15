#ifndef ZROBOT_DEPLOY_DAMPING_H
#define ZROBOT_DEPLOY_DAMPING_H

#include "zrobot_deploy/FSM.h"

#include <array>

// Damping 状态机
// 通过位置命令近似阻尼：q_cmd = q_fb - k_d * dq_fb
class Damping : public FSM
{
public:
    explicit Damping(std::shared_ptr<rclcpp::Node> node);
    ~Damping() override;

    void initialize() override;
    void run() override;
    void exit() override;

private:
    void loadParameters();

    bool is_initialized_;
    bool feedback_seeded_;

    float kd_default_;
    float velocity_deadband_;
    float max_position_delta_;

    std::array<float, 23> damping_gains_;
    std::array<float, 23> command_positions_;
};

#endif // ZROBOT_DEPLOY_DAMPING_H
