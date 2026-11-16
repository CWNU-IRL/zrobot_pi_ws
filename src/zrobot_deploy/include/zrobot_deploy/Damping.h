#ifndef ZROBOT_DEPLOY_DAMPING_H
#define ZROBOT_DEPLOY_DAMPING_H

#include "zrobot_deploy/FSM.h"

// Damping 状态机：向 bridge 发布 damp 命令，由驱动使用自身反馈做阻尼。
class Damping : public FSM
{
public:
    explicit Damping(std::shared_ptr<rclcpp::Node> node);
    ~Damping() override;

    void initialize() override;
    void run() override;
    void exit() override;

private:
    bool is_initialized_;
};

#endif // ZROBOT_DEPLOY_DAMPING_H
