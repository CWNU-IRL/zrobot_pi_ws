#ifndef ZROBOT_DEPLOY_PT_LOCOMOTION_H
#define ZROBOT_DEPLOY_PT_LOCOMOTION_H

#include "zrobot_deploy/FSM.h"
#include "zrobot_deploy/paralleljoint_z1.h"
#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <thread>
#include <torch/script.h>
#include <vector>

// PTLocomotion 状态机 - 使用 TorchScript(.pt) 模型进行强化学习推理控制
class PTLocomotion : public FSM
{
public:
    PTLocomotion(std::shared_ptr<rclcpp::Node> node);
    ~PTLocomotion() override;

    void initialize() override;
    void run() override;
    void exit() override;
    bool isReady() const override { return model_ready_; }

private:
    static constexpr int NUM_SINGLE_OBS = 47;
    static constexpr int NUM_ACTIONS = 12;
    static constexpr int NUM_MOTORS = 23;

    Eigen::VectorXf obs_current_;
    std::deque<Eigen::VectorXf> obs_history_;
    Eigen::VectorXf policy_input_;

    Eigen::VectorXf act_prev_;
    Eigen::VectorXf act_scaled_;
    Eigen::VectorXf act_temp_;
    Eigen::VectorXf default_pose_;
    std::array<int, NUM_ACTIONS> dof_indices_;
    std::array<float, NUM_ACTIONS> joint_signs_; // 每个关节动作的符号翻转 (±1.0)

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    Eigen::Vector3f current_angular_velocity_;
    Eigen::Vector3f current_euler_;
    Eigen::Vector3f current_command_;
    std::mutex sensor_data_mutex_;

    std::thread inference_thread_;
    std::atomic<bool> thread_running_;
    std::mutex action_mutex_;

    std::atomic<bool> imu_received_;
    bool require_imu_before_locomotion_;
    bool enable_action_safety_;
    float action_delta_limit_;
    double startup_hold_seconds_;
    std::chrono::steady_clock::time_point init_time_;
    std::chrono::steady_clock::time_point last_imu_time_;  // 最近一次 IMU 数据时间（sensor_data_mutex_ 保护）
    double imu_timeout_;                                   // IMU 数据超时阈值（秒），超时视为失联并保持姿态

    double dt_;
    double phase_period_;
    int frame_stack_;
    int model_obs_dim_;
    std::array<float, NUM_ACTIONS> action_scale_;  // 逐关节动作缩放（与训练配置对齐）
    float obs_clip_;
    float act_clip_;
    float obs_scale_lin_vel_;
    float obs_scale_ang_vel_;
    float obs_scale_dof_pos_;
    float obs_scale_dof_vel_;
    int counter_;
    bool model_ready_;

    std::string model_path_;
    torch::jit::script::Module policy_module_;

    void loadPolicy(const std::string& model_path);
    void inferenceLoop();
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void initializeParameters();
    void collectObservations();
    void runInference();

    // 通过踝关节 IK 设置踝关节电机位置
    // 机构方向翻转由 joint_signs_ 统一处理（右踝非镜像结构: joint_signs_[10] = -1）
    void applyAnkleIK(std::array<float, 23>& positions,
                      int policy_idx_pitch, int policy_idx_roll);

    // 踝关节并联机构 IK
    ParallelAnkle ankle_ik_;
    float prev_L_pitch_ = 0.3f;   // FK 迭代初值：左脚 IK-pitch（对应策略索引4, default 0.3）
    float prev_L_roll_  = 0.0f;   // FK 迭代初值：左脚 IK-roll （对应策略索引5, default 0.0）
    float prev_R_pitch_ = 0.3f;   // FK 迭代初值：右脚机构 IK-pitch（非镜像机构, default 机构侧 +0.3）
    float prev_R_roll_  = 0.0f;   // FK 迭代初值：右脚机构 IK-roll （对应策略索引11, default 0.0）

    // 12维动作中踝关节 pitch/roll 的索引（与 Locomotion 一致）
    static constexpr int IDX_L_FOOT_PITCH = 4;
    static constexpr int IDX_L_FOOT_ROLL  = 5;
    static constexpr int IDX_R_FOOT_PITCH = 10;
    static constexpr int IDX_R_FOOT_ROLL  = 11;
};

#endif // ZROBOT_DEPLOY_PT_LOCOMOTION_H
