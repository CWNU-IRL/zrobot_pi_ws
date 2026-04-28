#ifndef ZROBOT_DEPLOY_PT_LOCOMOTION_H
#define ZROBOT_DEPLOY_PT_LOCOMOTION_H

#include "zrobot_deploy/FSM.h"
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
    double startup_hold_seconds_;
    std::chrono::steady_clock::time_point init_time_;

    double dt_;
    double phase_period_;
    int frame_stack_;
    int model_obs_dim_;
    float action_scale_;
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
};

#endif // ZROBOT_DEPLOY_PT_LOCOMOTION_H
