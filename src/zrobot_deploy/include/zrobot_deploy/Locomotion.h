#ifndef ZROBOT_DEPLOY_LOCOMOTION_H
#define ZROBOT_DEPLOY_LOCOMOTION_H

#include "zrobot_deploy/FSM.h"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <onnxruntime_cxx_api.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include <deque>
#include <array>

// Locomotion 状态机 - 使用 ONNX 模型进行强化学习推理控制
class Locomotion : public FSM
{
public:
    Locomotion(std::shared_ptr<rclcpp::Node> node);
    ~Locomotion() override;
    
    void initialize() override;
    void run() override;
    void exit() override;
    
private:
    // ===== ONNX 推理相关 =====
    // ONNX Runtime 环境
    std::unique_ptr<Ort::Env> ort_env_;
    std::unique_ptr<Ort::Session> ort_session_;
    Ort::SessionOptions session_options_;
    Ort::MemoryInfo memory_info_;
    
    // 输入输出名称
    std::vector<std::string> input_names_storage_;
    std::vector<std::string> output_names_storage_;
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    
    // 观测和动作维度
    static constexpr int NUM_SINGLE_OBS = 47;
    static constexpr int NUM_ACTIONS = 12;
    static constexpr int NUM_MOTORS = 23;
    
    // 数据缓冲区
    Eigen::VectorXf obs_current_;     // 当前单帧观测 (47维)
    std::deque<Eigen::VectorXf> obs_history_;  // 历史观测帧
    Eigen::VectorXf policy_input_;    // 拼接后的模型输入
    
    Eigen::VectorXf act_prev_;        // 上一次原始动作 (12维)
    Eigen::VectorXf act_scaled_;      // 缩放后的动作增量 (12维)
    Eigen::VectorXf act_temp_;        // 临时动作缓冲（用于线程安全复制）
    Eigen::VectorXf default_pose_;    // 默认姿态 (12维)
    std::array<int, NUM_ACTIONS> dof_indices_;  // 12维策略动作映射到23维电机索引
    
    // ===== ROS 话题订阅相关 =====
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    
    // 当前传感器数据
    Eigen::Vector3f current_angular_velocity_;   // 角速度 (3维)
    Eigen::Vector3f current_gravity_vector_;     // 重力向量 (3维)
    Eigen::Vector3f current_euler_;              // 欧拉角 (roll, pitch, yaw)
    Eigen::Vector3f current_command_;            // 遥控命令 (3维: vx, vy, wz)
    
    std::mutex sensor_data_mutex_;               // 传感器数据锁
    
    // ===== 推理线程相关 =====
    std::thread inference_thread_;
    std::atomic<bool> thread_running_;
    std::mutex action_mutex_;                     // 动作数据锁

     // 启动与安全保护
     std::atomic<bool> imu_received_;
     std::atomic<bool> motor_feedback_ready_;
     bool require_imu_before_locomotion_;
     bool enable_action_safety_;
     double startup_hold_seconds_;
     float action_delta_limit_;
     float action_abs_limit_;
     std::chrono::steady_clock::time_point init_time_;
    
    // ===== 控制参数 =====
    double dt_;                                   // 控制周期 (秒)
    double phase_period_;                         // 相位周期 (秒)
    int frame_stack_;                             // 观测堆叠帧数
    int model_obs_dim_;                           // 模型输入维度
    float action_scale_;                          // 动作缩放
    float obs_clip_;                              // 观测裁剪
    float act_clip_;                              // 动作裁剪
    float obs_scale_lin_vel_;                     // cmd线速度缩放
    float obs_scale_ang_vel_;                     // cmd角速度缩放
    float obs_scale_dof_pos_;                     // 关节角缩放
    float obs_scale_dof_vel_;                     // 关节速度缩放
    int counter_;                                 // 循环计数器（用于相位计算）
    bool model_ready_;                            // 模型是否可推理

    // 加载 ONNX 模型
    void loadPolicy(const std::string& model_path);
    
    // 推理线程函数
    void inferenceLoop();
    
    // ROS 回调函数
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    // 初始化参数
    void initializeParameters();
    
    // 收集观测数据
    void collectObservations();
    
    // 执行 ONNX 推理
    void runInference();
};

#endif // ZROBOT_DEPLOY_LOCOMOTION_H
