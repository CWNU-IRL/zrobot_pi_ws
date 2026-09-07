#ifndef ZROBOT_DEPLOY_LOCOMOTION_H
#define ZROBOT_DEPLOY_LOCOMOTION_H

#include "zrobot_deploy/FSM.h"
#include "zrobot_deploy/paralleljoint_z1.h"
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
#include <fstream>
#include <ctime>

// Locomotion 状态机 - 使用 ONNX 模型进行强化学习推理控制
class Locomotion : public FSM
{
public:
    Locomotion(std::shared_ptr<rclcpp::Node> node);
    ~Locomotion() override;
    
    void initialize() override;
    void run() override;
    void exit() override;
    bool isReady() const override { return model_ready_; }
    
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
    std::array<int, NUM_ACTIONS> dof_indices_;   // 12维策略动作映射到23维电机索引
    std::array<float, NUM_ACTIONS> joint_signs_; // 每个关节动作的符号翻转 (±1.0)，用于匹配仿真→实物方向
    
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
     float action_filter_alpha_;      // 动作一阶低通系数（训练 zrobot_env delay 滤波同族）
     double action_fade_in_seconds_;  // hold 结束后动作线性渐入时长（<=0 关闭）
     std::chrono::steady_clock::time_point init_time_;
     std::chrono::steady_clock::time_point last_imu_time_;  // 最近一次 IMU 数据时间（sensor_data_mutex_ 保护）
     double imu_timeout_;                                   // IMU 数据超时阈值（秒），超时视为失联并保持姿态
    
    // ===== 控制参数 =====
    double dt_;                                   // 控制周期 (秒)
    double phase_period_;                         // 相位周期 (秒)
    int frame_stack_;                             // 观测堆叠帧数
    int model_obs_dim_;                           // 模型输入维度
    std::array<float, NUM_ACTIONS> action_scale_;  // 逐关节动作缩放（与训练配置对齐）
    float obs_clip_;                              // 观测裁剪
    float act_clip_;                              // 动作裁剪
    float obs_scale_lin_vel_;                     // cmd线速度缩放
    float obs_scale_ang_vel_;                     // cmd角速度缩放
    float obs_scale_dof_pos_;                     // 关节角缩放
    float obs_scale_dof_vel_;                     // 关节速度缩放
    int counter_;                                 // 推理循环计数（仅统计，相位改用墙钟）
    bool model_ready_;                            // 模型是否可推理

    // 数据记录（CSV：obs 输入 + action 输出 + 目标/反馈，对齐 sim2sim 日志）
    std::ofstream log_file_;
    bool enable_logging_;
    int log_decimation_;                          // 每 N 个推理周期写一行

    // IMU yaw 零偏（磁力计绝对航向 → 相对初始航向，与训练世界系 yaw≈0 对齐）
    double yaw_offset_ = 0.0;
    bool yaw_offset_set_ = false;

    // 踝关节并联机构 IK
    ParallelAnkle ankle_ik_;
    float prev_L_pitch_ = 0.19f;   // FK 迭代初值：左脚 IK-pitch（对应策略索引4, default 0.3）
    float prev_L_roll_  = 0.0f;   // FK 迭代初值：左脚 IK-roll （对应策略索引5, default 0.0）
    float prev_R_pitch_ = 0.19f;   // FK 迭代初值：右脚机构 IK-pitch（非镜像机构, default 机构侧 +0.3）
    float prev_R_roll_  = 0.0f;   // FK 迭代初值：右脚机构 IK-roll （对应策略索引11, default 0.0）
    
    // 12维动作中踝关节 pitch/roll 的索引
    // static constexpr int IDX_L_FOOT_PITCH = 4;
    // static constexpr int IDX_L_FOOT_ROLL  = 5;
    // static constexpr int IDX_R_FOOT_PITCH = 10;
    // static constexpr int IDX_R_FOOT_ROLL  = 11;
    static constexpr int IDX_L_FOOT_PITCH = 4;
    static constexpr int IDX_L_FOOT_ROLL  = 5;
    static constexpr int IDX_R_FOOT_PITCH = 10;
    static constexpr int IDX_R_FOOT_ROLL  = 11;

    // 加载 ONNX 模型
    void loadPolicy(const std::string& model_path);
    
    // 推理线程函数
    void inferenceLoop();
    
    // 通过踝关节 IK 设置踝关节电机位置
    // 机构方向翻转由 joint_signs_ 统一处理（右踝非镜像结构: joint_signs_[10] = -1）
    void applyAnkleIK(std::array<float, 23>& positions,
                      int policy_idx_pitch, int policy_idx_roll);
    
    // ROS 回调函数
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    // 初始化参数
    void initializeParameters();
    
    // 收集观测数据
    void collectObservations();
    
    // 执行 ONNX 推理
    void runInference();

    // 写一行 CSV: seq,t,obs[47],act_policy[12],act_applied[12],target_q[12],q_curr[12],tau_motor[12]
    void writeLogRow(const Eigen::VectorXf& policy_action, const Eigen::VectorXf& applied_action);
};

#endif // ZROBOT_DEPLOY_LOCOMOTION_H
