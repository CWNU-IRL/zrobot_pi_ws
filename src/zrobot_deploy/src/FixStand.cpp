#include "zrobot_deploy/FixStand.h"
#include <cmath>
#include <algorithm>

FixStand::FixStand(std::shared_ptr<rclcpp::Node> node)
    : FSM(node)
    , internal_state_(InternalState::INIT)
    , interpolation_time_(3.0)  // 3秒
    , elapsed_time_(0.0)
    , is_initialized_(false)
{
    current_state_ = FSMState::FIX_STAND;
    // 站立姿态目标，电机 id 约定与 Locomotion 的 dof_indices_ 一致:
    //   motor 0/1 = 左踝(pitch/roll), 2 = L_knee, 3 = L_hip_pitch, 4 = L_hip_yaw, 5 = L_hip_roll
    //   motor 6/7 = 右踝(pitch/roll), 8 = R_knee, 9 = R_hip_pitch, 10 = R_hip_yaw, 11 = R_hip_roll
    // 髋 pitch/roll 电机方向与 URDF 相反 (joint_signs_=-1)，目标值需取反
    target_positions_.fill(0.0f);
    target_positions_[2] = -0.65f;   // L_knee
    target_positions_[3] = 0.35f;    // L_hip_pitch (sign=-1: -(-0.35))
    target_positions_[4] = 0.0f;     // L_hip_yaw
    target_positions_[5] = 0.0f;     // L_hip_roll (sign=-1: -0.0)
    target_positions_[8] = 0.65f;    // R_knee
    target_positions_[9] = -0.35f;   // R_hip_pitch (sign=-1: -(0.35))
    target_positions_[10] = 0.0f;    // R_hip_yaw
    target_positions_[11] = 0.0f;    // R_hip_roll (sign=-1: -0.0)
    // 踝关节并联机构：用 IK 计算默认姿态对应电机角度
    // 右踝已改为与左踝相同的非镜像结构，故两脚使用相同机构角度
    // (URDF 右踝 pitch 轴为 +Y，与机构 -Y 相反，default -0.3 → 机构 +0.3)
    const auto ik_l = ankle_ik_.ik(0.3, 0.0);
    target_positions_[0] = static_cast<float>(ik_l.theta1);
    target_positions_[1] = static_cast<float>(ik_l.theta2);
    const auto ik_r = ankle_ik_.ik(0.3, 0.0);
    target_positions_[6] = static_cast<float>(ik_r.theta1);
    target_positions_[7] = static_cast<float>(ik_r.theta2);
    initial_positions_.fill(0.0f);
    
    RCLCPP_INFO(node_->get_logger(), "FixStand FSM created");
}

FixStand::~FixStand()
{
    RCLCPP_INFO(node_->get_logger(), "FixStand FSM destroyed");
}

void FixStand::initialize()
{
    FSM::initialize();
    
    RCLCPP_INFO(node_->get_logger(), "FixStand initializing...");
    
    // 获取当前电机位置作为初始位置
    if (getCurrentPositions(initial_positions_)) {
        RCLCPP_INFO(node_->get_logger(), "Initial positions read successfully from get_positions service");
        // RCLCPP_DEBUG(node_->get_logger(), "Sample positions: [0]=%.3f, [1]=%.3f, [2]=%.3f rad", 
        //             initial_positions_[0], initial_positions_[1], initial_positions_[2]);
    } else {
        // 读取失败时拒绝启动：以全 0 为插值起点会把电机猛拉向编码器零位，
        // 若零位未校准（编码器 0 ≠ 机械零位姿态）将直接冲向物理限位。
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to read initial positions from service. FixStand will NOT start. "
                     "Check zrobot_bridge feedback link before retrying.");
        internal_state_ = InternalState::INIT;
        is_initialized_ = false;
        return;
    }
    
    // 上肢电机 (12-22) 不参与 FixStand 姿态规划，保持当前位置而不是被拉回 0
    for (size_t i = 12; i < target_positions_.size(); ++i) {
        target_positions_[i] = initial_positions_[i];
    }
    
    // 重置时间
    elapsed_time_ = 0.0;
    start_time_ = std::chrono::steady_clock::now();
    last_time_ = start_time_;
    
    internal_state_ = InternalState::MOVING;
    is_initialized_ = true;
    
    RCLCPP_INFO(node_->get_logger(), 
                "FixStand initialized, will move to standing pose in %.1f seconds", 
                interpolation_time_);
}

void FixStand::run()
{
    if (!is_initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "FixStand not initialized! Call initialize() first.");
        return;
    }
    
    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - last_time_).count();
    last_time_ = current_time;
    
    switch (internal_state_) {
        case InternalState::INIT:
            RCLCPP_ERROR(node_->get_logger(), "FixStand in INIT state during run!");
            break;
            
        case InternalState::MOVING:
        {
            // 计算插值参数 t (0 到 1)，基于已成功推进的时间
            double t = std::min(elapsed_time_ / interpolation_time_, 1.0);
            
            // 线性插值计算当前目标位置
            for (size_t i = 0; i < 23; ++i) {
                current_motor_positions_[i] = lerp(initial_positions_[i], target_positions_[i], t);
            }
            
            // 发送位置指令；失败时不推进插值时间，下个周期重发同一目标，
            // 避免通信中断期间时间累积、恢复后目标位置突然跳变
            if (!sendMotorPositions(current_motor_positions_)) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to send motor positions");
                break;
            }
            
            // 发送成功后才推进已过时间
            elapsed_time_ += dt;
            
            // 检查是否完成移动
            if (t >= 1.0) {
                internal_state_ = InternalState::STANDING;
                RCLCPP_INFO(node_->get_logger(), "Reached standing position, now holding");
            } else {
                // 每秒打印一次进度
                static double last_print_time = 0.0;
                if (elapsed_time_ - last_print_time >= 1.0) {
                    RCLCPP_INFO(node_->get_logger(), 
                               "Moving to standing pose: %.1f%% complete", 
                               t * 100.0);
                    last_print_time = elapsed_time_;
                }
            }
            break;
        }
            
        case InternalState::STANDING:
        {
            // 保持位置
            current_motor_positions_ = target_positions_;
            
            // 持续发送位置指令以保持位置
            if (!sendMotorPositions(current_motor_positions_)) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to send motor positions");
            }
            
            // 每5秒打印一次状态
            static double last_print_time = 0.0;
            double total_time = std::chrono::duration<double>(current_time - start_time_).count();
            if (total_time - last_print_time >= 5.0) {
                RCLCPP_INFO(node_->get_logger(), "Holding standing pose");
                last_print_time = total_time;
            }
            break;
        }
    }
}

void FixStand::exit()
{
    RCLCPP_INFO(node_->get_logger(), "Exiting FixStand FSM");
    FSM::exit();
}
// 线性插值: start + t * (end - start)
float FixStand::lerp(float start, float end, float t)
{
    return start + t * (end - start);
}
