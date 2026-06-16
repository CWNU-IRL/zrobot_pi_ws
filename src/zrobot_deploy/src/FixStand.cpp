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
    // target: training standing posture (matches sim2sim default_angle)
    target_positions_.fill(0.0f);
    // motor 0: L_hip_roll=0.0, 1: L_hip_yaw=0.0, 2: L_hip_pitch=-0.45, 3: L_knee=-0.85, 4: L_foot_pitch=0.4, 5: L_foot_roll=0.0
    target_positions_[2] = -0.45f;
    target_positions_[3] = -0.85f;
    target_positions_[4] = 0.4f;
    // motor 6: R_hip_roll=0.0, 7: R_hip_yaw=0.0, 8: R_hip_pitch=0.45, 9: R_knee=0.85, 10: R_foot_pitch=-0.4, 11: R_foot_roll=0.0
    target_positions_[8] = 0.45f;
    target_positions_[9] = 0.85f;
    target_positions_[10] = -0.4f;
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
        RCLCPP_WARN(node_->get_logger(), "Failed to read initial positions from service, assuming zero");
        initial_positions_.fill(0.0f);
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
            // 更新已过时间
            elapsed_time_ += dt;
            
            // 计算插值参数 t (0 到 1)
            double t = std::min(elapsed_time_ / interpolation_time_, 1.0);
            
            // 线性插值计算当前目标位置
            for (size_t i = 0; i < 23; ++i) {
                current_motor_positions_[i] = lerp(initial_positions_[i], target_positions_[i], t);
            }
            
            // 发送位置指令
            if (!sendMotorPositions(current_motor_positions_)) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to send motor positions");
            }
            
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
