#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rs_interface/srv/rob_stride_msgs.hpp"
#include "zrobot_bridge/motor_cfg.h"

#include <memory>
#include <array>
#include <map>
#include <thread>
#include <mutex>

/**
 * @class MotorControllerNode
 * @brief ROS2节点，用于控制23个RobStride电机并与RobStrideMsgs服务通信
 */
class MotorControllerNode : public rclcpp::Node
{
public:
    MotorControllerNode();
    ~MotorControllerNode();

private:
    // 服务回调函数
    void handle_rob_stride_service(
        const std::shared_ptr<rs_interface::srv::RobStrideMsgs::Request> request,
        std::shared_ptr<rs_interface::srv::RobStrideMsgs::Response> response);

    // 初始化电机
    void initialize_motors();

    // // 从参数服务器获取电机配置
    // bool load_motor_configuration();

    // 主机ID
    uint8_t master_id_;

    // 电机数量（固定为23）
    static constexpr int NUM_MOTORS = 23;

    // 电机数组：key为电机索引(0-22)，value为RobStrideMotor对象的指针
    std::array<std::shared_ptr<RobStrideMotor>, NUM_MOTORS> motors_;

    // 电机CAN ID映射：index -> CAN ID
    std::array<uint8_t, NUM_MOTORS> motor_can_ids_;

    // 电机类型映射：index -> ActuatorType
    std::array<int, NUM_MOTORS> motor_types_;

    // 电机CAN接口映射：index -> CAN interface name
    std::array<std::string, NUM_MOTORS> motor_can_interfaces_;

    // 服务提供者
    rclcpp::Service<rs_interface::srv::RobStrideMsgs>::SharedPtr service_;

    // 互斥锁，用于电机访问的线程安全
    mutable std::mutex motors_mutex_;

    // 日志记录器
    rclcpp::Logger logger_;
};
