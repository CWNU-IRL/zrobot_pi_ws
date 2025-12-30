#include "zrobot_bridge/motor_controller.h"
#include <string>
#include <vector>

MotorControllerNode::MotorControllerNode()
    : rclcpp::Node("motor_controller_node"),
      logger_(this->get_logger())
{
    RCLCPP_INFO(logger_, "初始化电机控制节点");

    // 声明参数
    this->declare_parameter("master_id", 0);
    this->declare_parameter<std::vector<int64_t>>("motor_can_ids", std::vector<int64_t>(NUM_MOTORS, 0));
    this->declare_parameter<std::vector<int64_t>>("motor_types", std::vector<int64_t>(NUM_MOTORS, 0));
    this->declare_parameter<std::vector<std::string>>("motor_can_interfaces", std::vector<std::string>(NUM_MOTORS, "can0"));

    // 获取参数
    master_id_ = static_cast<uint8_t>(this->get_parameter("master_id").as_int());

    auto can_ids_param = this->get_parameter("motor_can_ids").as_integer_array();
    auto types_param = this->get_parameter("motor_types").as_integer_array();
    auto interfaces_param = this->get_parameter("motor_can_interfaces").as_string_array();

    // 验证参数
    if (can_ids_param.size() != NUM_MOTORS)
    {
        RCLCPP_WARN(logger_, "电机CAN ID列表大小不正确，默认使用ID 0-22");
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            motor_can_ids_[i] = i;
        }
    }
    else
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            motor_can_ids_[i] = static_cast<uint8_t>(can_ids_param[i]);
        }
    }

    if (types_param.size() != NUM_MOTORS)
    {
        RCLCPP_WARN(logger_, "电机类型列表大小不正确，默认使用类型0");
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            motor_types_[i] = 0;
        }
    }
    else
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            motor_types_[i] = static_cast<int>(types_param[i]);
        }
    }

    if (interfaces_param.size() != NUM_MOTORS)
    {
        RCLCPP_WARN(logger_, "电机CAN接口列表大小不正确，默认使用can0");
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            motor_can_interfaces_[i] = "can0";
        }
    }
    else
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            motor_can_interfaces_[i] = interfaces_param[i];
        }
    }

    RCLCPP_INFO(logger_, "主机ID: %d", master_id_);

    // 初始化电机
    initialize_motors();

    // 创建服务
    service_ = this->create_service<rs_interface::srv::RobStrideMsgs>(
        "rob_stride_control",
        std::bind(&MotorControllerNode::handle_rob_stride_service, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(logger_, "电机控制服务已创建: /motor_controller_node/rob_stride_control");
}

MotorControllerNode::~MotorControllerNode()
{
    RCLCPP_INFO(logger_, "关闭电机控制节点");
    // 停用所有电机
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        if (motors_[i])
        {
            try
            {
                motors_[i]->Disenable_Motor(0);
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(logger_, "关闭电机%d时出错: %s", i, e.what());
            }
        }
    }
}

void MotorControllerNode::initialize_motors()
{
    std::lock_guard<std::mutex> lock(motors_mutex_);

    RCLCPP_INFO(logger_, "开始初始化%d个电机...", NUM_MOTORS);

    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        try
        {
            uint8_t can_id = motor_can_ids_[i];
            int motor_type = motor_types_[i];
            std::string can_interface = motor_can_interfaces_[i];

            motors_[i] = std::make_shared<RobStrideMotor>(
                can_interface, master_id_, can_id, motor_type);

            RCLCPP_DEBUG(logger_, "电机%d初始化成功 (CAN接口: %s, CAN ID: %d, 类型: %d)",
                         i, can_interface.c_str(), can_id, motor_type);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "初始化电机%d失败: %s", i, e.what());
            motors_[i] = nullptr;
        }
    }

    RCLCPP_INFO(logger_, "电机初始化完成");
}

void MotorControllerNode::handle_rob_stride_service(
    const std::shared_ptr<rs_interface::srv::RobStrideMsgs::Request> request,
    std::shared_ptr<rs_interface::srv::RobStrideMsgs::Response> response)
{
    std::lock_guard<std::mutex> lock(motors_mutex_);

    RCLCPP_INFO(logger_, "接收到RobStrideMsgs服务请求");

    // 验证请求数据
    if (request->positions.size() != NUM_MOTORS)
    {
        response->success = false;
        response->message = "请求的位置数据数量不匹配，期望23个";
        RCLCPP_ERROR(logger_, "%s", response->message.c_str());
        return;
    }

    // 初始化响应数据
    // 注意：响应的数组是固定大小的[23]，不需要resize

    try
    {
        // 向所有电机发送指令并收集反馈
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            if (!motors_[i])
            {
                RCLCPP_WARN(logger_, "电机%d未初始化，跳过", i);
                continue;
            }

            float target_position = request->positions[i];

            try
            {
                // 发送运控模式指令（位置 + 速度 + KP + KD）
                // 参数：torque, position, velocity, kp, kd
                auto [pos, vel, torq, temp] = motors_[i]->send_motion_command(
                    0.0f,                  // torque
                    target_position,       // position (rad)
                    5.0f,                  // velocity (rad/s)
                    0.5f,                  // kp
                    0.1f                   // kd
                );

                // 存储反馈数据
                response->feedback_positions[i] = pos;
                response->feedback_velocities[i] = vel;
                response->feedback_torques[i] = torq;
                response->feedback_temperatures[i] = temp;

                RCLCPP_DEBUG(logger_, 
                    "电机%d 目标位置: %.3f rad, 反馈 - 位置: %.3f, 速度: %.3f, 扭矩: %.3f, 温度: %.1f",
                    i, target_position, pos, vel, torq, temp);
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(logger_, "电机%d 控制失败: %s", i, e.what());
                response->feedback_positions[i] = 0.0f;
                response->feedback_velocities[i] = 0.0f;
                response->feedback_torques[i] = 0.0f;
                response->feedback_temperatures[i] = 0.0f;
            }
        }

        response->success = true;
        response->message = "所有电机已成功控制";
        RCLCPP_INFO(logger_, "RobStrideMsgs服务请求处理完成");
    }
    catch (const std::exception& e)
    {
        response->success = false;
        response->message = std::string("处理请求时出错: ") + e.what();
        RCLCPP_ERROR(logger_, "%s", response->message.c_str());
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
