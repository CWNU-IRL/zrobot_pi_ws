#include "zrobot_bridge/motor_controller.h"
#include <string>
#include <vector>

/**
 * @brief MotorControllerNode 构造函数
 * 
 * 初始化电机控制节点，包括以下步骤：
 * 1. 初始化ROS 2节点，节点名称为 "motor_controller_node"
 * 2. 声明并获取ROS参数：
 *    - master_id: 主机ID (默认值: 0)
 *    - motor_can_ids: 电机CAN ID列表 (默认值: vector<0>)
 *    - motor_types: 电机类型列表 (默认值: vector<0>)
 *    - motor_can_interfaces: 电机CAN接口列表 (默认值: vector<"can0">)
 * 3. 验证参数有效性，若参数大小不符合NUM_MOTORS则使用默认值
 * 4. 调用initialize_motors()初始化所有电机
 * 5. 创建ROS服务 "rob_stride_control" 用于电机控制
 * 
 * @throw 若任何电机初始化失败，将记录错误但不会抛出异常
 * @see initialize_motors() - 电机初始化方法
 * @see handle_rob_stride_service() - 服务回调函数
 */
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
    // 电机的CAN ID
    auto can_ids_param = this->get_parameter("motor_can_ids").as_integer_array();
    // 电机类型
    auto types_param = this->get_parameter("motor_types").as_integer_array();
    // 电机CAN接口
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
    set_zeros_service_ = this->create_service<rs_interface::srv::SetZeros>(
        "set_zeros",
        std::bind(&MotorControllerNode::handle_set_zeros_service, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(logger_, "零点设置服务已创建: /motor_controller_node/set_zeros");

    get_positions_service_ = this->create_service<rs_interface::srv::GetPositions>(
            "get_positions",
            std::bind(&MotorControllerNode::handle_get_positions_service, this,
                    std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(logger_, "位置读取服务已创建: /motor_controller_node/get_positions");

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

/**
 * @brief 初始化所有电机
 * 
 * 该方法遍历所有电机配置（由构造函数加载的参数），为每个电机创建一个RobStrideMotor对象。
 * 
 * 执行步骤：
 * 1. 使用互斥锁锁定motors_数组，确保线程安全
 * 2. 记录初始化开始的日志信息
 * 3. 对每个电机执行以下操作：
 *    - 从配置数组中获取该电机的CAN ID、类型和CAN接口信息
 *    - 使用std::make_shared创建RobStrideMotor对象并存储在motors_[i]中
 *    - 若初始化成功，记录调试日志信息，包含CAN接口、CAN ID和电机类型
 *    - 若初始化失败，记录错误日志，并将motors_[i]设置为nullptr
 * 4. 记录初始化完成的日志信息
 * 
 * @thread_safety 使用std::lock_guard保护motors_数组的访问，确保初始化过程中的线程安全
 * 
 * @note 该方法不会因为某个电机初始化失败而中断整个过程，失败的电机将被标记为nullptr
 * 
 * @see RobStrideMotor - 电机驱动类
 * @see motors_ - 存储所有电机指针的数组
 */
void MotorControllerNode::initialize_motors()
{
    std::lock_guard<std::mutex> lock(motors_mutex_);

    RCLCPP_INFO(logger_, "开始初始化%d个电机...", NUM_MOTORS);
    // 遍历所有电机配置，创建RobStrideMotor对象
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

/**
 * @brief RobStrideMsgs 服务回调
 *
 * 处理步态控制服务请求：
 * 1. 加锁保护motors_，确保控制和反馈采集线程安全
 * 2. 校验请求位置数组尺寸是否等于NUM_MOTORS
 * 3. 逐电机发送位置/速度/KP/KD指令，收集位置、速度、扭矩、温度反馈
 * 4. 若单个电机初始化缺失或控制失败，仅影响该电机反馈并记录日志，其余电机继续
 *
 * @param request 服务请求，包含目标位置数组
 * @param response 服务响应，包含反馈数据以及成功与否信息
 *
 * @note 响应数组为固定长度[NUM_MOTORS]，无需resize；异常会被捕获并写入日志
 */
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

void MotorControllerNode::handle_set_zeros_service(
    const std::shared_ptr<rs_interface::srv::SetZeros::Request>,
    std::shared_ptr<rs_interface::srv::SetZeros::Response> response)
{
    std::lock_guard<std::mutex> lock(motors_mutex_);

    bool has_error = false;

    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        if (!motors_[i])
        {
            RCLCPP_WARN(logger_, "电机%d未初始化，跳过零点设置", i);
            has_error = true;
            continue;
        }

        try
        {
            motors_[i]->Set_ZeroPos();
            RCLCPP_INFO(logger_, "电机%d零点设置指令已发送", i);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "电机%d设置零点失败: %s", i, e.what());
            has_error = true;
        }
    }

    response->success = !has_error;
    response->message = has_error ? "部分或全部电机零点设置失败" : "所有电机零点设置指令已发送";
}

/**
 * @brief 获取所有电机当前位置的服务回调
 * 
 * 通过调用每个电机的 read_initial_position() 方法读取当前位置。
 * 
 * 执行流程：
 * 1. 使用互斥锁保护电机访问，确保线程安全
 * 2. 遍历所有电机，调用 read_initial_position() 读取位置
 * 3. 将读取到的位置存储到响应数组中
 * 4. 若电机未初始化或读取失败，该位置设为0.0并记录警告
 * 5. 返回是否有错误发生的状态
 * 
 * @param request  服务请求（空请求，无参数）
 * @param response 服务响应，包含23个电机的位置、成功标志和消息
 * 
 * @thread_safety 使用互斥锁保护电机数组访问
 * 
 * @note 该方法会阻塞，因为 read_initial_position() 可能需要等待电机反馈
 * @note 每个电机的读取最多等待10秒（read_initial_position的超时时间）
 * 
 * @see RobStrideMotor::read_initial_position() - 读取电机位置的底层方法
 */
void MotorControllerNode::handle_get_positions_service(
    const std::shared_ptr<rs_interface::srv::GetPositions::Request>,
    std::shared_ptr<rs_interface::srv::GetPositions::Response> response)
{
    std::lock_guard<std::mutex> lock(motors_mutex_);

    RCLCPP_INFO(logger_, "接收到获取电机位置服务请求");

    bool has_error = false;

    // 遍历所有电机，读取初始位置
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        if (!motors_[i])
        {
            RCLCPP_WARN(logger_, "电机%d未初始化，位置设为0", i);
            response->feedback_positions[i] = 0.0f;
            has_error = true;
            continue;
        }

        try
        {
            // 调用 read_initial_position 读取当前位置
            float position = motors_[i]->read_initial_position();
            response->feedback_positions[i] = position;
            
            RCLCPP_INFO(logger_, "电机%d当前位置: %.3f rad", i, position);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "读取电机%d位置失败: %s", i, e.what());
            response->feedback_positions[i] = 0.0f;
            has_error = true;
        }
    }

    response->success = !has_error;
    response->message = has_error 
        ? "部分或全部电机位置读取失败" 
        : "所有电机位置读取成功";
    
    RCLCPP_INFO(logger_, "位置读取服务请求处理完成");
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
