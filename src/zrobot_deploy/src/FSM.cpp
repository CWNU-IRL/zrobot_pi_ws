#include "zrobot_deploy/FSM.h"

FSM::FSM(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
    , current_state_(FSMState::IDLE)
    , control_frequency_(100.0)  // 100Hz
{
    // 初始化
    current_motor_positions_.fill(0.0f);
    feedback_positions_.fill(0.0f);
    feedback_velocities_.fill(0.0f);
    feedback_torques_.fill(0.0f);
    feedback_temperatures_.fill(0.0f);
    
    // 创建服务客户端
    control_client_ = node_->create_client<rs_interface::srv::RobStrideMsgs>("/rob_stride_control");
    get_positions_client_ = node_->create_client<rs_interface::srv::GetPositions>("/get_positions");
    
    RCLCPP_INFO(node_->get_logger(), "FSM initialized");
}

FSM::~FSM()
{
    RCLCPP_INFO(node_->get_logger(), "FSM destroyed");
}

void FSM::initialize()
{
    // 等待服务可用
    RCLCPP_INFO(node_->get_logger(), "Waiting for /rob_stride_control service...");
    while (!control_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for service");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Service not available, waiting...");
    }
    RCLCPP_INFO(node_->get_logger(), "Service /rob_stride_control is ready");
}

void FSM::exit()
{
    RCLCPP_INFO(node_->get_logger(), "FSM exiting");
}

/**
 * @brief 向底层控制器发送23个电机的目标位置指令
 * 
 * 通过调用 /rob_stride_control 服务发送电机位置控制指令，并接收电机反馈数据。
 * 该函数会阻塞等待服务响应，超时时间为100ms。
 * 
 * @param positions 包含23个电机目标位置的数组（单位：弧度）
 * @return true  服务调用成功且电机控制成功
 * @return false 服务调用失败或超时，或电机控制失败
 * 
 * @note 函数执行流程：
 *       1. 创建服务请求，填充目标位置
 *       2. 异步发送请求到 /rob_stride_control 服务
 *       3. 等待响应（最多100ms）
 *       4. 如果成功，更新电机反馈数据（位置、速度、力矩、温度）
 *       5. 检查响应中的 success 标志
 * 
 * @warning 调用此函数前必须确保服务已初始化且可用
 * @warning 函数会阻塞最多100ms，可能影响控制循环时序
 */
bool FSM::sendMotorPositions(const std::array<float, 23>& positions)
{
    // 创建服务请求对象
    auto request = std::make_shared<rs_interface::srv::RobStrideMsgs::Request>();
    request->positions = positions;
    
    // 异步发送服务请求
    auto future = control_client_->async_send_request(request);
    
    // 等待响应
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(100)) 
        == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        if (response->feedback_positions.size() == 23) {
            for (size_t i = 0; i < 23; ++i) {
                feedback_positions_[i] = response->feedback_positions[i];
                feedback_velocities_[i] = response->feedback_velocities[i];
                feedback_torques_[i] = response->feedback_torques[i];
                feedback_temperatures_[i] = response->feedback_temperatures[i];
            }
        }
        
        if (!response->success) {
            RCLCPP_WARN(node_->get_logger(), "Motor control failed: %s", response->message.c_str());
        }
        
        return response->success;
    }
    else
    {
        // 服务调用超时或失败
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service /rob_stride_control");
        return false;
    }
}

/**
 * @brief 获取最近一次电机反馈数据
 * 
 * 从内部缓存中读取最近一次 sendMotorPositions() 调用时接收到的电机反馈数据。
 * 该函数不会进行服务调用，只是读取已缓存的数据。
 * 
 * @param[out] positions     23个电机的反馈位置（单位：弧度）
 * @param[out] velocities    23个电机的反馈速度（单位：弧度/秒）
 * @param[out] torques       23个电机的反馈力矩（单位：N·m）
 * @param[out] temperatures  23个电机的反馈温度（单位：摄氏度）
 * @return true 总是返回true（数据读取成功）
 * 
 * @note 返回的数据来自上一次成功调用 sendMotorPositions() 时的服务响应
 * @note 如果从未调用过 sendMotorPositions()，返回的数据都是初始值0
 * 
 * @see sendMotorPositions() 用于发送位置指令并更新反馈数据
 */
bool FSM::getMotorFeedback(std::array<float, 23>& positions,
                          std::array<float, 23>& velocities,
                          std::array<float, 23>& torques,
                          std::array<float, 23>& temperatures)
{
    positions = feedback_positions_;
    velocities = feedback_velocities_;
    torques = feedback_torques_;
    temperatures = feedback_temperatures_;
    return true;
}

/**
 * @brief 通过调用get_positions服务获取当前电机位置
 * 
 * 该方法调用/get_positions服务来读取所有23个电机的当前位置。
 * 与sendMotorPositions不同，此方法仅读取位置，不发送控制指令。
 * 
 * @param[out] positions 用于存储读取到的23个电机位置的数组（单位：弧度）
 * @return true  服务调用成功且位置读取成功
 * @return false 服务调用失败、超时或服务返回失败状态
 * 
 * @note 函数执行流程：
 *       1. 等待get_positions服务可用（最多1秒）
 *       2. 发送服务请求
 *       3. 等待响应（最多5秒，因为每个电机可能需要时间）
 *       4. 将响应中的位置数据复制到输出数组
 * 
 * @warning 该函数会阻塞等待服务响应，最多可能阻塞6秒
 * @warning 调用前必须确保get_positions服务已启动
 */
bool FSM::getCurrentPositions(std::array<float, 23>& positions)
{
    // 等待服务可用（最多1秒）
    if (!get_positions_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "get_positions service not available");
        return false;
    }
    
    // 创建服务请求（GetPos服务的请求为空）
    auto request = std::make_shared<rs_interface::srv::GetPositions::Request>();
    
    // 异步发送服务请求
    auto future = get_positions_client_->async_send_request(request);
    
    // 等待服务响应（超时时间：5秒，因为read_initial_position可能需要较长时间）
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) 
        == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        
        // 检查服务调用是否成功
        if (!response->success) {
            RCLCPP_WARN(node_->get_logger(), "get_positions service failed: %s", 
                       response->message.c_str());
            return false;
        }
        
        // 将读取到的位置复制到输出数组
        for (size_t i = 0; i < 23; ++i) {
            positions[i] = response->feedback_positions[i];
        }
        
        RCLCPP_INFO(node_->get_logger(), "Successfully read current motor positions");
        return true;
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call get_positions service (timeout)");
        return false;
    }
}
