#include "zrobot_gz_sim/gazebo_motor_bridge_node.hpp"

#include <cstdio>
#include <functional>

/**
 * @brief GazeboMotorBridgeNode 构造函数
 *
 * 初始化 Gazebo 电机桥接节点，包括：
 * - 从参数服务器读取关节名称和反馈温度
 * - 验证和处理关节数量
 * - 构建关节名称到索引的映射
 * - 创建主题发布者和订阅者
 * - 创建三个 ROS 2 服务用于控制和反馈
 */
GazeboMotorBridgeNode::GazeboMotorBridgeNode()
    : Node("gazebo_motor_bridge_node"),
      active_joint_count_(kNumMotors),
      feedback_temperature_(35.0f),
      has_joint_state_(false)
{
    // 从参数服务器读取参数，提供默认值
    joint_names_ = this->declare_parameter<std::vector<std::string>>("joint_names", default_joint_names());
    
    std::vector<double> default_kp(kNumMotors, 40.0);
    std::vector<double> default_kd(kNumMotors, 1.0);
    kp_ = this->declare_parameter<std::vector<double>>("kp", default_kp);
    kd_ = this->declare_parameter<std::vector<double>>("kd", default_kd);
    
    feedback_temperature_ = static_cast<float>(this->declare_parameter<double>("feedback_temperature", 35.0));

    // 检查 joint_names 是否为空，为空则使用默认值
    if (joint_names_.empty())
    {
        RCLCPP_WARN(
            get_logger(),
            "joint_names is empty. Falling back to default names.");
        joint_names_ = default_joint_names();
    }

    active_joint_count_ = joint_names_.size();

    // 检查关节数量是否超过预设值
    if (active_joint_count_ > kNumMotors)
    {
        RCLCPP_WARN(
            get_logger(),
            "joint_names size is %zu, larger than %zu. Extra joints will be ignored.",
            active_joint_count_, kNumMotors);
        joint_names_.resize(kNumMotors);
        active_joint_count_ = kNumMotors;
    }

    // 检查关节数量是否少于最大数量，剩余的索引被零填充
    if (active_joint_count_ < kNumMotors)
    {
        RCLCPP_WARN(
            get_logger(),
            "joint_names size is %zu, less than %zu. Remaining service channels will be zero-filled.",
            active_joint_count_, kNumMotors);
    }

    // 构建关节名称到数组索引的映射表，用于快速查找
    for (size_t i = 0; i < active_joint_count_; ++i)
    {
        joint_name_to_index_[joint_names_[i]] = i;
    }

    // 初始化状态数组（零偏移、位置、速度、力矩）
    zero_offsets_.fill(0.0f);
    last_positions_.fill(0.0f);
    last_velocities_.fill(0.0f);
    last_efforts_.fill(0.0f);

    // 检查并调整 kp_ 和 kd_ 的大小以匹配 active_joint_count_
    if (kp_.size() < active_joint_count_) {
        RCLCPP_WARN(get_logger(), "kp size (%zu) is less than active joints (%zu). Paddding with 40.0", kp_.size(), active_joint_count_);
        kp_.resize(active_joint_count_, 40.0);
    }
    if (kd_.size() < active_joint_count_) {
        RCLCPP_WARN(get_logger(), "kd size (%zu) is less than active joints (%zu). Paddding with 1.0", kd_.size(), active_joint_count_);
        kd_.resize(active_joint_count_, 1.0);
    }

    // 创建发布者，由joint_group_effort_controller控制器订阅
    command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/joint_group_effort_controller/commands", 10);

    // 订阅Gazebo的/joint_states话题，获取关节状态信息
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 50, std::bind(&GazeboMotorBridgeNode::on_joint_state, this, std::placeholders::_1));

    // 创建服务：处理机器人步态控制请求和返回反馈
    rob_stride_service_ = create_service<rs_interface::srv::RobStrideMsgs>(
        "/rob_stride_control",
        std::bind(&GazeboMotorBridgeNode::on_rob_stride_control, this, std::placeholders::_1, std::placeholders::_2));

    // 创建服务：获取当前关节位置
    get_positions_service_ = create_service<rs_interface::srv::GetPositions>(
        "/get_positions",
        std::bind( &GazeboMotorBridgeNode::on_get_positions, this, std::placeholders::_1, std::placeholders::_2));

    // 创建服务：设置零位偏移（校准原点）
    set_zeros_service_ = create_service<rs_interface::srv::SetZeros>(
        "/set_zeros",
        std::bind(&GazeboMotorBridgeNode::on_set_zeros, this, std::placeholders::_1, std::placeholders::_2));

    is_target_initialized_ = false;
    target_positions_.fill(0.0f);

    // 创建定时器：5ms（200Hz），用于持续下发力矩维持站立
    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(5), 
        std::bind(&GazeboMotorBridgeNode::control_loop, this));

    // 输出初始化完成的日志信息
    RCLCPP_INFO(get_logger(), "Gazebo motor bridge is ready. Active joints: %zu", active_joint_count_);
}

/**
 * @brief 生成默认关节名称列表
 * 
 * 当参数服务器中未指定关节名称时，使用此函数生成默认的关节名称。
 * 按照 "joint_01", "joint_02", ..., "joint_XX" 的格式生成，其中 XX 为两位数字。
 * 
 * @return std::vector<std::string> 包含所有默认关节名称的向量
 */
std::vector<std::string> GazeboMotorBridgeNode::default_joint_names()
{
    std::vector<std::string> names;
    names.reserve(kNumMotors);
    for (size_t i = 1; i <= kNumMotors; ++i)
    {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "joint_%02zu", i); // %02zu 两位无符号整数，左补零
        names.emplace_back(buf);
    }
    return names;
}

double GazeboMotorBridgeNode::compute_tau(
    double target_q,
    double current_q,
    double target_dq,
    double current_dq,
    double kp,
    double kd) const
{
    return (target_q - current_q) * kp + (target_dq - current_dq) * kd;
}


/**
 * @brief 关节状态回调函数
 * 
 * 本函数作为 /joint_states 话题的订阅回调，当接收到来自 Gazebo 仿真器的关节状态消息时被触发。
 * 该方法负责：
 * - 从消息中提取各关节的位置、速度和力矩数据
 * - 使用关节名称到数组索引的映射表进行快速查询
 * - 将接收到的数据存储到对应的状态数组中，供其他服务使用
 * - 标记已收到关节状态，用于后续的有效性检查
 * 
 * @param msg 指向 JointState 消息的共享指针，包含所有关节的名称、位置、速度和力矩数据
 * 
 * @note 使用互斥锁保护共享状态，确保线程安全。在多线程环境中防止数据竞争。
 * @note 若关节名称在 joint_name_to_index_ 映射中不存在，则跳过该关节（continue）。
 * @note 分别检查位置、速度、力矩数据的数组大小，避免数组越界访问。
 */
void GazeboMotorBridgeNode::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    // 遍历所有关节
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        auto it = joint_name_to_index_.find(msg->name[i]); // 获取对应关节
        if (it == joint_name_to_index_.end())
        {
            continue;
        }

        const size_t idx = it->second; // 获取关节索引（第二个成员）
        // 依次从消息中读取
        if (i < msg->position.size())
        {
            last_positions_[idx] = static_cast<float>(msg->position[i]);
        }
        if (i < msg->velocity.size())
        {
            last_velocities_[idx] = static_cast<float>(msg->velocity[i]);
        }
        if (i < msg->effort.size())
        {
            last_efforts_[idx] = static_cast<float>(msg->effort[i]);
        }
    }

    if (!is_target_initialized_) {
        target_positions_ = last_positions_;
        is_target_initialized_ = true;
    }

    has_joint_state_ = true;
}


/**
 * @brief 机器人步态控制服务回调函数
 *
 * 处理来自客户端的步态控制请求，执行以下操作：
 * - 在互斥锁保护下，读取当前关节反馈状态并计算 PD 控制力矩
 * - 将目标位置加上零位偏移，得到实际控制目标值
 * - 通过 compute_tau() 计算控制力矩并发布到关节控制器
 * - 释放锁后，将（减去零位偏移的）反馈数据填充到响应中
 * - 若尚未接收到关节状态，返回失败
 *
 * @param request  步态控制请求，包含目标关节位置数组 positions[]
 * @param response 步态控制响应，包含反馈位置、速度、力矩、温度和执行状态
 *
 * @note 采用双阶段设计：
 *   第一阶段在锁内拷贝反馈 + 计算力矩；第二阶段锁外发布命令 + 构建响应，
 *   最小化锁持有时间，提高并发性能。
 * @note 零位偏移用于原点校准：请求命令 + 偏移 = 实际控制值；
 *       反馈值 - 偏移 = 相对原点的位置，供上层使用。
 * @note 目标速度 target_dq 固定为 0.0，即 PD 控制仅对位置误差和当前速度做阻尼。
 * @note 若从未收到过 /joint_states 消息，has_joint_state_ 为 false，返回失败。
 */
void GazeboMotorBridgeNode::on_rob_stride_control(
    const std::shared_ptr<rs_interface::srv::RobStrideMsgs::Request> request,
    std::shared_ptr<rs_interface::srv::RobStrideMsgs::Response> response)
{
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(active_joint_count_);

    std::array<float, kNumMotors> positions;
    std::array<float, kNumMotors> velocities;
    std::array<float, kNumMotors> efforts;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // 储存当前关节反馈状态
        positions = last_positions_;
        velocities = last_velocities_;
        efforts = last_efforts_;

        for (size_t i = 0; i < active_joint_count_; ++i)
        {
            double target_q = request->positions[i] + zero_offsets_[i];
            target_positions_[i] = target_q; // 更新目标位置，供定时器使用
            double current_q = positions[i];
            double current_dq = velocities[i];
            double target_dq = 0.0;
            double tau = compute_tau(target_q, current_q, target_dq, current_dq, kp_[i], kd_[i]);
            cmd.data[i] = tau;
        }
    }

    command_pub_->publish(cmd);
    
    // 构建响应数据
    for (size_t i = 0; i < kNumMotors; ++i)
    {
        response->feedback_positions[i] = positions[i] - zero_offsets_[i];
        response->feedback_velocities[i] = velocities[i];
        response->feedback_torques[i] = efforts[i];
        response->feedback_temperatures[i] = feedback_temperature_;
    }
    if (!has_joint_state_)
    {
        response->success = false;
        response->message = "No joint_states received yet";
        return;
    }
    response->success = true;
    response->message = "Command forwarded to joint_group_position_controller";
}

/**
 * @brief 获取当前关节位置服务回调函数
 * 
 * 处理来自客户端的位置查询请求，完成以下操作：
 * - 读取所有关节的当前位置
 * - 减去零位偏移，返回相对原点的位置值
 * - 返回服务执行状态和错误信息
 * 
 * @param request 获取位置请求（无请求参数）
 * @param response 获取位置响应，包含所有关节的当前位置和执行状态
 * 
 * @note 使用互斥锁保护共享状态，防止与其他线程的数据竞争
 * @note 反馈位置 = 实际位置 - 零位偏移，确保返回相对原点的位置值
 * @note 若未收到过关节状态消息，返回失败状态
 */
void GazeboMotorBridgeNode::on_get_positions(
    const std::shared_ptr<rs_interface::srv::GetPositions::Request>,
    std::shared_ptr<rs_interface::srv::GetPositions::Response> response)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    for (size_t i = 0; i < kNumMotors; ++i)
    {
        response->feedback_positions[i] = last_positions_[i] - zero_offsets_[i];
    }

    response->success = has_joint_state_;
    response->message = has_joint_state_ ? "OK" : "No joint_states received yet";
}

/**
 * @brief 设置零位偏移（原点校准）服务回调函数
 * 
 * 将当前关节位置设定为零点基准，用于原点校准。后续所有位置反馈都将相对于此零点。
 * 该方法执行以下操作：
 * - 验证是否已接收到有效的关节状态消息
 * - 将当前关节位置保存为零位偏移值
 * - 此后的位置命令和反馈都将以此零点作为参考
 * 
 * @param request 设置零位请求（无请求参数）
 * @param response 设置零位响应，包含执行状态和消息
 * 
 * @note 使用互斥锁保护共享状态，防止与其他线程的数据竞争
 * @note 必须先接收到关节状态消息，否则返回失败。这确保了零点基准的有效性
 * @note 仅更新已激活的关节（active_joint_count_），其余关节的偏移保持不变（零）
 * 
 * @example 使用场景：
 *   - 机器人启动时，将当前姿态设为零点基准
 *   - 校准机械零点或重新定义原点坐标系
 *   - 清除之前的位置偏差累积
 */
void GazeboMotorBridgeNode::on_set_zeros(
    const std::shared_ptr<rs_interface::srv::SetZeros::Request>,
    std::shared_ptr<rs_interface::srv::SetZeros::Response> response)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!has_joint_state_)
    {
        response->success = false;
        response->message = "No joint_states received yet";
        return;
    }

    for (size_t i = 0; i < active_joint_count_; ++i)
    {
        zero_offsets_[i] = last_positions_[i];
    }
    response->success = true;
    response->message = "Zero offsets captured from current joint positions";
}

void GazeboMotorBridgeNode::control_loop()
{
    if (!is_target_initialized_) {
        return; // 等待接收到第一个 joint_state
    }

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(active_joint_count_);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t i = 0; i < active_joint_count_; ++i)
        {
            double target_q = target_positions_[i];
            double current_q = last_positions_[i];
            double current_dq = last_velocities_[i];
            double target_dq = 0.0;
            double tau = compute_tau(target_q, current_q, target_dq, current_dq, kp_[i], kd_[i]);
            cmd.data[i] = tau;
        }
    }

    command_pub_->publish(cmd);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeboMotorBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
