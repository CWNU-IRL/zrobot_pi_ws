#include <chrono>
#include <memory>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "rs_interface/srv/rob_stride_msgs.hpp"

using namespace std::chrono_literals;

class RobStrideClientNode : public rclcpp::Node
{
public:
    RobStrideClientNode() : Node("rob_stride_client_node")
    {
        // 初始化电机位置数组为0
        current_positions_.fill(0.0f);
        
        // 创建服务客户端
        client_ = this->create_client<rs_interface::srv::RobStrideMsgs>(
            "/rob_stride_control");
        
        // 创建定时器，每1秒触发一次
        timer_ = this->create_wall_timer(
            1s, std::bind(&RobStrideClientNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "RobStride客户端节点已启动，每1秒发送一次服务请求");
    }

private:
    void timer_callback()
    {
        // 等待服务可用
        if (!client_->wait_for_service(0s)) {
            RCLCPP_WARN(this->get_logger(), 
                "服务 /motor_controller_node/rob_stride_control 不可用");
            return;
        }

        // 创建请求
        auto request = std::make_shared<rs_interface::srv::RobStrideMsgs::Request>();
        
        // 使用当前存储的位置数据
        for (size_t i = 0; i < 23; i++) {
            request->positions[i] = current_positions_[i];
        }
        
        RCLCPP_INFO(this->get_logger(), "发送服务请求，第一个电机位置: %.2f", 
                    current_positions_[0]);
        
        // 异步发送请求并设置回调处理响应
        auto result_future = client_->async_send_request(
            request,
            std::bind(&RobStrideClientNode::response_callback, this, 
                     std::placeholders::_1));
    }
    
    void response_callback(
        rclcpp::Client<rs_interface::srv::RobStrideMsgs>::SharedFuture future)
    {
        try {
            auto response = future.get();
            
            if (response->success) {
                // 从响应中获取 feedback_positions，每个位置加1
                for (size_t i = 0; i < 23; i++) {
                    current_positions_[i] = response->feedback_positions[i] + 1.0f;
                }
                
                RCLCPP_INFO(this->get_logger(), 
                    "收到响应成功，已更新位置（第一个电机: %.2f -> %.2f）", 
                    response->feedback_positions[0], current_positions_[0]);
            } else {
                RCLCPP_WARN(this->get_logger(), 
                    "服务调用失败: %s", response->message.c_str());
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "处理响应时发生异常: %s", e.what());
        }
    }

    rclcpp::Client<rs_interface::srv::RobStrideMsgs>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::array<float, 23> current_positions_;  // 存储当前23个电机的位置
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobStrideClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
