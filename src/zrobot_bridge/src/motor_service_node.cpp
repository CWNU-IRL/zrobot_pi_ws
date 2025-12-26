#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "zrobot_bridge/motor_cfg.h"
#include "zrobot_bridge/srv/batch_motor_position.hpp"

namespace {
constexpr size_t kMotorCount = 23;

std::array<uint8_t, kMotorCount> parse_motor_ids_param(const rclcpp::Node& node)
{
    std::array<uint8_t, kMotorCount> ids{};

    std::vector<int64_t> raw;
    if (!node.get_parameter("motor_ids", raw)) {
        throw std::runtime_error("Missing required parameter: motor_ids (length 23)");
    }
    if (raw.size() != kMotorCount) {
        throw std::runtime_error("Parameter motor_ids must have length 23");
    }

    for (size_t i = 0; i < kMotorCount; ++i) {
        if (raw[i] < 0 || raw[i] > 255) {
            throw std::runtime_error("Parameter motor_ids must be in [0,255]");
        }
        ids[i] = static_cast<uint8_t>(raw[i]);
    }
    return ids;
}

} // namespace

class MotorServiceNode final : public rclcpp::Node
{
public:
    MotorServiceNode()
        : rclcpp::Node("zrobot_motor_service")
    {
        declare_parameter<std::string>("can_interface", "can0");
        declare_parameter<int>("master_id", 0x01);
        declare_parameter<int>("actuator_type", 0);
        declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>{});

        // 提前解析一次，参数不对直接报错，避免运行时才发现
        motor_ids_ = parse_motor_ids_param(*this);

        service_ = create_service<zrobot_bridge::srv::BatchMotorPosition>(
            "batch_motor_position",
            std::bind(
                &MotorServiceNode::on_service,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Service ready: /batch_motor_position");
    }

private:
    std::shared_ptr<RobStrideMotor> get_or_create_motor(uint8_t motor_id)
    {
        const auto it = motors_.find(motor_id);
        if (it != motors_.end()) {
            return it->second;
        }

        const auto iface = get_parameter("can_interface").as_string();
        const auto master_id_param = get_parameter("master_id").as_int();
        const auto actuator_type = get_parameter("actuator_type").as_int();

        if (master_id_param < 0 || master_id_param > 255) {
            throw std::runtime_error("Parameter master_id must be in [0,255]");
        }

        auto motor = std::make_shared<RobStrideMotor>(
            iface,
            static_cast<uint8_t>(master_id_param),
            motor_id,
            actuator_type);

        // 让 run_mode/pattern 尽快变成有效值，并确保可用
        try {
            motor->Get_RobStrite_Motor_parameter(0x7005);
        } catch (...) {
            // 忽略：有些固件可能不支持/或此时没回包
        }
        try {
            motor->enable_motor();
        } catch (...) {
            // 忽略：服务调用时再兜底
        }

        motors_.emplace(motor_id, motor);
        return motor;
    }

    void on_service(
        const std::shared_ptr<zrobot_bridge::srv::BatchMotorPosition::Request> req,
        std::shared_ptr<zrobot_bridge::srv::BatchMotorPosition::Response> res)
    {
        std::scoped_lock<std::mutex> lock(service_mutex_);

        try {
            for (size_t i = 0; i < kMotorCount; ++i) {
                const uint8_t motor_id = motor_ids_[i];
                auto motor = get_or_create_motor(motor_id);

                // 运控模式：torque=0, velocity=0，位置由positions给出
                const auto [p, v, t, temp] = motor->send_motion_command(
                    0.0f,
                    req->positions[i],
                    0.0f,
                    0.5f,
                    0.1f);

                res->feedback_positions[i] = p;
                res->feedback_velocities[i] = v;
                res->feedback_torques[i] = t;
                res->feedback_temperatures[i] = temp;
            }

            res->success = true;
            res->message = "ok";
        } catch (const std::exception& e) {
            res->success = false;
            res->message = e.what();
            RCLCPP_ERROR(get_logger(), "Service failed: %s", e.what());
        }
    }

private:
    std::mutex service_mutex_;
    std::array<uint8_t, kMotorCount> motor_ids_{};

    std::unordered_map<uint8_t, std::shared_ptr<RobStrideMotor>> motors_;

    rclcpp::Service<zrobot_bridge::srv::BatchMotorPosition>::SharedPtr service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorServiceNode>());
    rclcpp::shutdown();
    return 0;
}
