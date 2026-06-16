#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class CmdVelPublisher : public rclcpp::Node
{
public:
    CmdVelPublisher()
        : Node("cmd_vel_publisher")
    {
        declare_parameter("publish_rate", 10.0);
        declare_parameter("velocity_x", 0.4);
        declare_parameter("velocity_y", 0.0);
        declare_parameter("angular_z", 0.0);
        declare_parameter("topic_name", "cmd_vel");

        double rate = get_parameter("publish_rate").as_double();
        if (rate <= 0.0) {
            RCLCPP_WARN(get_logger(), "publish_rate <= 0, defaulting to 10.0 Hz");
            rate = 10.0;
        }

        const auto period = std::chrono::duration<double>(1.0 / rate);
        pub_ = create_publisher<geometry_msgs::msg::Twist>(
            get_parameter("topic_name").as_string(), 10);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&CmdVelPublisher::publish, this));

        RCLCPP_INFO(get_logger(),
            "CmdVel publisher ready. Topic: %s, rate: %.1f Hz, vx=%.2f m/s",
            get_parameter("topic_name").as_string().c_str(),
            rate,
            get_parameter("velocity_x").as_double());
    }

private:
    void publish()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = get_parameter("velocity_x").as_double();
        msg.linear.y = get_parameter("velocity_y").as_double();
        msg.angular.z = get_parameter("angular_z").as_double();
        pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelPublisher>());
    rclcpp::shutdown();
    return 0;
}
