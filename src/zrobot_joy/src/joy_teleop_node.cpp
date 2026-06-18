#include <chrono>
#include <cstdio>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyTeleopNode : public rclcpp::Node
{
public:
    JoyTeleopNode()
        : Node("joy_teleop_node")
    {
        tty_ = fopen("/dev/tty", "w");
        tty_ostream_ = tty_ ? nullptr : &std::cout;
        declare_parameter("joy_topic", "/joy");
        declare_parameter("cmd_topic", "cmd_vel");
        declare_parameter("publish_rate", 50.0);
        declare_parameter("enable_button", 4);
        declare_parameter("enable_turbo_button", 5);
        declare_parameter("axis_linear_x", 1);
        declare_parameter("axis_linear_y", 0);
        declare_parameter("axis_angular", 3);
        declare_parameter("scale_linear_x", 0.7);
        declare_parameter("scale_linear_y", 0.5);
        declare_parameter("scale_angular", 0.8);
        declare_parameter("scale_turbo", 1.5);
        declare_parameter("deadzone", 0.1);

        joy_topic_ = get_parameter("joy_topic").as_string();
        cmd_topic_ = get_parameter("cmd_topic").as_string();
        enable_button_ = get_parameter("enable_button").as_int();
        turbo_button_ = get_parameter("enable_turbo_button").as_int();
        axis_lin_x_ = get_parameter("axis_linear_x").as_int();
        axis_lin_y_ = get_parameter("axis_linear_y").as_int();
        axis_ang_ = get_parameter("axis_angular").as_int();
        scale_lin_x_ = get_parameter("scale_linear_x").as_double();
        scale_lin_y_ = get_parameter("scale_linear_y").as_double();
        scale_ang_ = get_parameter("scale_angular").as_double();
        scale_turbo_ = get_parameter("scale_turbo").as_double();
        deadzone_ = get_parameter("deadzone").as_double();

        double rate = get_parameter("publish_rate").as_double();
        if (rate <= 0.0) {
            rate = 50.0;
        }
        auto period = std::chrono::duration<double>(1.0 / rate);

        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            joy_topic_, rclcpp::SensorDataQoS(),
            std::bind(&JoyTeleopNode::joyCallback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);

        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&JoyTeleopNode::publishTimer, this));

        printHelp();
    }

    void printHelp()
    {
        std::string sep(50, '=');
        std::ostringstream oss;
        oss << "\n" << sep << "\n"
            << "  Xbox Controller Teleop\n\n"
            << "    Hold LB to enable movement.\n"
            << "    Left Stick  - Forward/Back & Strafe\n"
            << "    Right Stick - Turn\n"
            << "    Hold RB     - Turbo (speed x"
            << std::fixed << std::setprecision(1) << scale_turbo_ << ")\n\n"
            << "  Output: " << cmd_topic_ << " @ "
            << get_parameter("publish_rate").as_double() << " Hz\n"
            << sep << "\n" << std::endl;
        writeOutput(oss.str());
    }

    void writeOutput(const std::string& text)
    {
        if (tty_) {
            fprintf(tty_, "%s", text.c_str());
            fflush(tty_);
        } else {
            std::cout << text << std::flush;
        }
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist = geometry_msgs::msg::Twist();
        last_joy_time_ = now();

        bool enabled = buttonPressed(msg->buttons, enable_button_);
        if (!enabled) {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_twist_ = twist;
            enabled_active_ = false;
            return;
        }
        enabled_active_ = true;

        bool turbo = buttonPressed(msg->buttons, turbo_button_);
        double mul_lin = turbo ? scale_turbo_ : 1.0;
        double mul_ang = turbo ? scale_turbo_ : 1.0;

        double raw_x = 0.0, raw_y = 0.0, raw_z = 0.0;

        if (axis_lin_x_ >= 0 && axis_lin_x_ < static_cast<int>(msg->axes.size())) {
            double val = msg->axes[axis_lin_x_];
            raw_x = val;
            if (std::abs(val) < deadzone_) val = 0.0;
            twist.linear.x = val * scale_lin_x_ * mul_lin;
        }

        if (axis_lin_y_ >= 0 && axis_lin_y_ < static_cast<int>(msg->axes.size())) {
            double val = msg->axes[axis_lin_y_];
            raw_y = val;
            if (std::abs(val) < deadzone_) val = 0.0;
            twist.linear.y = val * scale_lin_y_ * mul_lin;
        }

        if (axis_ang_ >= 0 && axis_ang_ < static_cast<int>(msg->axes.size())) {
            double val = msg->axes[axis_ang_];
            raw_z = val;
            if (std::abs(val) < deadzone_) val = 0.0;
            twist.angular.z = val * scale_ang_ * mul_ang;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        latest_twist_ = twist;
        raw_ax_ = raw_x;
        raw_ay_ = raw_y;
        raw_az_ = raw_z;
    }

    void publishTimer()
    {
        geometry_msgs::msg::Twist twist;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if ((now() - last_joy_time_).seconds() > 1.0) {
                latest_twist_ = geometry_msgs::msg::Twist();
            }
            twist = latest_twist_;
        }
        cmd_pub_->publish(twist);
        displayStatus(twist);
    }

    void displayStatus(const geometry_msgs::msg::Twist& msg)
    {
        double raw_x, raw_z;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            raw_x = raw_ax_;
            raw_z = raw_az_;
        }

        if (msg.linear.x == last_lin_x_ &&
            msg.linear.y == last_lin_y_ &&
            msg.angular.z == last_ang_z_) {
            return;
        }
        last_lin_x_ = msg.linear.x;
        last_lin_y_ = msg.linear.y;
        last_ang_z_ = msg.angular.z;

        const char *tag = enabled_active_ ? "[LB]" : "[  ]";

        std::ostringstream oss;
        oss << "\033[2K\r" << tag
            << " stick[A" << axis_lin_x_ << ":"
            << std::setw(5) << std::fixed << std::setprecision(2) << raw_x
            << " A" << axis_ang_ << ":"
            << std::setw(5) << raw_z << "]"
            << "  cmd[vx:" << std::setw(6) << std::setprecision(2)
            << msg.linear.x
            << " vy:" << std::setw(6) << msg.linear.y
            << " vz:" << std::setw(6) << msg.angular.z << "]";
        writeOutput(oss.str());
    }

    static bool buttonPressed(
        const std::vector<int> &buttons, int index)
    {
        return index >= 0 && index < static_cast<int>(buttons.size()) &&
               buttons[index] == 1;
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string joy_topic_;
    std::string cmd_topic_;
    int enable_button_;
    int turbo_button_;
    int axis_lin_x_;
    int axis_lin_y_;
    int axis_ang_;
    double scale_lin_x_;
    double scale_lin_y_;
    double scale_ang_;
    double scale_turbo_;
    double deadzone_;

    std::mutex mutex_;
    geometry_msgs::msg::Twist latest_twist_;
    double raw_ax_ = 0.0;
    double raw_ay_ = 0.0;
    double raw_az_ = 0.0;
    bool enabled_active_ = false;
    rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};

    double last_lin_x_ = -1.0;
    double last_lin_y_ = -1.0;
    double last_ang_z_ = -1.0;

    FILE *tty_ = nullptr;
    std::ostream *tty_ostream_ = nullptr;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyTeleopNode>();
    rclcpp::spin(node);
    node->writeOutput("\n");
    rclcpp::shutdown();
    return 0;
}
