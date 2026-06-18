#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop()
        : Node("cmd_vel_publisher")
    {
        declare_parameter("publish_rate", 10.0);
        declare_parameter("topic_name", "cmd_vel");
        declare_parameter("linear_speed", 0.5);
        declare_parameter("angular_speed", 0.5);

        double rate = get_parameter("publish_rate").as_double();
        if (rate <= 0.0) {
            RCLCPP_WARN(get_logger(), "publish_rate <= 0, defaulting to 10.0 Hz");
            rate = 10.0;
        }
        publish_period_ = std::chrono::duration<double>(1.0 / rate);

        linear_speed_ = get_parameter("linear_speed").as_double();
        angular_speed_ = get_parameter("angular_speed").as_double();

        pub_ = create_publisher<geometry_msgs::msg::Twist>(
            get_parameter("topic_name").as_string(), 10);

        printHelp();
    }

    void printHelp()
    {
        std::string sep(50, '=');
        std::cout << "\n" << sep << "\n"
                  << "  Keyboard Teleop\n\n"
                  << "    W  - Forward\n"
                  << "    S  - Backward\n"
                  << "    A  - Left turn\n"
                  << "    D  - Right turn\n"
                  << "    X  - Force stop\n"
                  << "    Q  - Quit\n\n"
                  << "  Hold key to move, release to stop.\n"
                  << "  Multiple keys for combined motion (e.g. W+A).\n"
                  << sep << "\n"
                  << "  Topic: " << get_parameter("topic_name").as_string()
                  << ", rate: " << get_parameter("publish_rate").as_double() << " Hz\n"
                  << "  linear_speed: " << linear_speed_ << " m/s"
                  << ", angular_speed: " << angular_speed_ << " rad/s\n"
                  << sep << "\n" << std::endl;
    }

    void processKeyboardInput()
    {
        char c;
        bool any_key = false;
        while (read(STDIN_FILENO, &c, 1) > 0) {
            any_key = true;
            switch (c) {
            case 'w': case 'W': w_active_ = true; break;
            case 's': case 'S': s_active_ = true; break;
            case 'a': case 'A': a_active_ = true; break;
            case 'd': case 'D': d_active_ = true; break;
            case 'x': case 'X':
                w_active_ = a_active_ = s_active_ = d_active_ = false;
                break;
            case 'q': case 'Q':
                quit_ = true;
                return;
            default:
                break;
            }
        }
        if (any_key) {
            last_key_time_ = now();
        }
        if (now() - last_key_time_ > key_timeout_) {
            w_active_ = a_active_ = s_active_ = d_active_ = false;
        }
    }

    void publishCommand()
    {
        auto msg = geometry_msgs::msg::Twist();
        if (w_active_)  msg.linear.x  =  linear_speed_;
        if (s_active_)  msg.linear.x  = -linear_speed_;
        if (a_active_)  msg.angular.z =  angular_speed_;
        if (d_active_)  msg.angular.z = -angular_speed_;
        pub_->publish(msg);
        displayStatus(msg);
    }

    void displayStatus(const geometry_msgs::msg::Twist& msg)
    {
        if (msg.linear.x == last_linear_x_ &&
            msg.linear.y == last_linear_y_ &&
            msg.angular.z == last_angular_z_) {
            return;
        }
        last_linear_x_ = msg.linear.x;
        last_linear_y_ = msg.linear.y;
        last_angular_z_ = msg.angular.z;

        std::ostringstream oss;
        oss << "\033[2K\r"
            << "[vx:" << std::setw(7) << std::fixed << std::setprecision(3)
            << msg.linear.x
            << "  vy:" << std::setw(7) << msg.linear.y
            << "  vz:" << std::setw(7) << msg.angular.z << "]";
        std::cout << oss.str() << std::flush;
    }

    bool shouldQuit() const { return quit_; }
    double publishRate() const { return 1.0 / publish_period_.count(); }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    std::chrono::duration<double> publish_period_;
    double linear_speed_;
    double angular_speed_;

    static constexpr auto key_timeout_ = std::chrono::milliseconds(150);

    bool w_active_ = false;
    bool a_active_ = false;
    bool s_active_ = false;
    bool d_active_ = false;
    bool quit_ = false;
    rclcpp::Time last_key_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    double last_linear_x_ = -1.0;
    double last_linear_y_ = -1.0;
    double last_angular_z_ = -1.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    struct termios orig_term;
    tcgetattr(STDIN_FILENO, &orig_term);
    struct termios raw = orig_term;
    raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    int old_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, old_flags | O_NONBLOCK);

    auto node = std::make_shared<KeyboardTeleop>();

    rclcpp::WallRate rate(node->publishRate());
    while (rclcpp::ok() && !node->shouldQuit()) {
        node->processKeyboardInput();
        node->publishCommand();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &orig_term);
    fcntl(STDIN_FILENO, F_SETFL, old_flags);

    std::cout << "\n";
    rclcpp::shutdown();
    return 0;
}
