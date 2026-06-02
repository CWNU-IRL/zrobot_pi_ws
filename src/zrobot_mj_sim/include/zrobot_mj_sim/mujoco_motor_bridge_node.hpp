#ifndef ZROBOT_MJ_SIM__MUJOCO_MOTOR_BRIDGE_NODE_HPP_
#define ZROBOT_MJ_SIM__MUJOCO_MOTOR_BRIDGE_NODE_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rs_interface/srv/get_positions.hpp"
#include "rs_interface/srv/rob_stride_msgs.hpp"
#include "rs_interface/srv/set_zeros.hpp"

#include <mujoco/mujoco.h>

class MujocoMotorBridgeNode : public rclcpp::Node
{
public:
    MujocoMotorBridgeNode();
    ~MujocoMotorBridgeNode() override;

private:
    static constexpr size_t kNumMotors = 23;

    enum class ControlMode
    {
        kTorquePd,
        kPosition
    };

    struct JointHandle
    {
        int joint_id;
        int qpos_adr;
        int dof_adr;
        int actuator_id;
        bool valid;
    };

    struct SensorHandle
    {
        int id;
        int adr;
        int dim;
        bool valid;
    };

    static std::vector<std::string> default_joint_names();

    void load_model(const std::string &model_path);
    void build_joint_mappings();
    void update_state_from_sim_locked();
    void apply_control_locked();
    void control_loop();

    double compute_tau(
        double target_q,
        double current_q,
        double target_dq,
        double current_dq,
        double kp,
        double kd) const;

    void on_rob_stride_control(
        const std::shared_ptr<rs_interface::srv::RobStrideMsgs::Request> request,
        std::shared_ptr<rs_interface::srv::RobStrideMsgs::Response> response);

    void on_get_positions(
        const std::shared_ptr<rs_interface::srv::GetPositions::Request> request,
        std::shared_ptr<rs_interface::srv::GetPositions::Response> response);

    void on_set_zeros(
        const std::shared_ptr<rs_interface::srv::SetZeros::Request> request,
        std::shared_ptr<rs_interface::srv::SetZeros::Response> response);

    void publish_joint_states(
        const rclcpp::Time &stamp,
        const std::array<float, kNumMotors> &positions,
        const std::array<float, kNumMotors> &velocities,
        const std::array<float, kNumMotors> &efforts);

    void publish_imu(
        const rclcpp::Time &stamp,
        const SensorHandle &orientation,
        const SensorHandle &gyro,
        const SensorHandle &accel);

    void publish_clock(const rclcpp::Time &stamp);

    rclcpp::Time current_sim_time_locked() const;
    SensorHandle find_sensor_handle(const std::string &name) const;

    void start_render_thread();
    void render_loop();
    void render_scene();
    void reset_simulation();

    static void glut_display();
    static void glut_reshape(int width, int height);
    static void glut_mouse(int button, int state, int x, int y);
    static void glut_motion(int x, int y);
    static void glut_keyboard(unsigned char key, int x, int y);

    mjtMouse map_button_to_action(int button, int state) const;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Service<rs_interface::srv::RobStrideMsgs>::SharedPtr rob_stride_service_;
    rclcpp::Service<rs_interface::srv::GetPositions>::SharedPtr get_positions_service_;
    rclcpp::Service<rs_interface::srv::SetZeros>::SharedPtr set_zeros_service_;

    std::vector<std::string> joint_names_;
    size_t active_joint_count_;
    std::vector<double> kp_;
    std::vector<double> kd_;

    std::vector<JointHandle> joint_handles_;

    SensorHandle orientation_sensor_;
    SensorHandle gyro_sensor_;
    SensorHandle accel_sensor_;

    std::string base_frame_;
    std::string imu_frame_;
    double control_frequency_;
    size_t sim_substeps_;
    float feedback_temperature_;
    bool publish_joint_states_;
    bool publish_imu_;
    bool publish_clock_;
    bool has_state_;
    bool is_target_initialized_;

    ControlMode control_mode_;
    std::string control_mode_param_;
    std::string model_path_;
    std::string position_model_path_;

    std::mutex state_mutex_;
    std::array<float, kNumMotors> zero_offsets_;
    std::array<float, kNumMotors> last_positions_;
    std::array<float, kNumMotors> last_velocities_;
    std::array<float, kNumMotors> last_efforts_;
    std::array<float, kNumMotors> target_positions_;

    std::thread render_thread_;
    std::atomic<bool> render_running_{false};
    mjvCamera render_cam_;
    mjvOption render_opt_;
    mjvPerturb render_pert_;
    mutable mjvScene render_scene_;
    mjrContext render_context_;
    int render_width_{1200};
    int render_height_{900};
    int last_mouse_x_{0};
    int last_mouse_y_{0};
    mjtMouse mouse_action_left_{mjMOUSE_ROTATE_V};
    mjtMouse mouse_action_right_{mjMOUSE_ZOOM};
    mjtMouse mouse_action_middle_{mjMOUSE_MOVE_V};
    bool render_initialized_{false};

    static MujocoMotorBridgeNode* render_instance_;

    mjModel *model_;
    mjData *data_;
};

#endif
