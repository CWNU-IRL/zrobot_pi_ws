#ifndef ZROBOT_MJ_SIM__MUJOCO_MOTOR_BRIDGE_NODE_HPP_
#define ZROBOT_MJ_SIM__MUJOCO_MOTOR_BRIDGE_NODE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rs_interface/msg/joint_cmd.hpp"
#include "rs_interface/msg/joint_state.hpp"
#include "rs_interface/msg/motor_telemetry.hpp"

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

    void on_joint_cmd(const rs_interface::msg::JointCmd::SharedPtr msg);

    void publish_contract_state();

    void publish_telemetry();

    const char* safety_state_string() const;

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

    rclcpp::Time current_sim_time_locked();
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
    rclcpp::Publisher<rs_interface::msg::JointState>::SharedPtr contract_state_pub_;
    rclcpp::Publisher<rs_interface::msg::MotorTelemetry>::SharedPtr telemetry_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::Subscription<rs_interface::msg::JointCmd>::SharedPtr joint_cmd_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, size_t> joint_name_to_index_;
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

    // /joint_cmd 命令缓存（位置为相对零位）
    bool enabled_;
    bool damping_mode_;
    bool has_mit_command_;
    bool command_stale_;
    std::chrono::steady_clock::time_point last_command_time_;
    double command_timeout_ms_;
    double damping_kd_;
    double state_publish_hz_;
    uint64_t state_publish_divider_;
    uint64_t state_publish_counter_;
    double telemetry_hz_;
    uint64_t telemetry_divider_;
    uint64_t telemetry_counter_;

    ControlMode control_mode_;
    std::string control_mode_param_;
    std::string model_path_;
    std::string position_model_path_;

    std::mutex state_mutex_;
    std::array<float, kNumMotors> zero_offsets_;
    std::array<float, kNumMotors> last_positions_;
    std::array<float, kNumMotors> last_velocities_;
    std::array<float, kNumMotors> last_efforts_;
    std::array<float, kNumMotors> target_positions_{};
    std::array<float, kNumMotors> target_velocities_{};
    std::array<float, kNumMotors> target_torques_{};
    std::array<float, kNumMotors> target_kps_{};
    std::array<float, kNumMotors> target_kds_{};

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
