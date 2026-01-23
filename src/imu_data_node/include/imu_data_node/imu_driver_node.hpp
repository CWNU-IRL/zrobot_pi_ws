#ifndef IMU_DATA_NODE__IMU_DRIVER_NODE_HPP_
#define IMU_DATA_NODE__IMU_DRIVER_NODE_HPP_

#include <memory>
#include <thread>
#include <map>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "imu_msg/msg/imu_data.hpp"
#include "libserial/SerialPort.h"

namespace imu_data_node
{

    enum class ProtocolType
    {
        TTL = 1,
        CAN = 2,
        RS485 = 3
    };

    class ImuDriverNode : public rclcpp::Node
    {
    public:
        explicit ImuDriverNode(
            const std::string &port_name = "/dev/imu_usb",
            int baud_rate = 2000000,
            ProtocolType protocol = ProtocolType::TTL);
        ~ImuDriverNode();

    private:
        // Serial port
        LibSerial::SerialPort serial_port_;
        std::string port_name_;
        int baud_rate_;
        ProtocolType protocol_;

        // Thread
        std::thread driver_thread_;
        bool running_;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        rclcpp::Publisher<imu_msg::msg::ImuData>::SharedPtr imu_with_rpy_publisher_;

        // Current IMU data
        std::vector<int16_t> acceleration_;     // raw
        std::vector<int16_t> angular_velocity_; // raw
        std::vector<int16_t> angle_degree_;     // raw
        std::vector<int16_t> magnetometer_;     // raw

        // Processed data
        double accel_x_, accel_y_, accel_z_;
        double gyro_x_, gyro_y_, gyro_z_;
        double roll_, pitch_, yaw_;
        std::vector<double> quaternion_;

        // Serial buffer
        std::map<int, uint8_t> buffer_;
        int key_;

        // RS485 state
        int rs485_index_;

        // Methods
        void driverLoop();
        void openPort();
        void rs485RequestData();
        void readData();
        bool handleSerialData(uint8_t data);
        void processData();
        void publishData();

        // Helper functions
        static std::vector<int16_t> hexToShort(const std::vector<uint8_t> &hex, ProtocolType protocol);
        static bool checkSum(const std::vector<uint8_t> &data, uint8_t checksum);
        static std::vector<double> getQuaternionFromEuler(double roll, double pitch, double yaw);
    };

} // namespace imu_data_node

#endif // IMU_DATA_NODE__IMU_DRIVER_NODE_HPP_