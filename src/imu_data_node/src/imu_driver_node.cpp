#include "imu_data_node/imu_driver_node.hpp"

#include <cmath>
#include <chrono>
#include <iostream>
#include <stdexcept>

using namespace std::chrono_literals;

namespace imu_data_node
{

    // RS485 command map
    static const std::map<int, std::vector<uint8_t>> RS485_COMMANDS = {
        {0x34, {0x50, 0x03, 0x00, 0x34, 0x00, 0x03, 0x49, 0x84}}, // acceleration
        {0x37, {0x50, 0x03, 0x00, 0x37, 0x00, 0x03, 0x48, 0x55}}, // angular velocity
        {0x3A, {0x50, 0x03, 0x00, 0x3A, 0x00, 0x03, 0x48, 0xC2}}, // magnetometer
        {0x3D, {0x50, 0x03, 0x00, 0x3D, 0x00, 0x03, 0x49, 0x33}}  // angle
    };

    ImuDriverNode::ImuDriverNode(
        const std::string &port_name,
        int baud_rate,
        ProtocolType protocol)
        : Node("imu_driver_node"),
          port_name_(port_name),
          baud_rate_(baud_rate),
          protocol_(protocol),
          running_(true),
          key_(0),
          rs485_index_(0)
    {
        // Initialize vectors
        acceleration_.resize(3, 0);     // 加速度
        angular_velocity_.resize(3, 0); // 角速度
        angle_degree_.resize(3, 0);     // 角度
        magnetometer_.resize(3, 0);     // 磁力计
        quaternion_.resize(4, 0.0);     // 四元数

        // Create publishers
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        imu_with_rpy_publisher_ = this->create_publisher<imu_msg::msg::ImuData>("imu/ImuDataWithRPY", 10);

        // Start driver thread
        driver_thread_ = std::thread(&ImuDriverNode::driverLoop, this);
    }

    ImuDriverNode::~ImuDriverNode()
    {
        running_ = false;
        if (driver_thread_.joinable())
        {
            driver_thread_.join();
        }
        if (serial_port_.IsOpen())
        {
            serial_port_.Close();
        }
    }

    void ImuDriverNode::driverLoop()
    {
        openPort();
        readData();
    }

    void ImuDriverNode::openPort()
    {
        try
        {
            serial_port_.Open(port_name_);
            // Convert baud rate to enum
            LibSerial::BaudRate baud_enum = LibSerial::BaudRate::BAUD_2000000;
            if (baud_rate_ == 115200)
            {
                baud_enum = LibSerial::BaudRate::BAUD_115200;
            }
            else if (baud_rate_ == 9600)
            {
                baud_enum = LibSerial::BaudRate::BAUD_9600;
            }
            else if (baud_rate_ == 2000000)
            {
                baud_enum = LibSerial::BaudRate::BAUD_2000000;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unsupported baud rate %d, defaulting to 2000000", baud_rate_);
                baud_enum = LibSerial::BaudRate::BAUD_2000000;
            }
            serial_port_.SetBaudRate(baud_enum);
            serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
            // Note: libserial may not support all baud rates, adjust as needed
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully...");
        }
        catch (const LibSerial::OpenFailed &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", e.what());
            throw;
        }
    }

    void ImuDriverNode::rs485RequestData()
    {
        // Cycle through four registers
        auto it = RS485_COMMANDS.begin();
        std::advance(it, rs485_index_);
        const std::vector<uint8_t> &command = it->second;
        serial_port_.Write(command);
        rs485_index_ = (rs485_index_ + 1) % RS485_COMMANDS.size();
    }

    void ImuDriverNode::readData()
    {
        while (rclcpp::ok() && running_)
        {
            try
            {
                size_t bytes_available = serial_port_.GetNumberOfBytesAvailable();
                if (protocol_ == ProtocolType::RS485)
                {
                    rs485RequestData();
                    std::this_thread::sleep_for(20ms); // 50Hz
                }
                if (bytes_available > 0)
                {
                    std::vector<uint8_t> buffer(bytes_available);
                    serial_port_.Read(buffer, bytes_available);
                    for (uint8_t byte : buffer)
                    {
                        bool has_angle = handleSerialData(byte);
                        if (has_angle)
                        {
                            processData();
                            publishData();
                        }
                    }
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error reading serial data: %s", e.what());
                break;
            }
        }
    }

    bool ImuDriverNode::handleSerialData(uint8_t data)
    {
        buffer_[key_] = data;
        key_++;

        // Check header
        if (protocol_ == ProtocolType::TTL || protocol_ == ProtocolType::CAN)
        {
            if (buffer_[0] != 0x55)
            {
                buffer_.clear();
                key_ = 0;
                return false;
            }
        }
        else if (protocol_ == ProtocolType::RS485)
        {
            static const uint8_t MODBUS_ID = 0x50;
            if (buffer_[0] != MODBUS_ID)
            {
                buffer_.clear();
                key_ = 0;
                return false;
            }
        }

        // Check packet length
        if (protocol_ == ProtocolType::TTL || protocol_ == ProtocolType::RS485)
        {
            // TTL and RS485 packet length 11
            if (key_ < 11)
            {
                return false;
            }
        }
        else if (protocol_ == ProtocolType::CAN)
        {
            // CAN packet length 8
            if (key_ < 8)
            {
                return false;
            }
        }

        // Convert buffer to vector
        std::vector<uint8_t> data_buff;
        for (int i = 0; i < key_; ++i)
        {
            data_buff.push_back(buffer_[i]);
        }

        bool angle_flag = false;

        if (protocol_ == ProtocolType::TTL)
        {
            uint8_t data_type = buffer_[1];
            if (data_type == 0x51) // acceleration
            {
                if (checkSum(std::vector<uint8_t>(data_buff.begin(), data_buff.begin() + 10), data_buff[10]))
                {
                    std::vector<uint8_t> raw(data_buff.begin() + 2, data_buff.begin() + 10);
                    acceleration_ = hexToShort(raw, protocol_);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "0x51 Check failure");
                }
            }
            else if (data_type == 0x52) // angular velocity
            {
                if (checkSum(std::vector<uint8_t>(data_buff.begin(), data_buff.begin() + 10), data_buff[10]))
                {
                    std::vector<uint8_t> raw(data_buff.begin() + 2, data_buff.begin() + 10);
                    angular_velocity_ = hexToShort(raw, protocol_);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "0x52 Check failure");
                }
            }
            else if (data_type == 0x53) // angle
            {
                if (checkSum(std::vector<uint8_t>(data_buff.begin(), data_buff.begin() + 10), data_buff[10]))
                {
                    std::vector<uint8_t> raw(data_buff.begin() + 2, data_buff.begin() + 10);
                    angle_degree_ = hexToShort(raw, protocol_);
                    angle_flag = true;
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "0x53 Check failure");
                }
            }
            else if (data_type == 0x54) // magnetometer
            {
                if (checkSum(std::vector<uint8_t>(data_buff.begin(), data_buff.begin() + 10), data_buff[10]))
                {
                    std::vector<uint8_t> raw(data_buff.begin() + 2, data_buff.begin() + 10);
                    magnetometer_ = hexToShort(raw, protocol_);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "0x54 Check failure");
                }
            }
            else
            {
                buffer_.clear();
                key_ = 0;
                return false;
            }
        }
        else if (protocol_ == ProtocolType::CAN)
        {
            uint8_t data_type = buffer_[1];
            std::vector<uint8_t> raw(data_buff.begin() + 2, data_buff.begin() + 8);
            if (data_type == 0x51)
            {
                acceleration_ = hexToShort(raw, protocol_);
            }
            else if (data_type == 0x52)
            {
                angular_velocity_ = hexToShort(raw, protocol_);
            }
            else if (data_type == 0x53)
            {
                angle_degree_ = hexToShort(raw, protocol_);
                angle_flag = true;
            }
            else if (data_type == 0x54)
            {
                magnetometer_ = hexToShort(raw, protocol_);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unknown data type: 0x%02x", data_type);
            }
        }
        else if (protocol_ == ProtocolType::RS485)
        {
            // For RS485, we rely on the order of requests
            static int last_addr = 0x34;
            std::vector<uint8_t> raw(data_buff.begin() + 3, data_buff.begin() + 9);
            std::vector<int16_t> values = hexToShort(raw, protocol_);

            if (last_addr == 0x34)
            {
                acceleration_ = values;
            }
            else if (last_addr == 0x37)
            {
                angular_velocity_ = values;
            }
            else if (last_addr == 0x3A)
            {
                magnetometer_ = values;
            }
            else if (last_addr == 0x3D)
            {
                angle_degree_ = values;
                angle_flag = true;
            }

            // Update next address
            last_addr += 3;
            if (last_addr > 0x3D)
            {
                last_addr = 0x34;
            }
        }

        // Clear buffer
        buffer_.clear();
        key_ = 0;
        return angle_flag;
    }

    void ImuDriverNode::processData()
    {
        // Acceleration scaling
        const double ACCEL_SCALE = 16.0 / 32768.0;
        accel_x_ = acceleration_[0] * ACCEL_SCALE;
        accel_y_ = acceleration_[1] * ACCEL_SCALE;
        accel_z_ = acceleration_[2] * ACCEL_SCALE;

        // Gyro scaling (convert to rad/s)
        const double GYRO_SCALE = 2000.0 / 32768.0;
        gyro_x_ = angular_velocity_[0] * GYRO_SCALE * M_PI / 180.0;
        gyro_y_ = angular_velocity_[1] * GYRO_SCALE * M_PI / 180.0;
        gyro_z_ = angular_velocity_[2] * GYRO_SCALE * M_PI / 180.0;

        // Angle scaling (convert to radians)
        const double ANGLE_SCALE = 180.0 / 32768.0;
        roll_ = angle_degree_[0] * ANGLE_SCALE * M_PI / 180.0;
        pitch_ = angle_degree_[1] * ANGLE_SCALE * M_PI / 180.0;
        yaw_ = angle_degree_[2] * ANGLE_SCALE * M_PI / 180.0;

        // Compute quaternion
        quaternion_ = getQuaternionFromEuler(roll_, pitch_, yaw_);
    }

    void ImuDriverNode::publishData()
    {
        // Create Imu message
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        imu_msg.linear_acceleration.x = accel_x_;
        imu_msg.linear_acceleration.y = accel_y_;
        imu_msg.linear_acceleration.z = accel_z_;

        imu_msg.angular_velocity.x = gyro_x_;
        imu_msg.angular_velocity.y = gyro_y_;
        imu_msg.angular_velocity.z = gyro_z_;

        imu_msg.orientation.x = quaternion_[0];
        imu_msg.orientation.y = quaternion_[1];
        imu_msg.orientation.z = quaternion_[2];
        imu_msg.orientation.w = quaternion_[3];

        // Create custom message with RPY
        auto imu_with_rpy = imu_msg::msg::ImuData();
        imu_with_rpy.header = imu_msg.header;
        imu_with_rpy.imu = imu_msg;
        imu_with_rpy.roll = roll_ * 180.0 / M_PI;
        imu_with_rpy.pitch = pitch_ * 180.0 / M_PI;
        imu_with_rpy.yaw = yaw_ * 180.0 / M_PI;

        // Publish
        imu_publisher_->publish(imu_msg);
        imu_with_rpy_publisher_->publish(imu_with_rpy);

        RCLCPP_INFO(this->get_logger(),
                    "roll: %.2f°, pitch: %.2f°, yaw: %.2f°",
                    roll_ * 180.0 / M_PI,
                    pitch_ * 180.0 / M_PI,
                    yaw_ * 180.0 / M_PI);
    }

    std::vector<int16_t> ImuDriverNode::hexToShort(
        const std::vector<uint8_t> &hex,
        ProtocolType protocol)
    {
        std::vector<int16_t> result(3, 0);
        if (protocol == ProtocolType::TTL)
        {
            if (hex.size() != 8)
            {
                return result;
            }
            // TTL uses little-endian 16-bit values
            for (size_t i = 0; i < 3; ++i)
            {
                uint16_t low = hex[i * 2];
                uint16_t high = hex[i * 2 + 1];
                int16_t value = static_cast<int16_t>((high << 8) | low);
                result[i] = value;
            }
        }
        else if (protocol == ProtocolType::CAN)
        {
            if (hex.size() != 6)
            {
                return result;
            }
            // CAN: each 16-bit value split across two bytes in little-endian?
            // According to Python code: data1 = (Hex[1] << 8) | Hex[0]
            for (size_t i = 0; i < 3; ++i)
            {
                uint16_t low = hex[i * 2];
                uint16_t high = hex[i * 2 + 1];
                int16_t value = static_cast<int16_t>((high << 8) | low);
                // Convert to signed 16-bit
                result[i] = value;
            }
        }
        else if (protocol == ProtocolType::RS485)
        {
            if (hex.size() != 6)
            {
                return result;
            }
            // RS485: big-endian? Python code: data1 = (Hex[0] << 8) | Hex[1]
            for (size_t i = 0; i < 3; ++i)
            {
                uint16_t high = hex[i * 2];
                uint16_t low = hex[i * 2 + 1];
                int16_t value = static_cast<int16_t>((high << 8) | low);
                result[i] = value;
            }
        }
        return result;
    }

    bool ImuDriverNode::checkSum(const std::vector<uint8_t> &data, uint8_t checksum)
    {
        uint8_t sum = 0;
        for (uint8_t byte : data)
        {
            sum += byte;
        }
        return (sum & 0xFF) == checksum;
    }

    std::vector<double> ImuDriverNode::getQuaternionFromEuler(
        double roll, double pitch, double yaw)
    {
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        std::vector<double> q(4);
        q[0] = sr * cp * cy - cr * sp * sy; // x
        q[1] = cr * sp * cy + sr * cp * sy; // y
        q[2] = cr * cp * sy - sr * sp * cy; // z
        q[3] = cr * cp * cy + sr * sp * sy; // w

        return q;
    }

} // namespace imu_data_node

#include "rclcpp/utilities.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Use default protocol TTL, baud rate 2000000, port /dev/imu_usb
    auto node = std::make_shared<imu_data_node::ImuDriverNode>(
        "/dev/imu_usb",
        2000000,
        imu_data_node::ProtocolType::TTL);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}