# IMU Data Node

基于 C++ 实现的 ROS 2 IMU 数据采集节点，支持 WIT 系列 IMU 模块的 TTL、CAN、RS485 三种通信协议。本软件包是 `wit_ros2_imu.py` Python 版本的 C++ 面向对象实现，使用 `libserial` 库进行串口通信。

## 功能特性

- ✅ **多协议支持**：完整兼容 TTL、CAN、RS485 三种通信协议
- ✅ **数据完整解析**：实时解析加速度、角速度、欧拉角、磁力计数据
- ✅ **单位标准化**：自动转换为标准单位（加速度：g，角速度：rad/s，角度：度）
- ✅ **四元数输出**：从欧拉角计算标准四元数表示
- ✅ **双消息发布**：同时发布标准 `sensor_msgs/Imu` 和自定义 `imu_msg/ImuData` 消息
- ✅ **线程安全**：独立线程处理串口数据，避免阻塞 ROS 2 主线程
- ✅ **错误恢复**：完善的异常处理和日志记录机制

## 依赖项

### 系统依赖
- Ubuntu 22.04 / ROS 2 Humble（或更高版本）
- `libserial-dev` 库

### ROS 2 依赖
- `rclcpp`
- `sensor_msgs`
- `imu_msg`（自定义消息包）

## 安装与构建

### 1. 安装系统依赖
```bash
sudo apt update
sudo apt install libserial-dev
```

### 2. 构建软件包
```bash
cd /root/codes/zrobot_pi_ws
colcon build --packages-select imu_data_node
source install/setup.bash
```

## 使用方法

### 基本运行
```bash
ros2 run imu_data_node imu_data_node
```

### 查看发布的数据
```bash
# 查看标准 IMU 消息
ros2 topic echo /imu/data

# 查看包含 RPY 角度的自定义消息
ros2 topic echo /imu/ImuDataWithRPY
```

## 主题说明

| 主题 | 消息类型 | 描述 |
|------|----------|------|
| `/imu/data` | `sensor_msgs/msg/Imu` | 标准 IMU 消息，包含加速度、角速度、四元数 |
| `/imu/ImuDataWithRPY` | `imu_msg/msg/ImuData` | 自定义消息，包含标准 IMU 消息和欧拉角（度） |

## 协议配置

### 协议类型
代码中定义了三种协议类型（`ProtocolType` 枚举）：
- `TTL`：标准串口协议，11字节数据包
- `CAN`：CAN总线协议，8字节数据包  
- `RS485`：Modbus RTU协议，轮询寄存器方式

### 配置修改
如需修改协议配置，请编辑 `src/imu_driver_node.cpp` 文件中的 `main()` 函数：

```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // 参数说明：串口设备、波特率、协议类型
  auto node = std::make_shared<imu_data_node::ImuDriverNode>(
    "/dev/imu_usb",           // 串口设备路径
    2000000,                  // 波特率
    imu_data_node::ProtocolType::TTL  // 协议类型
  );
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### 支持的波特率
- 9600
- 115200  
- 2000000（默认）

如需其他波特率，请在 `openPort()` 方法中添加对应的 `LibSerial::BaudRate` 枚举值。

## 数据解析说明

### 原始数据转换
- **加速度**：原始值 × (16.0 / 32768.0) = 加速度（g）
- **角速度**：原始值 × (2000.0 / 32768.0) × (π/180) = 角速度（rad/s）
- **欧拉角**：原始值 × (180.0 / 32768.0) × (π/180) = 角度（弧度）

### 四元数计算
采用标准 ZYX 旋转顺序（Yaw-Pitch-Roll）计算四元数：
```
q = q_roll * q_pitch * q_yaw
```

## 硬件连接

### 串口设备
默认使用 `/dev/imu_usb` 设备，请根据实际连接情况修改：
- USB转串口设备：通常为 `/dev/ttyUSB0` 或 `/dev/ttyACM0`
- 直接串口：`/dev/ttyS0`（COM1）

### 权限设置
确保当前用户有串口访问权限：
```bash
sudo usermod -a -G dialout $USER
# 或使用临时权限
sudo chmod 666 /dev/ttyUSB0
```

## 故障排除

### 1. 串口打开失败
```
[FATAL] [imu_driver_node]: Failed to open serial port: Bad file descriptor
```
- 检查设备路径是否正确
- 确认用户有串口访问权限
- 确认设备已连接且未被其他程序占用

### 2. 数据解析异常
- 确认 IMU 模块的协议类型设置正确
- 检查波特率是否与 IMU 模块配置一致
- 查看 ROS 2 日志输出的警告信息

### 3. 编译错误
```
CMake Error: Could not find a package configuration file provided by "LibSerial"
```
- 确认已安装 `libserial-dev` 库
- 运行 `sudo apt install libserial-dev`

## 代码结构

```
imu_data_node/
├── CMakeLists.txt              # 构建配置
├── package.xml                 # 包定义文件
├── README.md                   # 本文档
├── include/imu_data_node/
│   └── imu_driver_node.hpp     # 头文件
├── src/
│   └── imu_driver_node.cpp     # 源文件
└── LICENSE                     # 许可证文件
```

## 与 Python 版本对比

| 特性 | Python 版本 | C++ 版本 |
|------|-------------|----------|
| 串口库 | pyserial | libserial |
| 线程模型 | threading.Thread | std::thread |
| 协议支持 | TTL, CAN, RS485 | TTL, CAN, RS485 |
| 数据处理 | 全局变量 | 类成员变量 |
| 错误处理 | print 语句 | ROS 2 日志系统 |
| 性能 | 中等 | 较高 |

## 许可证

本项目采用 Apache 2.0 许可证，详见 [LICENSE](LICENSE) 文件。

## 维护者

- 项目维护：root <callmebill@billw.cn>

## 更新日志

### v1.0.0 (2026-01-22)
- 初始版本发布
- 完整实现 Python 版本的所有功能
- 添加 C++ 面向对象设计和 libserial 支持
- 完善文档和构建系统