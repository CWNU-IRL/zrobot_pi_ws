# imu_data_node

## 概述

C++ ROS 2 节点，通过串口读取 WIT 系列 IMU 模块数据，支持 TTL、CAN 和 RS485 三种协议。解析加速度、角速度、欧拉角和磁力计原始数据，经单位换算后以 `sensor_msgs/Imu`（`/imu/data`）和自定义 `imu_msg/ImuData`（`/imu/ImuDataWithRPY`）两种消息格式发布，频率约 50 Hz。

## 依赖

### 系统依赖

- `libserial-dev`（串口通信库）

### ROS 2

- `rclcpp`
- `sensor_msgs`
- `imu_msg`（自定义消息包）

## 构建

```bash
sudo apt install libserial-dev
colcon build --packages-select imu_msg imu_data_node
```

## 使用

### 启动

当前配置为编译期硬编码，默认值：端口 `/dev/imu_usb`、波特率 2000000、协议类型 TTL。

```bash
ros2 run imu_data_node imu_data_node
```

如需修改串口路径或协议类型，编辑 `src/imu_driver_node.cpp` 中的 `main()` 函数后重新编译。

### 查看数据

```bash
# 标准 IMU 消息（四元数 + 角速度 + 加速度）
ros2 topic echo /imu/data

# 含欧拉角的扩展消息
ros2 topic echo /imu/ImuDataWithRPY
```

## ROS 2 接口

### 发布的话题

| 话题名                | 类型                  | 频率   | 说明                                          |
| --------------------- | --------------------- | ------ | --------------------------------------------- |
| `/imu/data`           | `sensor_msgs/msg/Imu` | ~50 Hz | 标准 IMU 消息（四元数方向、角速度、线加速度） |
| `/imu/ImuDataWithRPY` | `imu_msg/msg/ImuData` | ~50 Hz | 扩展消息，包含欧拉角（rad）                   |

## 项目结构

```
imu_data_node/
├── CMakeLists.txt
├── package.xml
├── include/imu_data_node/
│   └── imu_driver_node.hpp
└── src/
    └── imu_driver_node.cpp
```

## 技术细节

### 数据流

```
IMU (串口) → libserial 读取 → 协议解析 → 单位换算 → 话题发布
```

### 协议对比

| 协议  | 帧长    | 校验方式    | 字节序 | 数据帧类型                                             |
| ----- | ------- | ----------- | ------ | ------------------------------------------------------ |
| TTL   | 11 字节 | 累加和校验  | 小端   | 0x51 = 加速度, 0x52 = 角速度, 0x53 = 角度, 0x54 = 磁场 |
| CAN   | 8 字节  | 无校验      | 小端   | 同上数据类型                                           |
| RS485 | 11 字节 | Modbus 校验 | 大端   | 50 Hz 轮询 4 个寄存器（0x34 / 0x37 / 0x3A / 0x3D）     |

### 数据缩放公式

| 物理量 | 原始值范围     | 缩放公式                           | 单位  |
| ------ | -------------- | ---------------------------------- | ----- |
| 加速度 | -32768 ~ 32767 | `raw * 16.0 / 32768.0`             | g     |
| 角速度 | -32768 ~ 32767 | `raw * 2000.0 / 32768.0 * π / 180` | rad/s |
| 欧拉角 | -32768 ~ 32767 | `raw * 180.0 / 32768.0 * π / 180`  | rad   |

### 实现要点

- `ImuDriverNode` 在构造函数中启动独立 `std::thread` 运行 `driverLoop()`，串口读取和协议解析在后台线程中完成，不阻塞 ROS 2 主事件循环
- 状态机解析器 `handleSerialData()` 按协议逐字节解析，当解析到完整角度数据帧时触发 `processData()` → `publishData()`
- 欧拉角到四元数的转换采用 ZYX（Yaw-Pitch-Roll）顺序：`q = q_yaw * q_pitch * q_roll`
- 仅当成功接收到完整的角度数据帧时才发布消息，确保数据完整性
