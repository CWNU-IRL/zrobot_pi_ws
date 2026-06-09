# imu_msg

## 概述

自定义 ROS 2 消息包，定义 `ImuData.msg`。该消息在标准 `sensor_msgs/Imu` 基础上扩展了欧拉角（roll/pitch/yaw）字段，方便直接获取姿态角而无需自行做四元数转欧拉角计算。

## 依赖

### ROS 2
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `rosidl_default_generators`

## 构建

```bash
colcon build --packages-select imu_msg
```

## 使用

下游包在 `package.xml` 和 `CMakeLists.txt` 中依赖此包后即可引入消息类型：

```cpp
#include "imu_msg/msg/imu_data.hpp"
```

## 消息定义

```
std_msgs/Header header
sensor_msgs/Imu imu
float64 roll
float64 pitch
float64 yaw
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 时间戳与坐标系 ID |
| `imu` | `sensor_msgs/Imu` | 标准 IMU 消息（四元数、角速度、线加速度） |
| `roll` | `float64` | 横滚角 |
| `pitch` | `float64` | 俯仰角 |
| `yaw` | `float64` | 偏航角 |

## 项目结构

```
imu_msg/
├── CMakeLists.txt
├── package.xml
└── msg/
    └── ImuData.msg
```
