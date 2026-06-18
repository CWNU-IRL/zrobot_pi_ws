# imu_msg

## 概述

自定义 ROS 2 消息包，定义 `ImuData.msg` 消息类型。该消息在标准 `sensor_msgs/Imu` 的基础上扩展了欧拉角（Roll / Pitch / Yaw），适用于需要直接使用欧拉角的控制节点（如 `zrobot_deploy` 的 Locomotion 状态机）。

## 依赖

### ROS 2

- `rosidl_default_generators`（消息代码生成）
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`

## 构建

```bash
colcon build --packages-select imu_msg
```

## 消息定义

### ImuData.msg

```
std_msgs/Header header     # 时间戳与坐标系 ID
sensor_msgs/Imu imu        # 标准 IMU 消息（含四元数方向、角速度、线加速度协方差）
float64 roll               # 横滚角 (rad)
float64 pitch              # 俯仰角 (rad)
float64 yaw                # 偏航角 (rad)
```

- `imu` 字段嵌套标准 `sensor_msgs/Imu`，提供四元数形式的方向信息
- `roll` / `pitch` / `yaw` 为额外添加的欧拉角，方便需要角度信息的节点直接使用

## 项目结构

```
imu_msg/
├── CMakeLists.txt
├── package.xml
└── msg/
    └── ImuData.msg
```

## 技术细节

### 使用方式

在其他包的 `package.xml` 中添加依赖：

```xml
<depend>imu_msg</depend>
```

在 C++ 代码中使用：

```cpp
#include <imu_msg/msg/imu_data.hpp>

auto msg = imu_msg::msg::ImuData();
double roll = msg.roll;
double pitch = msg.pitch;
double yaw = msg.yaw;

// 访问嵌套的标准 IMU 数据
double orientation_w = msg.imu.orientation.w;
```
