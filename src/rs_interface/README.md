# rs_interface

## 概述

自定义 ROS 2 服务接口包，定义 3 个电机控制服务（`.srv`）。这些服务被 `zrobot_bridge`（实机）、`zrobot_gz_sim`（Gazebo 仿真）和 `zrobot_mj_sim`（MuJoCo 仿真）三个桥接节点同时提供，也被 `zrobot_deploy`（FSM 运动控制器）统一调用。通过该接口，控制层无需关心底层是实机还是仿真。

## 依赖

### ROS 2

- `rosidl_default_generators`（服务代码生成）

## 构建

```bash
colcon build --packages-select rs_interface
```

## 服务定义

### RobStrideMsgs.srv — 运控指令

```bash
# 请求：23 个电机目标位置 (rad)
float32[23] positions
---
# 响应：电机反馈数据
float32[23] feedback_positions     # 实际位置 (rad)
float32[23] feedback_velocities    # 实际速度 (rad/s)
float32[23] feedback_torques       # 实际转矩 (Nm)
float32[23] feedback_temperatures  # 实际温度 (°C)
bool success                       # 是否成功
string message                     # 附加信息
```

| 字段                    | 类型          | 说明                          |
| ----------------------- | ------------- | ----------------------------- |
| `positions`             | `float32[23]` | 请求：23 个电机的目标关节位置 |
| `feedback_positions`    | `float32[23]` | 响应：当前实际位置            |
| `feedback_velocities`   | `float32[23]` | 响应：当前速度                |
| `feedback_torques`      | `float32[23]` | 响应：当前转矩                |
| `feedback_temperatures` | `float32[23]` | 响应：当前温度                |

### GetPositions.srv — 读取位置

```bash
---
float32[23] feedback_positions
bool success
string message
```

### SetZeros.srv — 设置零位

```bash
---
bool success
string message
```

## 项目结构

```
rs_interface/
├── CMakeLists.txt
├── package.xml
└── srv/
    ├── RobStrideMsgs.srv
    ├── GetPositions.srv
    └── SetZeros.srv
```

## 技术细节

### 23 电机数组的索引含义

虽然机器人共有 23 个关节，但电机索引 0-22 的排列顺序需要配合 YAML 配置文件中的 `motor_can_ids` 使用。实机桥接节点（`zrobot_bridge`）在 YAML 中定义每个索引对应的 CAN ID，而仿真桥接节点只处理其中 12 个主动关节（腿部关节），其余索引被标记为无效并跳过。

### 使用方式

在其他包的 `package.xml` 中添加依赖：

```xml
<depend>rs_interface</depend>
```

在 C++ 代码中调用服务：

```cpp
#include <rs_interface/srv/rob_stride_msgs.hpp>

auto client = node->create_client<rs_interface::srv::RobStrideMsgs>("/rob_stride_control");
auto request = std::make_shared<rs_interface::srv::RobStrideMsgs::Request>();
request->positions = {0.0f};  // 23 个电机位置
auto result = client->async_send_request(request);
```
