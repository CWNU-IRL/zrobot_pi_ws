# rs_interface

## 概述

自定义 ROS 2 接口包。控制层与驱动层通过命令流通信：

- `/joint_cmd`（`JointCmd`）：deploy → bridge，reliable / depth 1，只保留最新命令。
- `/joint_state`（`JointState`）：bridge → deploy，best effort / depth 1。
- `/motor_telemetry`（`MotorTelemetry`）：bridge → 监控/录包，best effort / depth 1，默认 10 Hz。
- `/set_zeros`（`SetZeros`）：标定工具 → 实机 bridge，回调只入队，由维护槽执行。

实机 `zrobot_bridge`、`zrobot_gz_sim`、`zrobot_mj_sim` 使用同一套消息契约，deploy 不区分底层是实机还是仿真。

## 依赖

- `rosidl_default_generators`
- `builtin_interfaces`

## 消息定义

### JointCmd.msg — 命令流

```text
string command              # "mit" | "enable" | "disable" | "stop" | "damp" | "set_zero" | "status"
builtin_interfaces/Time stamp
string[] name               # 可空，空表示约定的 0..22 固定顺序
float64[] position
float64[] velocity
float64[] torque
float64[] kp
float64[] kd
```

`mit` 的数组长度统一为 23；`kp/kd` 留空时由 bridge 使用 `motor_config.yaml` 的 `motor_kps/motor_kds`。

### JointState.msg — 周期状态

```text
builtin_interfaces/Time stamp
string[] name
float64[] position
float64[] velocity
float64[] torque
float64[] temperature
bool[] valid
```

### MotorTelemetry.msg — 诊断遥测

```text
builtin_interfaces/Time stamp
string[] name
uint8[] error_code
float64[] feedback_age_ms
bool[] online
bool command_stale
string safety_state         # "disabled" | "active" | "damping" | "preparing" | "maintenance"
```

### SetZeros.srv — 设置机械零位

```text
---
bool success
string message
```

## 项目结构

```text
rs_interface/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── JointCmd.msg
│   ├── JointState.msg
│   └── MotorTelemetry.msg
└── srv/
    └── SetZeros.srv
```

## 使用方式

```xml
<depend>rs_interface</depend>
```

```cpp
#include <rs_interface/msg/joint_cmd.hpp>
#include <rs_interface/msg/joint_state.hpp>

auto pub = node->create_publisher<rs_interface::msg::JointCmd>(
    "/joint_cmd", rclcpp::QoS(1).reliable());
auto sub = node->create_subscription<rs_interface::msg::JointState>(
    "/joint_state", rclcpp::QoS(1).best_effort(),
    [](rs_interface::msg::JointState::SharedPtr msg) { /* ... */ });

rs_interface::msg::JointCmd cmd;
cmd.command = "mit";
cmd.position.assign(23, 0.0);
pub->publish(cmd);
```

## 23 路索引约定

数组长度固定为 23。权威顺序由各节点的 `joint_names` 参数定义，实机与仿真使用同一顺序：前 12 个为腿部关节（L/R 各 6 个），12..22 为备用/未接线通道，仿真未接线关节补 0。
