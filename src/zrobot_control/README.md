# zrobot_control

## 概述

ROS 2 控制客户端节点，周期性地调用 `/rob_stride_control` 服务向机器人发送 23 个电机位置命令，并根据反馈结果递增位置值（每周期 +1.0 rad），用于测试机器人的基本运动控制链路连通性。

## 依赖

### ROS 2
- `rclcpp`
- `rs_interface`（自定义服务定义包）

## 构建

```bash
colcon build --packages-select rs_interface zrobot_bridge zrobot_control
```

先构建 `rs_interface` 和 `zrobot_bridge`，再构建本包。

## 使用

```bash
# 终端 1：启动桥接服务端
ros2 launch zrobot_bridge motor_controller.launch.py

# 终端 2：启动控制客户端
ros2 run zrobot_control rob_stride_client_node
```

节点启动后每 5 秒向 `/rob_stride_control` 发送一次请求，首次发送全零位置，每次成功后将所有电机位置递增 1.0 rad。

## ROS 2 接口

### 调用的服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/rob_stride_control` | `rs_interface/srv/RobStrideMsgs` | 发送 23 电机位置命令并接收反馈 |

### 行为说明

- 节点名：`rob_stride_client_node`
- 控制周期：5 秒（固定定时器）
- 位置更新策略：`current_positions[i] = feedback_positions[i] + 1.0f`
- 当服务不可用时打印 WARN 日志并跳过当前周期

## 项目结构

```
zrobot_control/
├── CMakeLists.txt
├── package.xml
└── src/
    └── rob_stride_client_node.cpp
```

## 技术细节

该节点是 `zrobot_deploy` 中 FSM 控制系统的简化等价实现，不包含任何状态机或 RL 推理逻辑，仅做周期性位置发送与更新。其主要用途是验证 `zrobot_bridge` 的 `MotorControllerNode` 服务端是否正常工作。
