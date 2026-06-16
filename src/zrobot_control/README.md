# zrobot_control

## 概述

ROS 2 速度指令发布节点，周期性地向 `cmd_vel` 话题发送 `geometry_msgs/msg/Twist` 消息，控制机器人的运动方向和速度。`zrobot_deploy` 的 Locomotion 和 PTLocomotion 状态机订阅 `cmd_vel` 话题，将速度指令作为 RL 策略的观测输入，从而驱动机器人行走。

## 依赖

### ROS 2
- `rclcpp`
- `geometry_msgs`

## 构建

```bash
colcon build --packages-select zrobot_control
```

## 使用

### 默认启动

```bash
ros2 run zrobot_control cmd_vel_publisher
```

默认以 10 Hz 频率发布 `linear.x = 0.4 m/s`（直行前进）。

### 使用参数文件

```bash
ros2 run zrobot_control cmd_vel_publisher --ros-args \
    --params-file src/zrobot_control/config/cmd_vel_params.yaml
```

### 命令行覆盖参数

```bash
ros2 run zrobot_control cmd_vel_publisher --ros-args \
    -p velocity_x:=0.8 -p angular_z:=0.5
```

## ROS 2 接口

### 发布的话题

| 话题名 | 类型 | 频率 | 说明 |
|--------|------|------|------|
| `cmd_vel` | `geometry_msgs/msg/Twist` | 10 Hz（可配） | 机器人运动速度指令 |

### 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `publish_rate` | double | 10.0 | 发布频率 (Hz) |
| `velocity_x` | double | 0.4 | 前向速度 (m/s) |
| `velocity_y` | double | 0.0 | 侧向速度 (m/s) |
| `angular_z` | double | 0.0 | 偏航角速度 (rad/s) |
| `topic_name` | string | `cmd_vel` | 目标话题名 |

## 项目结构

```
zrobot_control/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── cmd_vel_params.yaml
└── src/
    └── cmd_vel_publisher.cpp
```
