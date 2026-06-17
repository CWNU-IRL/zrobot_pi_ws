# zrobot_control

## 概述

ROS 2 键盘遥控节点，通过键盘实时控制机器人运动，向 `cmd_vel` 话题发送 `geometry_msgs/msg/Twist` 消息。按下按键时发出速度指令，松开按键后自动停止。`zrobot_deploy` 的 Locomotion 和 PTLocomotion 状态机订阅 `cmd_vel` 话题，将速度指令作为 RL 策略的观测输入，从而驱动机器人行走。

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

### 使用参数文件

```bash
ros2 run zrobot_control cmd_vel_publisher --ros-args \
    --params-file src/zrobot_control/config/cmd_vel_params.yaml
```

### 命令行覆盖参数

```bash
ros2 run zrobot_control cmd_vel_publisher --ros-args \
    -p linear_speed:=0.8 -p angular_speed:=0.5
```

## 按键映射

| 按键 | 行为                               |
|------|------------------------------------|
| `W`  | 前进 (linear.x = +linear_speed)    |
| `S`  | 后退 (linear.x = -linear_speed)    |
| `A`  | 左转 (angular.z = +angular_speed)  |
| `D`  | 右转 (angular.z = -angular_speed)  |
| `X`  | 强制停止                           |
| `Q`  | 退出节点                           |

- **按下移动 / 松开停止**：按住按键持续发送非零速度，松开 150ms 后自动归零。
- **组合按键**：支持同时按住多个按键（如 `W` + `A` = 前进同时左转）。
- 冲突按键（如 `W` + `S`）以后按下的按键为准。

## ROS 2 接口

### 发布的话题

| 话题名 | 类型 | 频率 | 说明 |
|--------|------|------|------|
| `cmd_vel` | `geometry_msgs/msg/Twist` | 10 Hz（可配） | 机器人运动速度指令 |

### 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `publish_rate` | double | 10.0 | 发布频率 (Hz) |
| `linear_speed` | double | 0.5 | 前/后退线速度 (m/s) |
| `angular_speed` | double | 0.5 | 转向角速度 (rad/s) |
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
