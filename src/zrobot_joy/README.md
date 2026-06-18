# zrobot_joy

## 概述

ROS 2 Xbox 控制器遥控节点。订阅 Linux `joy` 驱动发布的 `/joy` 话题，将手柄摇杆和按键映射为 `geometry_msgs/Twist` 消息发布到 `cmd_vel` 话题。支持左摇杆控制线速度、右摇杆控制角速度、LB 使能开关、RB 加速模式。

## 依赖

### ROS 2

- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `joy`（Linux 手柄驱动，运行时依赖）

## 构建

```bash
colcon build --packages-select zrobot_joy
```

## 使用

### 启动

```bash
ros2 launch zrobot_joy joy_teleop.launch.py
```

该启动文件同时启动 `joy_node`（Linux 手柄驱动）和 `joy_teleop_node`。

### 使用自定义参数文件

```bash
ros2 launch zrobot_joy joy_teleop.launch.py \
    params_file:=src/zrobot_joy/config/xbox_joy_params.yaml
```

## 操作说明

| 手柄操作     | 行为                        |
| ------------ | --------------------------- |
| 左摇杆 Y     | 前进 / 后退                 |
| 左摇杆 X     | 横向平移                    |
| 右摇杆 X     | 左右旋转                    |
| LB（按键 4） | 使能开关（按住才发送指令）  |
| RB（按键 5） | 加速模式（按住 1.5 倍速度） |

## ROS 2 接口

### 订阅的话题

| 话题名 | 类型                  | 说明                             |
| ------ | --------------------- | -------------------------------- |
| `/joy` | `sensor_msgs/msg/Joy` | Linux 手柄驱动发布的原始手柄数据 |

### 发布的话题

| 话题名    | 类型                      | 频率  | 说明               |
| --------- | ------------------------- | ----- | ------------------ |
| `cmd_vel` | `geometry_msgs/msg/Twist` | 50 Hz | 机器人运动速度指令 |

### 参数

| 参数名                | 类型   | 默认值    | 说明              |
| --------------------- | ------ | --------- | ----------------- |
| `joy_topic`           | string | `/joy`    | 手柄输入话题名    |
| `cmd_topic`           | string | `cmd_vel` | 速度指令话题名    |
| `publish_rate`        | double | 50.0      | 发布频率 (Hz)     |
| `enable_button`       | int    | 4         | 使能按键（LB）    |
| `enable_turbo_button` | int    | 5         | 加速按键（RB）    |
| `axis_linear_x`       | int    | 1         | 前进 / 后退摇杆轴 |
| `axis_linear_y`       | int    | 0         | 横向平移摇杆轴    |
| `axis_angular`        | int    | 3         | 旋转摇杆轴        |
| `scale_linear_x`      | double | 0.7       | 线速度缩放因子    |
| `scale_linear_y`      | double | 0.5       | 横向速度缩放因子  |
| `scale_angular`       | double | 0.8       | 角速度缩放因子    |
| `scale_turbo`         | double | 1.5       | 加速倍率          |
| `deadzone`            | double | 0.1       | 摇杆死区          |

## 项目结构

```
zrobot_joy/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── xbox_joy_params.yaml
├── launch/
│   └── joy_teleop.launch.py
└── src/
    └── joy_teleop_node.cpp
```

## 技术细节

- 使用 `std::mutex` 保护共享 Twist 数据，确保发布线程和回调线程安全
- 1 秒超时保护：如果在 1 秒内未收到手柄数据（如手柄断开），自动停止机器人
- 摇杆死区（默认 0.1）滤除小幅度噪声
- 使能按钮（LB）防止意外碰撞：松开 LB 后立即发送零速度
