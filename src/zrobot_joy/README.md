# zrobot_joy

## 概述

Xbox 手柄遥控节点，将手柄摇杆输入转换为 `cmd_vel` 速度指令，控制机器人运动。与 `joy_node`（系统 `joy` 包）配合使用，前者读取 `/dev/input/js0` 手柄原始数据并发布 `/joy` 话题，本节点订阅该话题并映射为 `geometry_msgs/msg/Twist`。

## 依赖

### 系统
- Xbox 手柄（USB 或无线适配器）
- `ros-jazzy-joy`（已安装 `joy_node`）

### ROS 2
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`

## 构建

```bash
colcon build --packages-select zrobot_joy
```

## 使用

### 一键启动（推荐）

```bash
ros2 launch zrobot_joy joy_teleop.launch.py
```

同时启动 `joy_node` + `joy_teleop_node`。

### 手动启动

```bash
# 终端 1: 启动手柄驱动
ros2 run joy joy_node --ros-args -p deadzone:=0.05

# 终端 2: 启动映射节点
ros2 run zrobot_joy joy_teleop_node
```

## 按键映射

| 控件         | 行为                                   |
| ------------ | -------------------------------------- |
| **LB**       | **启用键** —— 按住时摇杆生效，松开停止 |
| **左摇杆 Y** | 前进/后退 (linear.x)                   |
| **左摇杆 X** | 左/右平移 (linear.y)                   |
| **右摇杆 X** | 左/右转向 (angular.z)                  |
| **RB**       | 极速模式 —— 速度 ×1.5                  |

## ROS 2 接口

### 订阅的话题

| 话题名 | 类型                  | 说明                            |
| ------ | --------------------- | ------------------------------- |
| `/joy` | `sensor_msgs/msg/Joy` | 手柄原始数据（来自 `joy_node`） |

### 发布的话题

| 话题名    | 类型                      | 频率  | 说明           |
| --------- | ------------------------- | ----- | -------------- |
| `cmd_vel` | `geometry_msgs/msg/Twist` | 50 Hz | 机器人速度指令 |

### 参数

| 参数名                | 类型   | 默认值    | 说明                 |
| --------------------- | ------ | --------- | -------------------- |
| `joy_topic`           | string | `/joy`    | 手柄数据话题         |
| `cmd_topic`           | string | `cmd_vel` | 输出话题             |
| `publish_rate`        | double | 50.0      | 发布频率 (Hz)        |
| `enable_button`       | int    | 4         | 启用按钮（LB）       |
| `enable_turbo_button` | int    | 5         | 极速按钮（RB）       |
| `axis_linear_x`       | int    | 1         | 前后轴               |
| `axis_linear_y`       | int    | 0         | 平移轴               |
| `axis_angular`        | int    | 3         | 转向轴               |
| `scale_linear_x`      | double | 0.7       | 最大前向速度 (m/s)   |
| `scale_linear_y`      | double | 0.5       | 最大平移速度 (m/s)   |
| `scale_angular`       | double | 0.8       | 最大转向速度 (rad/s) |
| `scale_turbo`         | double | 1.5       | 极速倍率             |
| `deadzone`            | double | 0.1       | 摇杆死区             |

## 终端显示

运行时终端实时显示当前速度指令：

```
[LB] vx:  0.350  vy:  0.000  vz:  0.400
```

- `[LB]` 表示 LB 已按下（启用），`[  ]` 表示未启用
- 手柄断开 1 秒后自动归零

## Xbox 按钮索引参考

| 按钮 | 索引 |
| ---- | ---- |
| A    | 0    |
| B    | 1    |
| X    | 2    |
| Y    | 3    |
| LB   | 4    |
| RB   | 5    |
| View | 6    |
| Menu | 7    |
| Xbox | 8    |
| L3   | 9    |
| R3   | 10   |
