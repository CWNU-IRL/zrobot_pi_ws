# zrobot_bridge

## 概述

ROS 2 C++ 桥接节点，通过 CAN 总线与 RobStride 系列无刷电机通信。`MotorControllerNode` 暴露三个 ROS 2 服务（`/rob_stride_control`、`/get_positions`、`/set_zeros`），将上层控制指令转换为 CAN 扩展帧发送给 23 个电机，并将电机反馈（位置、速度、扭矩、温度）返回给调用方。

## 依赖

### 系统依赖
- Linux CAN 支持（`socketCAN`）
- 4 路 CAN 接口（默认：can10~can13，1 Mbps）

### ROS 2
- `rclcpp`
- `rs_interface`（自定义服务定义包）

## 构建

```bash
colcon build --packages-select rs_interface zrobot_bridge
```

## 使用

### 1. 配置 CAN 接口

```bash
# 设置 4 路 CAN 总线
sudo ip link set can10 up type can bitrate 1000000
sudo ip link set can11 up type can bitrate 1000000
sudo ip link set can12 up type can bitrate 1000000
sudo ip link set can13 up type can bitrate 1000000
```

或使用脚本：

```bash
sudo bash src/zrobot_bridge/scripts/setup_can_interfaces.sh
```

### 2. 启动桥接节点

```bash
ros2 launch zrobot_bridge motor_controller.launch.py
```

### 3. 调用服务测试

```bash
# 发送位置命令（全零）
ros2 service call /rob_stride_control rs_interface/srv/RobStrideMsgs "{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 读取当前位置
ros2 service call /get_positions rs_interface/srv/GetPositions

# 设置零位
ros2 service call /set_zeros rs_interface/srv/SetZeros
```

## ROS 2 接口

### 提供的服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/rob_stride_control` | `rs_interface/srv/RobStrideMsgs` | 发送 23 个电机位置指令，返回完整反馈 |
| `/get_positions` | `rs_interface/srv/GetPositions` | 读取当前电机位置 |
| `/set_zeros` | `rs_interface/srv/SetZeros` | 设置机械零位偏移 |

### 节点参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `master_id` | int | 0xFD | CAN 主机 ID |
| `motor_can_ids` | int[] | [0..22] | 各电机的 CAN ID |
| `motor_types` | int[] | [2, 2, 4, 4, 3, 3, ...] | 各电机的执行器类型 |
| `motor_can_interfaces` | string[] | ["can10", ...] | 各电机分配的 CAN 接口 |
| `motor_kps` | float[] | 1.0 | 各电机位置 PID 比例增益 |
| `motor_kds` | float[] | 0.5 | 各电机位置 PID 微分增益 |

## 项目结构

```
zrobot_bridge/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── motor_config.yaml           # 23 电机完整配置
│   └── motor_config1.yaml          # 6 电机简化配置（测试用）
├── include/zrobot_bridge/
│   ├── motor_cfg.h                 # RobStrideMotor 类（CAN 协议实现）
│   └── motor_controller.h          # MotorControllerNode 类声明
├── launch/
│   └── motor_controller.launch.py  # 启动文件
├── scripts/
│   └── setup_can_interfaces.sh     # CAN 接口配置脚本
└── src/
    ├── motor_cfg.cpp               # CAN 收发与 RobStride 协议实现
    └── motor_controller_node.cpp   # ROS 2 节点主逻辑
```

## 技术细节

### CAN 协议

`RobStrideMotor` 使用 **raw socket** 发送和接收 CAN 扩展帧（29-bit ID）。CAN ID 编码格式：

```
CAN ID = (master_id << 24) | (communication_type << 16) | (motor_id << 8) | extra_data
```

| 通信类型 | 值 | 说明 |
|---------|-----|------|
| `MotionControl` | 0x01 | 运动控制指令（位置/速度/扭矩/Kp/Kd） |
| `MotorRequest` | 0x02 | 电机请求/反馈 |
| `MotorEnable` | 0x03 | 使能电机 |
| `MotorStop` | 0x04 | 停止/失能电机 |
| `SetPosZero` | 0x06 | 设置机械零位 |
| `Control_Mode` | 0x12 | 设置/读取控制器参数 |

### 执行器类型

| 类型 | 最大位置 (rad) | 最大速度 (rad/s) | 最大扭矩 (Nm) |
|------|---------------|------------------|--------------|
| ROBSTRIDE_00 | 23.0 | 20.0 | 23.0 |
| ROBSTRIDE_01 | 23.0 | 20.0 | 23.0 |
| ROBSTRIDE_02 | 23.0 | 20.0 | 23.0 |
| ROBSTRIDE_03 | 23.0 | 20.0 | 60.0 |
| ROBSTRIDE_04 | 23.0 | 20.0 | 120.0 |
| ROBSTRIDE_05 | 23.0 | 20.0 | 17.0 |
| ROBSTRIDE_06 | 23.0 | 20.0 | 23.0 |

### CAN 总线分配

23 个电机分布在 4 路 CAN 总线上：

| CAN 接口 | 电机索引 | 数量 |
|----------|---------|------|
| can10 | 0-5 | 6 |
| can11 | 6-12 | 7 |
| can12 | 13-16 | 4 |
| can13 | 17-22 | 6 |

### 数据流

```
上层控制节点 (e.g. zrobot_deploy)
    ↓ /rob_stride_control (ROS 2 Service)
MotorControllerNode
    ↓ 每个电机独立 CAN socket（硬件过滤）
23× RobStrideMotor
    ↓ CAN 扩展帧
23× RobStride 电机
    ↑ 反馈（位置/速度/扭矩/温度）
```
