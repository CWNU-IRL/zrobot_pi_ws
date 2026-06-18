# zrobot_bridge

## 概述

实机 RobStride 电机 CAN 总线桥接节点。通过 4 路 CAN 接口（can10 ~ can13）与 23 个 RobStride 系列电机通信，实现位置控制、速度控制、电流控制、参数读写、零点校准等功能。提供 `rs_interface` 定义的三个 ROS 2 服务，是 `zrobot_deploy` 在实机部署时的底层电机驱动。

## 依赖

### 系统依赖

- `can-utils`（Linux CAN 工具集）
- Linux Kernel CAN 支持（`CONFIG_CAN`）

### ROS 2

- `rclcpp`
- `rs_interface`（自定义服务接口）

## 构建

```bash
colcon build --packages-select rs_interface zrobot_bridge
```

## 使用

### 1. 配置 CAN 接口

```bash
# 使用脚本自动配置 4 路 CAN（can10 ~ can13），波特率 1 Mbps
sudo bash src/zrobot_bridge/scripts/setup_can_interfaces.sh
```

### 2. 启动桥接节点

```bash
ros2 launch zrobot_bridge motor_controller.launch.py
```

### 3. 使用自定义配置文件

```bash
ros2 launch zrobot_bridge motor_controller.launch.py \
    config_file:=src/zrobot_bridge/config/motor_config.yaml
```

## 配置

### motor_config.yaml

| 参数                   | 类型       | 说明                         |
| ---------------------- | ---------- | ---------------------------- |
| `master_id`            | int        | 主机 CAN ID（默认 0xFD）     |
| `motor_can_ids`        | int[23]    | 23 个电机的 CAN ID           |
| `motor_types`          | int[23]    | 23 个电机的型号（0-6）       |
| `motor_can_interfaces` | string[23] | 23 个电机所属的 CAN 接口名称 |
| `motor_kps`            | float[23]  | 23 个电机的 Kp 参数          |
| `motor_kds`            | float[23]  | 23 个电机的 Kd 参数          |

### CAN 接口分布

| CAN 接口 | 电机索引 | 电机数量 |
| -------- | -------- | -------- |
| can10    | 0-5      | 6        |
| can11    | 6-12     | 7        |
| can12    | 13-16    | 4        |
| can13    | 17-22    | 6        |

## ROS 2 接口

### 提供的服务

| 服务名                | 服务类型                         | 说明                                 |
| --------------------- | -------------------------------- | ------------------------------------ |
| `/rob_stride_control` | `rs_interface/srv/RobStrideMsgs` | 发送 23 个电机位置指令，返回反馈数据 |
| `/get_positions`      | `rs_interface/srv/GetPositions`  | 读取 23 个电机的当前位置             |
| `/set_zeros`          | `rs_interface/srv/SetZeros`      | 将当前位置校准为机械零位             |

## 项目结构

```
zrobot_bridge/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── motor_config.yaml
│   └── motor_config1.yaml
├── launch/
│   └── motor_controller.launch.py
├── scripts/
│   └── setup_can_interfaces.sh
├── include/zrobot_bridge/
│   ├── motor_controller.h
│   └── motor_cfg.h
└── src/
    ├── motor_controller_node.cpp
    └── motor_cfg.cpp
```

## 技术细节

### CAN 协议

使用 SocketCAN（`PF_CAN`, `SOCK_RAW`）通信，所有帧采用**扩展帧 ID**（29 位）。CAN ID 编码格式：

```
| 31-24        | 23-22 | 21-16   | 15-8     | 7-0       |
| 通信类型      | 保留  | 错误码  | 扩展数据  | 主机 ID   |
```

### 通信类型

| 值   | 类型           | 说明                          |
| ---- | -------------- | ----------------------------- |
| 0x00 | Get ID         | 获取设备 ID 和 MCU 唯一标识符 |
| 0x01 | Motion Control | 运控模式控制指令              |
| 0x02 | Motor Request  | 电机状态反馈                  |
| 0x03 | Motor Enable   | 电机使能                      |
| 0x04 | Motor Stop     | 电机停止                      |
| 0x06 | Set Pos Zero   | 设置机械零位                  |
| 0x07 | Set CAN ID     | 更改电机 CAN ID               |
| 0x11 | Get Parameter  | 读取单个参数                  |
| 0x12 | Set Parameter  | 设定单个参数                  |
| 0x15 | Error Feedback | 故障反馈帧                    |

### 控制模式

| 模式             | 说明                                                                    |
| ---------------- | ----------------------------------------------------------------------- |
| 运控模式 (0)     | 运动控制：发送位置 + 速度 + Kp + Kd + 转矩，接收位置/速度/转矩/温度反馈 |
| 位置模式 PP (1)  | 点对点位置控制：指定速度、加速度、目标角度                              |
| 速度模式 (2)     | 速度环控制：发送目标速度                                                |
| 电流模式 (3)     | 电流环控制：Iq 和 Id 指令                                               |
| 零点模式 (4)     | 设置机械零位                                                            |
| 位置模式 CSP (5) | 循环同步位置控制                                                        |

### 电机型号参数

| 型号         | 最大位置 | 最大速度 | 最大转矩 | Kp 范围 | Kd 范围 |
| ------------ | -------- | -------- | -------- | ------- | ------- |
| ROBSTRIDE_00 | ±4π rad  | 50 rad/s | 17 Nm    | 0-500   | 0-5     |
| ROBSTRIDE_01 | ±4π rad  | 44 rad/s | 17 Nm    | 0-500   | 0-5     |
| ROBSTRIDE_02 | ±4π rad  | 44 rad/s | 17 Nm    | 0-500   | 0-5     |
| ROBSTRIDE_03 | ±4π rad  | 50 rad/s | 60 Nm    | 0-5000  | 0-100   |
| ROBSTRIDE_04 | ±4π rad  | 15 rad/s | 120 Nm   | 0-5000  | 0-100   |
| ROBSTRIDE_05 | ±4π rad  | 33 rad/s | 17 Nm    | 0-500   | 0-5     |
| ROBSTRIDE_06 | ±4π rad  | 20 rad/s | 60 Nm    | 0-5000  | 0-100   |

### 可读写参数

| 索引   | 参数名        | 类型  | 说明                            |
| ------ | ------------- | ----- | ------------------------------- |
| 0x7005 | run_mode      | uint8 | 运行模式                        |
| 0x7006 | iq_ref        | float | 电流模式 Iq 指令 (-23~23A)      |
| 0x700A | spd_ref       | float | 速度模式转速指令 (-30~30 rad/s) |
| 0x700B | imit_torque   | float | 转矩限制 (0~12 Nm)              |
| 0x7010 | cur_kp        | float | 电流 Kp (默认 0.125)            |
| 0x7011 | cur_ki        | float | 电流 Ki (默认 0.0158)           |
| 0x7014 | cur_filt_gain | float | 电流滤波系数 (0~1.0)            |
| 0x7016 | loc_ref       | float | 位置模式角度指令 (rad)          |
| 0x7017 | limit_spd     | float | 位置模式速度限制 (0~30 rad/s)   |
| 0x7018 | limit_cur     | float | 速度/位置模式电流限制 (0~23 A)  |
| 0x7019 | mechPos       | float | 负载端机械角度 (rad，只读)      |
| 0x701A | iqf           | float | Iq 滤波值 (只读)                |
| 0x701B | mechVel       | float | 负载端转速 (只读)               |
| 0x701C | VBUS          | float | 母线电压 (V，只读)              |
| 0x701D | rotation      | int16 | 圈数 (只读)                     |
