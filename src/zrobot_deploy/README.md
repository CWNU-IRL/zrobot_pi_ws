# zrobot_deploy

## 概述

ZRobot 的核心运动控制部署包。实现基于有限状态机的运动控制器框架，包含 4 种可切换的状态机：`FixStand`（固定站立）、`Locomotion`（ONNX 模型推理）、`PTLocomotion`（TorchScript 模型推理）和 `Damping`（虚拟阻尼）。通过 `rs_interface` 的统一服务接口与底层电机桥接通信，支持实机与仿真无缝切换。

## 依赖

### 系统依赖

- `libeigen3-dev`（Eigen 线性代数库）

### 第三方预编译库

- ONNX Runtime 1.16.3（`Locomotion` 状态机使用）
- LibTorch 2.12+cpu（`PTLocomotion` 状态机使用）

### ROS 2

- `rclcpp`
- `rs_interface`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `tf2`

## 构建

```bash
colcon build --packages-select rs_interface zrobot_deploy
```

## 使用

### 启动

```bash
# 先启动底层桥接（实机、Gazebo 或 MuJoCo），再启动 FSM 控制器
ros2 run zrobot_deploy main
```

### 键盘控制

启动后在终端按键切换状态机：

| 按键 | 模式         | 说明                                              |
| ---- | ------------ | ------------------------------------------------- |
| `F`  | FixStand     | 3 秒线性插值到站立姿态并保持                      |
| `L`  | Locomotion   | ONNX Runtime 推理运动（需 IMU 数据）              |
| `T`  | PTLocomotion | LibTorch 推理运动（需 IMU 数据）                  |
| `D`  | Damping      | 软件阻尼模式                                      |
| `S`  | Stop         | 停止当前状态机（Damping 停止后自动恢复 FixStand） |
| `Q`  | Quit         | 退出程序                                          |

主循环运行在 **100 Hz**，键盘检测非阻塞，ROS 回调通过 `rclcpp::spin_some()` 异步处理。

## ROS 2 接口

### 订阅的话题

| 话题名      | 类型                      | 说明                                           |
| ----------- | ------------------------- | ---------------------------------------------- |
| `/imu/data` | `sensor_msgs/msg/Imu`     | IMU 数据（Locomotion / PTLocomotion 使用）     |
| `cmd_vel`   | `geometry_msgs/msg/Twist` | 遥控速度指令（Locomotion / PTLocomotion 使用） |

### 调用的服务

| 服务名                | 服务类型                         | 说明                   |
| --------------------- | -------------------------------- | ---------------------- |
| `/rob_stride_control` | `rs_interface/srv/RobStrideMsgs` | 发送 23 个电机位置指令 |
| `/get_positions`      | `rs_interface/srv/GetPositions`  | 读取当前电机位置       |

## 项目结构

```
zrobot_deploy/
├── CMakeLists.txt
├── package.xml
├── include/zrobot_deploy/
│   ├── FSM.h              # 状态机基类
│   ├── FixStand.h         # 固定站立状态
│   ├── Locomotion.h       # ONNX 推理运动状态
│   ├── PTLocomotion.h     # TorchScript 推理运动状态
│   └── Damping.h          # 虚拟阻尼状态
└── src/
    ├── main.cpp            # FSM 控制器入口 + 键盘交互
    ├── FSM.cpp             # 基类实现（服务调用封装）
    ├── FixStand.cpp        # 站立姿态插值
    ├── Locomotion.cpp      # ONNX 模型加载 + 推理循环
    ├── PTLocomotion.cpp    # TorchScript 模型加载 + 推理循环
    └── Damping.cpp         # 虚拟阻尼计算
```

## 技术细节

### FSM 架构

```
                    +------------+
                    |   IDLE     |
                    +-----+------+
                          |
          +---------------+---------------+
          |               |               |
          v               v               v
    +----------+   +-----------+   +----------+
    | FixStand |   | Locomotion|   | Damping  |
    +----------+   | (ONNX)    |   +----------+
                   +-----------+
                   | PTLocomot.|
                   | (.pt)     |
                   +-----------+
```

所有状态机继承自 `FSM` 基类，基类封装了：
- 两个服务客户端（`/rob_stride_control`, `/get_positions`）
- `sendMotorPositions()` — 发送 23 电机位置（100 ms 超时）
- `getMotorFeedback()` — 读取缓存反馈
- `getCurrentPositions()` — 通过服务读取位置（最长 5 s 超时）

### FixStand（固定站立）

- 3 秒线性插值：`q_target = q_current + (q_stand - q_current) * t / 3.0`
- 站立姿态（12 个腿部关节）：
  - 左腿：hip_pitch = -0.45, knee = -0.85, foot_pitch = 0.4
  - 右腿：hip_pitch = 0.45, knee = 0.85, foot_pitch = -0.4
  - hip_roll / hip_yaw / foot_roll 归零
- 插值完成后持续保持站立位置

### Locomotion（ONNX 推理）

- 使用 ONNX Runtime C++ API 加载 `resources/policy/policy.onnx`
- **观测空间**：47 维单帧，经过 `frame_stack_`（默认 15）帧堆叠后输入维度为 705

| 观测分量   | 维度 | 说明                                         |
| ---------- | ---- | -------------------------------------------- |
| 步态相位   | 2    | sin(phase), cos(phase)，周期 `phase_period_` |
| 指令速度   | 3    | vx, vy, ωz，来自 `cmd_vel`                   |
| 关节位置   | 12   | 12 个腿部关节当前角度                        |
| 关节速度   | 12   | 12 个腿部关节当前速度                        |
| 上一帧动作 | 12   | 上一控制周期的策略输出                       |
| IMU 角速度 | 3    | 机体角速度 (rad/s)                           |
| IMU 欧拉角 | 3    | Roll, Pitch, Yaw (rad)                       |

- **动作空间**：12 维关节位置增量，通过 `dof_indices_` 映射到 23 电机数组
- **控制参数**：`dt = 0.01`、`action_scale = 0.25`、`obs_clip = 18`、`act_clip = 18`
- **推理线程**：独立 `std::thread` 运行 `inferenceLoop()`，100 Hz 推理
- **主线程**：100 Hz 发送电机指令，读写锁保护动作缓冲区
- 启动后等待 IMU 数据（最长 3 秒），确保观测完整性

### PTLocomotion（TorchScript 推理）

- 使用 LibTorch `torch::jit::load()` 加载 `resources/policy/policy_1.pt`
- 观测空间、动作空间、控制逻辑与 `Locomotion` 相同
- 使用 `torch::NoGradGuard()` 确保推理时不计算梯度

### Damping（虚拟阻尼）

- 用于机器人倒地等非正常状态下的安全保护
- 计算公式：`q_cmd = q_fb - kd * dq_fb`
- 可配置参数：`kd_default = 0.08`、`velocity_deadband = 0.02`、`max_position_delta = 0.15`、每关节独立 kd 数组
- 首次调用时将当前位置作为历史值缓存

### 构建说明

`CMakeLists.txt` 自动检测架构：
- **aarch64（树莓派等）**：LibTorch 从系统 Python 包中获取，ONNX Runtime 从 `third_party/onnxruntime-linux-aarch64-1.16.3/`
- **x86_64（PC）**：LibTorch 从 `third_party/libtorch/`，ONNX Runtime 从 `third_party/onnxruntime-linux-x64-1.16.3/`
