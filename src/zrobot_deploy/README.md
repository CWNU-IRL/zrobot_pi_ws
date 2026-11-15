# zrobot_deploy

## 概述

ZRobot 的核心运动控制部署包。实现基于有限状态机的运动控制器框架，包含 4 种可切换的状态机：`FixStand`（固定站立）、`Locomotion`（ONNX 模型推理）、`PTLocomotion`（TorchScript 模型推理）和 `Damping`（驱动阻尼）。通过 `rs_interface` 的 `/joint_cmd` / `/joint_state` 命令流契约与底层电机桥接通信，支持实机与仿真无缝切换。

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
| `D`  | Damping      | 发布 damp 命令，由驱动做阻尼                      |
| `S`  | Stop         | 停止当前状态机（Damping 停止后自动恢复 FixStand） |
| `Q`  | Quit         | 退出程序                                          |

主循环运行在 **100 Hz**，键盘检测非阻塞；ROS 回调由独立 `SingleThreadedExecutor` 线程处理，`initialize()` 可在主线程安全等待首帧 `/joint_state`。

## ROS 2 接口

### 订阅的话题

| 话题名         | 类型                                | 说明 |
| -------------- | ----------------------------------- | ---- |
| `/joint_state` | `rs_interface/msg/JointState`       | 23 路电机反馈 + `valid[]`，QoS best effort / depth 1 |
| `/imu/data`    | `sensor_msgs/msg/Imu`               | IMU 数据（Locomotion / PTLocomotion 使用） |
| `cmd_vel`      | `geometry_msgs/msg/Twist`           | 遥控速度指令（Locomotion / PTLocomotion 使用） |

### 发布的话题

| 话题名      | 类型                        | 说明 |
| ----------- | --------------------------- | ---- |
| `/joint_cmd` | `rs_interface/msg/JointCmd` | reliable / depth 1；`mit/enable/damp/disable/set_zero/status` |

`initialize()` 先发布 `enable`，然后等待首帧 `/joint_state` 且 12 个主动关节 `valid=true`；不再有服务调用或 `spin_until_future_complete` 阻塞。

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
│   └── Damping.h          # 驱动阻尼状态
└── src/
    ├── main.cpp            # FSM 控制器入口 + 键盘交互
    ├── FSM.cpp             # 基类实现（命令发布/状态缓存）
    ├── FixStand.cpp        # 站立姿态插值
    ├── Locomotion.cpp      # ONNX 模型加载 + 推理循环
    ├── PTLocomotion.cpp    # TorchScript 模型加载 + 推理循环
    └── Damping.cpp         # damp 命令发布
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
- `/joint_cmd` publisher（reliable / depth 1）与 `/joint_state` subscriber（best effort / depth 1）
- `sendMotorPositions()` — 发布 23 路 MIT 位置命令（非阻塞）
- `getMotorFeedback()` / `getCurrentPositions()` — 读取最新状态缓存
- `stateFresh()` / `activeJointsValid()` — 状态新鲜度与有效性门控
- `sendDampCommand()` — 发布 `damp`，由驱动使用自身反馈做阻尼

Locomotion / PTLocomotion 在 `/joint_state` 超时或主动关节 `valid=false` 时暂停推理并发布 `damp`；`cmd_vel` 超过 `cmd_vel_timeout_ms` 未更新时速度指令清零。`Q` 退出时 `main` 发布一次 `damp`。

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

### Damping（驱动阻尼）

- 用于机器人倒地等非正常状态下的安全保护
- 每周期发布 `command="damp"`；bridge 使用最后有效反馈位置、`kp=0`、`kd=damping_kd` 生成阻尼命令
- 阻尼增益在 bridge 的 `motor_config.yaml` 中配置，deploy 不再计算软件阻尼位置

### 构建说明

`CMakeLists.txt` 自动检测架构：
- **aarch64（树莓派等）**：LibTorch 从系统 Python 包中获取，ONNX Runtime 从 `third_party/onnxruntime-linux-aarch64-1.16.3/`
- **x86_64（PC）**：LibTorch 从 `third_party/libtorch/`，ONNX Runtime 从 `third_party/onnxruntime-linux-x64-1.16.3/`
