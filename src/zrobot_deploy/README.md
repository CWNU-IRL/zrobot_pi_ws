# zrobot_deploy

## 概述

ROS 2 C++ 运动控制系统，基于有限状态机（FSM）控制 zrobot 机器人。支持四种运行模式：FixStand（固定站立）、Locomotion（ONNX 推理运动）、PTLocomotion（LibTorch 推理运动）、Damping（软件阻尼仿真）。所有模式均通过 `/rob_stride_control` 服务向底层电机驱动发送 23 个关节的位置命令，控制频率 100 Hz。

## 依赖

### 系统依赖
- ONNX Runtime（`onnxruntime-linux-aarch64-1.16.3` 或 `onnxruntime-linux-x64-1.16.3`）
- LibTorch（与系统架构匹配的 PyTorch C++ 库）
- Eigen3

### ROS 2
- `rclcpp`
- `rs_interface`（自定义服务定义包）
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `tf2`

### 策略模型文件

放置在工作区 `resources/policy/` 目录下（默认路径编译时由 `POLICY_DIR` 宏定义）：

```
resources/policy/
├── policy.onnx          # Locomotion 使用的 ONNX 模型
└── policy_1.pt          # PTLocomotion 使用的 TorchScript 模型
```

## 构建

```bash
# 首次需要在工作区根目录放置 ONNX Runtime 和 LibTorch
# 目录结构参考 CMakeLists.txt 中的路径配置
colcon build --packages-select rs_interface zrobot_deploy
```

## 使用

```bash
# 启动底层电机桥接节点
ros2 launch zrobot_bridge motor_controller.launch.py

# 另一终端启动 FSM 控制器
ros2 run zrobot_deploy main
```

### 键盘控制

| 按键 | 功能 |
|------|------|
| `F` | 启动 FixStand（移动到机械零位并保持站立） |
| `L` | 启动 Locomotion（ONNX 推理运动） |
| `T` | 启动 PTLocomotion（TorchScript 推理运动） |
| `D` | 启动 Damping（软件阻尼） |
| `S` | 停止当前状态机 |
| `Q` | 退出程序 |

### 注意事项

- FixStand 使用 3 秒线性插值将机器人从当前位置移动到零位，避免突然动作
- 启动 Locomotion 前确保 IMU 节点已运行（订阅 `/imu/data`）
- 可在 `cmd_vel` 话题上发布速度指令控制机器人前进/转向
- 停止 Damping 时会自动切换到 FixStand，防止机器人失电瘫倒

## ROS 2 接口

### 调用的服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/rob_stride_control` | `rs_interface/srv/RobStrideMsgs` | 发送 23 电机位置命令（100 Hz） |
| `/get_positions` | `rs_interface/srv/GetPositions` | 读取当前电机位置（初始化时使用） |

### 订阅的话题

| 话题名 | 类型 | 用途 |
|--------|------|------|
| `/imu/data` | `sensor_msgs/msg/Imu` | Locomotion/PTLocomotion 观测输入（角速度、姿态角） |
| `cmd_vel` | `geometry_msgs/msg/Twist` | 运动速度指令（vx, vy, wz） |

## 项目结构

```
zrobot_deploy/
├── CMakeLists.txt
├── package.xml
├── include/zrobot_deploy/
│   ├── FSM.h               # FSM 抽象基类
│   ├── FixStand.h          # 固定站立状态机
│   ├── Locomotion.h        # ONNX 推理运动状态机
│   ├── PTLocomotion.h      # TorchScript 推理运动状态机
│   └── Damping.h           # 软件阻尼状态机
└── src/
    ├── FSM.cpp             # 基类实现（服务调用、反馈缓存）
    ├── FixStand.cpp        # 站立：3s 线性插值到零位
    ├── Locomotion.cpp      # ONNX 推理控制循环
    ├── PTLocomotion.cpp    # TorchScript 推理控制循环
    ├── Damping.cpp         # 阻尼：q_cmd = q_fb - kd * dq_fb
    └── main.cpp            # 入口：键盘控制 + 100Hz 主循环
```

## 技术细节

### FSM 基类

每个状态机继承自 `FSM`，实现三个虚函数：

| 方法 | 调用时机 | 用途 |
|------|---------|------|
| `initialize()` | 切换到该状态时 | 初始化参数、订阅话题、启动线程 |
| `run()` | 每 10ms（100 Hz） | 计算并发送电机位置 |
| `exit()` | 离开该状态时 | 清理资源、停止线程 |

### FixStand

3 个内部状态：`INIT` → `MOVING`（3 秒线性插值）→ `STANDING`（保持零位）

插值公式：`current[i] = initial[i] + t * (0 - initial[i])`，其中 `t = min(elapsed / 3.0, 1.0)`

### Locomotion（ONNX 推理）

47 维观测向量：

| 索引 | 维度 | 数据 |
|------|------|------|
| 0-1 | 2 | 步态相位 sin/cos（周期 0.64s） |
| 2-4 | 3 | 指令速度 vx/vy/wz（缩放后） |
| 5-16 | 12 | 关节位置（相对默认姿态） |
| 17-28 | 12 | 关节速度 |
| 29-40 | 12 | 前一步动作 |
| 41-43 | 3 | IMU 角速度 |
| 44-46 | 3 | IMU 欧拉角 |

策略输出 12 维动作（腿部 12 个关节的增量位置），通过 `dof_indices_` 映射到 23 个电机索引。动作经过限幅安全保护（`action_abs_limit_`、`action_delta_limit_`），观测使用 15 帧堆叠输入。

推理在独立线程中以 100 Hz 运行，主线程仅读取最新推理结果并发送位置命令。

### PTLocomotion（LibTorch 推理）

结构和逻辑与 Locomotion 完全一致，区别在于使用 `torch::jit::load()` 加载 `.pt` 模型进行推理。

### Damping（软件阻尼）

模拟弹簧阻尼效果：`q_cmd = q_fb - kd * dq_fb`，通过位置控制实现近似阻尼行为。

- 默认 `kd_default_` = 0.08
- 死区 `velocity_deadband_` = 0.02 rad/s（低于此不做阻尼）
- 最大位置变化 `max_position_delta_` = 0.15 rad/步（安全限幅）

### 主循环

`main.cpp` 使用 raw terminal（`termios`）实现非阻塞键盘检测，100 Hz 控制循环中依次执行：

```
if 有按键: 切换/停止状态机
if 有当前 FSM: current_fsm->run()
rclcpp::spin_some(node)  # 处理话题回调
```
