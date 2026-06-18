# ZRobot Pi Workspace

基于 ROS 2 的 ZRobot 双足机器人控制与仿真工作空间，覆盖硬件驱动（CAN 电机、IMU 传感器）到上层运动控制（有限状态机 + 强化学习推理），并提供 Gazebo 和 MuJoCo 双仿真环境。所有控制节点通过统一 ROS 2 服务接口通信，实现实机与仿真间的无缝切换。

## 快速开始

### 环境要求

- Ubuntu 22.04（ROS 2 Humble）或 Ubuntu 24.04（ROS 2 Jazzy）
- ROS 2 desktop 安装

### 安装系统依赖

```bash
sudo apt install -y libserial-dev can-utils libeigen3-dev freeglut3-dev
```

### 安装 ROS 2 仿真依赖

```bash
# Jazzy
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control ros-jazzy-ros2controllers

# Humble
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
    ros-humble-gz-ros2-control ros-humble-ros2controllers
```

### 下载预编译第三方库

手动下载并放置于 `third_party/` 目录：

| 库           | 架构                                                 | 版本     | 下载                                                                                   |
| ------------ | ---------------------------------------------------- | -------- | -------------------------------------------------------------------------------------- |
| MuJoCo       | `aarch64` + `x86_64`                                 | 3.9.0    | [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco/releases/tag/3.9.0) |
| ONNX Runtime | `aarch64` + `x86_64`                                 | 1.16.3   | [microsoft/onnxruntime](https://github.com/microsoft/onnxruntime/releases/tag/v1.16.3) |
| LibTorch     | `aarch64`（系统 Python）/ `x86_64`（`third_party/`） | 2.12+cpu | [pytorch.org](https://pytorch.org/get-started/locally/)                                |

### 构建

```bash
# 构建全部包
colcon build --symlink-install

# 或按层级构建
colcon build --packages-select rs_interface imu_msg              # 接口层
colcon build --packages-select zrobot_bridge imu_data_node       # 驱动层
colcon build --packages-select zrobot_deploy                     # 控制层
colcon build --packages-select zrobot_gz_sim                     # Gazebo 仿真
colcon build --packages-select zrobot_mj_sim                     # MuJoCo 仿真

source install/setup.bash
```

## 使用方式

### 实机部署

```bash
# 1. 配置 CAN 接口（需 4 路：can10 ~ can13）
sudo bash src/zrobot_bridge/scripts/setup_can_interfaces.sh

# 2. 启动电机 CAN 桥接
ros2 launch zrobot_bridge motor_controller.launch.py

# 3. （可选）启动 IMU 节点
ros2 run imu_data_node imu_data_node

# 4. 启动 FSM 运动控制器
ros2 run zrobot_deploy main
```

### Gazebo 仿真

```bash
ros2 launch zrobot_gz_sim sim_bringup.launch.py

# 另一终端
source install/setup.bash
ros2 run zrobot_deploy main
```

### MuJoCo 仿真

```bash
ros2 launch zrobot_mj_sim mujoco_bringup.launch.py

# 另一终端
source install/setup.bash
ros2 run zrobot_deploy main
```

### FSM 键盘控制

启动 `zrobot_deploy` 后，在终端按键切换状态机：

| 按键 | 模式         | 说明                                              |
| ---- | ------------ | ------------------------------------------------- |
| `F`  | FixStand     | 3 秒线性插值到站立姿态并保持                      |
| `L`  | Locomotion   | ONNX Runtime 推理运动（需 IMU 数据）              |
| `T`  | PTLocomotion | LibTorch 推理运动（需 IMU 数据）                  |
| `D`  | Damping      | 软件阻尼模式（倒地保护）                          |
| `S`  | Stop         | 停止当前状态机（Damping 停止后自动恢复 FixStand） |
| `Q`  | Quit         | 退出程序                                          |

MuJoCo 渲染窗口额外支持：`ESC` 退出、`r` 重置仿真、`c` 重置相机。

### 键盘 / 手柄遥控

- **键盘**：`ros2 run zrobot_control cmd_vel_publisher`（WASD 控制）
- **Xbox 手柄**：`ros2 launch zrobot_joy joy_teleop.launch.py`（LB 使能 + 摇杆控制）

遥控指令发布到 `cmd_vel`，被 `zrobot_deploy` 的 Locomotion / PTLocomotion 状态机订阅。

### 模型转换

```bash
# 将 TorchScript 模型转换为 ONNX 格式
python3 scripts/pt2onnx.py --jit_model policy_1.pt --onnx_model policy.onnx --obs_dim 47
```

## 项目结构

```
zrobot_pi_ws/
├── src/                          # ROS 2 包源码
│   ├── imu_msg/                  # 自定义 IMU 消息（ImuData.msg）
│   ├── rs_interface/             # 自定义电机控制服务（3 个 srv）
│   ├── imu_data_node/            # WIT 系列 IMU 串口驱动（TTL/CAN/RS485）
│   ├── zrobot_bridge/            # RobStride 电机 CAN 总线桥接（实机）
│   ├── zrobot_control/           # 键盘遥控 cmd_vel 发布节点
│   ├── zrobot_joy/               # Xbox 手柄遥控 cmd_vel 发布节点
│   ├── zrobot_deploy/            # FSM 运动控制 + RL 推理部署
│   ├── zrobot_gz_sim/            # Gazebo 仿真桥接
│   └── zrobot_mj_sim/            # MuJoCo 仿真桥接
├── resources/
│   └── policy/                   # RL 策略模型（.onnx, .pt）
├── scripts/
│   └── pt2onnx.py                # TorchScript → ONNX 模型转换与验证
├── third_party/                  # 预编译第三方库
│   ├── mujoco-3.9.0/
│   ├── onnxruntime-linux-{aarch64,x64}-1.16.3/
│   └── libtorch/
├── build/                        # colcon 构建中间文件
├── install/                      # colcon 产出
└── log/                          # 构建日志
```

### 包速览

| 包名                                             | 类型        | 说明                                                                  |
| ------------------------------------------------ | ----------- | --------------------------------------------------------------------- |
| [`imu_msg`](src/imu_msg/README.md)               | 接口（msg） | `ImuData` 消息：标准 IMU + 欧拉角扩展                                 |
| [`rs_interface`](src/rs_interface/README.md)     | 接口（srv） | 3 个电机控制服务定义                                                  |
| [`imu_data_node`](src/imu_data_node/README.md)   | 驱动        | WIT 系列 IMU 数据采集，50 Hz 双话题发布                               |
| [`zrobot_bridge`](src/zrobot_bridge/README.md)   | 驱动        | CAN 总线控制 23 个 RobStride 电机，4 路 CAN                           |
| [`zrobot_control`](src/zrobot_control/README.md) | 输入        | 键盘遥控，WASD 控制 + 150 ms 自动归零                                 |
| [`zrobot_joy`](src/zrobot_joy/README.md)         | 输入        | Xbox 手柄遥控，支持加速模式和使能开关                                 |
| [`zrobot_deploy`](src/zrobot_deploy/README.md)   | 控制        | FSM 系统：FixStand / Locomotion (ONNX) / PTLocomotion (.pt) / Damping |
| [`zrobot_gz_sim`](src/zrobot_gz_sim/README.md)   | 仿真        | Gazebo + ros2_control + PD 力矩桥接                                   |
| [`zrobot_mj_sim`](src/zrobot_mj_sim/README.md)   | 仿真        | MuJoCo + GLUT 渲染 + 力矩 / 位置双控制模式                            |

## 关键概念

### 统一服务接口

`zrobot_bridge`（实机）、`zrobot_gz_sim`、`zrobot_mj_sim` 三个包均提供相同的三个 ROS 2 服务，控制层通过服务名调用，无需关心底层是实机还是仿真：

| 服务                  | 请求                    | 响应                                                       |
| --------------------- | ----------------------- | ---------------------------------------------------------- |
| `/rob_stride_control` | `float32[23] positions` | `feedback_positions / velocities / torques / temperatures` |
| `/get_positions`      | （空）                  | `float32[23] feedback_positions`                           |
| `/set_zeros`          | （空）                  | `success + message`                                        |

### 23 电机与 12 驱动关节

机器人总定义 23 个关节，其中 12 个腿部关节（左右各 6：hip_roll / yaw / pitch + knee + foot_pitch / roll）由 RL 策略直接控制。上肢关节在实机中有电机但在仿真中固定，通过 `active_joint_count_` 处理差异。

### RL 推理

Locomotion 和 PTLocomotion 使用相同的 47 维观测空间和 12 维动作空间：

- **观测**：步态相位（2）+ 指令速度（3）+ 关节位置（12）+ 关节速度（12）+ 上一帧动作（12）+ 角速度（3）+ 欧拉角（3）
- **动作**：12 个腿部关节的位置增量，通过 `dof_indices_` 映射到 23 电机数组
- **控制频率**：100 Hz，推理在独立线程运行，不阻塞主循环
