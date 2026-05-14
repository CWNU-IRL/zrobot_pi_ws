# ZRobot Pi Workspace (`zrobot_pi_ws`)

基于 ROS 2 的 ZRobot 双足机器人控制与仿真工作空间，覆盖硬件驱动层（CAN 电机、IMU）到上层运动控制（FSM + 强化学习推理），并提供 Gazebo 仿真支持。

## 目录结构

```
zrobot_pi_ws/
├── src/                          # ROS 2 包源码
│   ├── rs_interface/             # 自定义服务接口 (srv)
│   ├── imu_msg/                  # 自定义 IMU 消息 (msg)
│   ├── imu_data_node/            # WIT 系列 IMU 驱动节点
│   ├── zrobot_bridge/            # RobStride 电机 CAN 总线控制桥接
│   ├── zrobot_gz_sim/            # Gazebo 仿真（URDF + ros2_control 桥接）
│   ├── zrobot_control/           # 电机控制客户端示例节点
│   └── zrobot_deploy/            # 运动控制部署（FSM + RL 推理）
├── scripts/                      # 工具脚本
│   └── pt2onnx.py                # TorchScript → ONNX 模型转换
├── third_party/                  # 预编译第三方库
│   ├── onnxruntime-linux-aarch64-1.16.3/   # ONNX Runtime (ARM64)
│   ├── onnxruntime-linux-aarch64-1.16.3.tgz
│   ├── libtorch/                            # LibTorch (x86_64)
│   └── libtorch-shared-with-deps-2.11.0+cu126.zip
├── install/                      # colcon 构建产出（含所有包）
├── build/                        # colcon 构建中间文件
├── log/                          # 构建日志
└── README.md                     # 本文件
```

## ROS 2 包总览

| 包名 | 类型 | 描述 |
|------|------|------|
| `rs_interface` | 接口 (srv) | 定义 3 个电机控制服务接口 |
| `imu_msg` | 接口 (msg) | 定义自定义 IMU 消息（含欧拉角） |
| `imu_data_node` | 驱动 | WIT 系列 IMU 数据采集（TTL/CAN/RS485） |
| `zrobot_bridge` | 驱动 | RobStride 电机 CAN 总线通信桥接 |
| `zrobot_gz_sim` | 仿真 | Gazebo 仿真环境 + 服务桥接 |
| `zrobot_control` | 示例 | 电机控制客户端示例 |
| `zrobot_deploy` | 控制 | FSM 运动控制系统 + RL 推理 |

### 包依赖关系

```
rs_interface  ◄── (srv 依赖) ──  zrobot_bridge, zrobot_gz_sim,
                                  zrobot_control, zrobot_deploy

imu_msg       ◄── (msg 依赖) ──  imu_data_node
```

---

## 各包详情

### 1. `rs_interface` — 电机控制服务接口

定义 3 个自定义 ROS 2 服务，作为硬件层与上层控制的统一接口。

| 服务 | 请求 | 响应 |
|------|------|------|
| `RobStrideMsgs` | `float32[23] positions` — 23 个电机目标位置 (rad) | `float32[23] feedback_positions/velocities/torques/temperatures` + `bool success` + `string message` |
| `SetZeros` | 无 | `bool success` + `string message` |
| `GetPositions` | 无 | `float32[23] feedback_positions` + `bool success` + `string message` |

**依赖**: `rosidl_default_generators`

---

### 2. `imu_msg` — 自定义 IMU 消息

| 消息 | 字段 |
|------|------|
| `ImuData` | `std_msgs/Header header` + `sensor_msgs/Imu imu` + `float64 roll/pitch/yaw` (rad) |

**依赖**: `std_msgs`, `sensor_msgs`, `geometry_msgs`, `rosidl_default_generators`

---

### 3. `imu_data_node` — IMU 数据采集驱动

基于 C++ 实现的 WIT 系列 IMU 采集节点。支持 **TTL**（11 字节数据包）、**CAN**（8 字节数据包）、**RS485**（Modbus RTU 轮询）三种协议。

**发布话题**:

| 话题 | 类型 | 频率 | 内容 |
|------|------|------|------|
| `/imu/data` | `sensor_msgs/Imu` | ~200 Hz | 加速度 (m/s²)、角速度 (rad/s)、四元数 |
| `/imu/ImuDataWithRPY` | `imu_msg/ImuData` | ~200 Hz | 上述 + roll/pitch/yaw (rad) |

**系统依赖**: `libserial-dev` (`sudo apt install libserial-dev`)

**运行**:
```bash
ros2 run imu_data_node imu_data_node
```

默认使用 `/dev/imu_usb`、2000000 波特率、TTL 协议。修改协议见源码 `main()` 函数。

---

### 4. `zrobot_bridge` — RobStride 电机 CAN 控制桥接

核心电机驱动节点，通过 **CAN 总线** 与最多 23 个 RobStride 电机直接通信，实现完整底层协议（运控模式、位置模式、零位设置、参数读写等）。

**提供的服务**:

| 服务名 | 类型 |
|--------|------|
| `/motor_controller_node/rob_stride_control` | `rs_interface/RobStrideMsgs` |
| `/motor_controller_node/set_zeros` | `rs_interface/SetZeros` |
| `/motor_controller_node/get_positions` | `rs_interface/GetPositions` |

**支持的电机型号**:

| 类型值 | 型号 | 最大角度 | 最大速度 | 最大扭矩 |
|--------|------|---------|---------|---------|
| 0 | ROBSTRIDE_00 | 4π rad | 50 rad/s | 17 Nm |
| 1 | ROBSTRIDE_01 | 4π rad | 44 rad/s | 17 Nm |
| 2 | ROBSTRIDE_02 | 4π rad | 44 rad/s | 17 Nm |
| 3 | ROBSTRIDE_03 | 4π rad | 50 rad/s | 60 Nm |
| 4 | ROBSTRIDE_04 | 4π rad | 15 rad/s | 120 Nm |
| 5 | ROBSTRIDE_05 | 4π rad | 33 rad/s | 17 Nm |
| 6 | ROBSTRIDE_06 | 4π rad | 20 rad/s | 60 Nm |

**配置文件**: `config/motor_config.yaml` — 配置主机 ID、23 个电机的 CAN ID、电机类型、CAN 接口、KP/KD 参数。

**运行**:
```bash
# 启动 CAN 接口
sudo bash src/zrobot_bridge/scripts/setup_can_interfaces.sh 1000000

# 启动电机控制节点
ros2 launch zrobot_bridge motor_controller.launch.py
```

---

### 5. `zrobot_gz_sim` — Gazebo 仿真包

使用 **23 关节 URDF** + **ros2_control** 构建的 Gazebo 仿真环境。通过 `gazebo_motor_bridge_node` 提供与真实硬件完全相同的 3 个服务接口，使上层控制程序无需修改即可在仿真与实物之间切换。

**核心特性**:
- 23 个 revolute 关节 + IMU 传感器
- `joint_state_broadcaster` + `forward_command_controller` 组成的 ros2_control 架构
- `ros_gz_bridge` 桥接仿真时钟 (`/clock`) 和 IMU 数据 (`/imu/data`)
- `gazebo_motor_bridge_node` 接收服务请求并转换为关节力矩命令（PD 控制）

**配置文件**:
- `config/controllers.yaml` — ros2_control 控制器参数
- `config/bridge_params.yaml` — 关节名称、KP/KD、反馈温度

**资源**:
- `resources/zrobot/urdf/zrobot.urdf` — 机器人 URDF 模型
- `resources/zrobot/meshes/` — 3D 网格文件
- `resources/zrobot/mjcf/` — MuJoCo 模型（可选）
- `worlds/empty.sdf` — 仿真世界

**运行**:
```bash
ros2 launch zrobot_gz_sim sim_bringup.launch.py
```

**验证服务**:
```bash
ros2 service list | grep -E 'rob_stride_control|get_positions|set_zeros'
```

---

### 6. `zrobot_deploy` — 运动控制部署

基于 **有限状态机 (FSM)** 的运动控制系统，支持键盘交互切换多种控制模式。

**FSM 模式**:

| 按键 | 模式 | 描述 |
|------|------|------|
| `F` | FixStand | 缓慢移动至机械零位并保持（3 秒线性插值） |
| `L` | Locomotion | ONNX 推理强化学习运动控制（47 维观测 → 12 维动作） |
| `T` | PTLocomotion | TorchScript (.pt) 推理强化学习运动控制 |
| `D` | Damping | 仿真阻尼模式（通过位置指令近似阻尼） |
| `S` | Stop | 停止当前状态机 |
| `Q` | Quit | 退出程序 |

**架构**:

```
FSM (基类)
├── FixStand        // 机械零位站立
├── Locomotion      // ONNX RL 推理行走
├── PTLocomotion    // TorchScript RL 推理行走
└── Damping         // 仿真阻尼
```

**RL 模型规格**:
- 观测维度: 47 (单帧) × N (历史帧堆叠) = 模型输入
- 动作维度: 12（映射到 23 个电机中的 12 个主动自由度）
- 控制频率: 50–100 Hz（可配置）

**依赖**: `rclcpp`, `rs_interface`, `Eigen3`, `LibTorch` (仅 PTLocomotion), `ONNX Runtime` (仅 Locomotion), `tf2`

**运行**:
```bash
ros2 run zrobot_deploy main
```

---

### 7. `zrobot_control` — 示例客户端

简单的电机控制客户端，演示如何通过 `RobStrideMsgs` 服务控制电机。每 5 秒发送一次请求，每次将反馈位置 +1 作为新目标位置。

**运行**:
```bash
ros2 run zrobot_control rob_stride_client_node
```

---

## 系统依赖

### ROS 2

- **推荐**: ROS 2 Humble (Ubuntu 22.04) 或 Jazzy (Ubuntu 24.04)
- 必需包: `rclcpp`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `rosidl_default_generators`
- 仿真额外: `ros-gz-sim`, `ros-gz-bridge`, `gz-ros2-control`, `controller-manager`, `robot-state-publisher`, `xacro`, `ros2-controllers`

### 系统包

```bash
sudo apt install -y \
    libserial-dev \
    can-utils \
    libeigen3-dev
```

### 第三方库 (预编译于 `third_party/`)

| 库 | 架构 | 版本 | 用途 |
|----|------|------|------|
| ONNX Runtime | aarch64 | 1.16.3 | Locomotion RL 推理 |
| LibTorch | x86_64 | 2.11.0+cu126 | PTLocomotion RL 推理 |

> 在 ARM64 (Jetson/树莓派) 平台上，LibTorch 自动从系统 Python (`~/.local/lib/python3.10/site-packages/torch`) 加载。

---

## 构建

### 1. 安装 ROS 2 依赖

```bash
cd /home/c112/Codes/zrobot_pi_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 构建全部包

```bash
colcon build --symlink-install
```

### 3. 按需构建

```bash
# 仅构建接口包
colcon build --packages-select rs_interface imu_msg

# 构建驱动层
colcon build --packages-select zrobot_bridge imu_data_node

# 构建控制层
colcon build --packages-select zrobot_deploy

# 构建仿真
colcon build --packages-select zrobot_gz_sim
```

### 4. 加载环境

```bash
source install/setup.bash
```

---

## 使用流程

### 实物部署

```bash
# 1. 配置 CAN 接口
sudo bash src/zrobot_bridge/scripts/setup_can_interfaces.sh 1000000

# 2. 启动环境
source install/setup.bash

# 3. 启动电机桥接
ros2 launch zrobot_bridge motor_controller.launch.py

# 4. (可选) 启动 IMU
ros2 run imu_data_node imu_data_node

# 5. 启动运动控制
ros2 run zrobot_deploy main
# 按 F 进入站立，按 L 开始行走
```

### 仿真

```bash
source install/setup.bash
ros2 launch zrobot_gz_sim sim_bringup.launch.py

# 另开终端，运行控制
source install/setup.bash
ros2 run zrobot_deploy main
# 按 D 进入阻尼模式（仿真推荐），按 F 站立
```

---

## 模型转换

`scripts/pt2onnx.py` 用于将 PyTorch JIT 模型 (`.pt`) 转换为 ONNX (`.onnx`)，并自动验证输出一致性。

```bash
python3 scripts/pt2onnx.py \
    --jit_model policy_1.pt \
    --onnx_model policy_1.onnx \
    --obs_dim 47 \
    --opset 13
```

---

## 常见问题

### CAN 接口未就绪
```bash
# 检查 CAN 接口
ip link show | grep can
# 手动启用
sudo ip link set can0 up type can bitrate 1000000
```

### 串口权限不足 (IMU)
```bash
sudo usermod -a -G dialout $USER
# 或临时赋权
sudo chmod 666 /dev/ttyUSB0
```

### Gazebo 仿真缺少依赖
```bash
# Humble
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-gz-ros2-control

# Jazzy
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-ros2-control
```

### 控制器管理器未找到 (仿真)
仿真启动脚本会自动检测 `controller_manager`，若缺失则禁用控制器 spawner 并打印警告，不影响核心功能。

---

## 许可证

- `rs_interface`, `zrobot_bridge`, `zrobot_gz_sim`, `zrobot_control`, `imu_data_node`: Apache-2.0
- `imu_msg`, `zrobot_deploy`: 待声明

## 维护者

root <callmebill@billw.cn>
