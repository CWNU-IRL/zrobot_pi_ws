# zrobot_gz_sim

## 概述

zrobot 双足机器人的 Gazebo 仿真包。提供完整的仿真环境，包含机器人 URDF 模型、ros2_control 集成、以及 `GazeboMotorBridgeNode` 桥接节点。该节点将 `zrobot_deploy` 等上层控制节点的位置指令转换为 PD 力矩控制命令，通过 `forward_command_controller` 驱动 12 个腿部关节。暴露的服务接口与 `zrobot_bridge`（实机 CAN 驱动）完全一致，实现仿真与实机的无缝切换。

## 依赖

### ROS 2
- `rclcpp`
- `sensor_msgs`
- `std_msgs`
- `rs_interface`（自定义服务定义包）

### 运行时依赖
- `controller_manager`
- `gz_ros2_control`
- `robot_state_publisher`
- `ros_gz_bridge`
- `ros_gz_sim`
- `xacro`

## 构建

```bash
colcon build --packages-select rs_interface zrobot_gz_sim
```

## 使用

```bash
# 启动 Gazebo 仿真环境
ros2 launch zrobot_gz_sim sim_bringup.launch.py
```

启动后：
1. Gazebo 加载 empty world
2. 机器人以 1.05m 高度生成在地面上
3. `robot_state_publisher` 发布机器人 TF 和关节状态
4. `GazeboMotorBridgeNode` 启动，等待控制服务请求
5. 若 `controller_manager` 可用，自动加载 `joint_state_broadcaster` 和 `joint_group_effort_controller`

### 测试控制

```bash
# 发送站立指令（全零位置）
ros2 service call /rob_stride_control rs_interface/srv/RobStrideMsgs \
  "{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# 读取关节位置
ros2 service call /get_positions rs_interface/srv/GetPositions
```

结合 `zrobot_deploy` 运行完整运动控制：

```bash
# 终端 1：仿真
ros2 launch zrobot_gz_sim sim_bringup.launch.py

# 终端 2：FSM 控制器
ros2 run zrobot_deploy main
```

## ROS 2 接口

### 提供的服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/rob_stride_control` | `rs_interface/srv/RobStrideMsgs` | 发送 23 关节位置命令，PD 计算力矩 |
| `/get_positions` | `rs_interface/srv/GetPositions` | 读取当前关节位置 |
| `/set_zeros` | `rs_interface/srv/SetZeros` | 设置零位偏移 |

### 桥接话题

| 话题名 | 类型 | 方向 | 说明 |
|--------|------|------|------|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS | 仿真时间 |
| `/imu/data` | `sensor_msgs/msg/Imu` | Gazebo → ROS | IMU 数据 |

### 发布的话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/joint_group_effort_controller/commands` | `std_msgs/Float64MultiArray` | 12 关节力矩命令（PD 输出） |

## 项目结构

```
zrobot_gz_sim/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── bridge_params.yaml       # GazeboMotorBridgeNode 参数（关节名、Kp/Kd、模拟温度）
│   └── controllers.yaml         # ros2_control 控制器配置（200 Hz）
├── include/zrobot_gz_sim/
│   └── gazebo_motor_bridge_node.hpp
├── launch/
│   └── sim_bringup.launch.py    # 主启动文件
├── src/
│   └── gazebo_motor_bridge_node.cpp
├── worlds/
│   └── empty.sdf                # 空世界（含物理参数、IMU 传感器、光照）
└── resources/zrobot/
    ├── meshes/
    │   ├── *.STL                # 22 个 STL 网格文件（SolidWorks 导出）
    │   └── zrobot.urdf          # 原始 23 自由度 URDF（所有关节 revolue）
    └── urdf/
        ├── zrobot.urdf          # Gazebo 适配 URDF（12 驱动关节 + 固定上肢 + IMU + 浮动基座）
        └── zrobot.csv           # SolidWorks 导出参数表
```

## 技术细节

### 机器人模型

- **总质量**：~24.5 kg
- **生成高度**：1.05 m（浮动基座）
- **驱动关节**：12 个（左右腿各 6 个，hip_roll/yaw/pitch + knee + foot_pitch/roll）
- **固定关节**：上肢 5 个（chest_head、shoulder_pitch/roll、arm_yaw、elbow_pitch）
- **IMU**：固定在 `base_link` 上，200 Hz 发布，带高斯噪声

### PD 控制器

```
tau = (target_q - current_q) * kp + (0 - current_dq) * kd
```

默认增益（`config/bridge_params.yaml`）：

| 关节组 | Kp | Kd |
|--------|----|----|
| hip_roll/yaw | 40.0 | 2.0 |
| hip_pitch/knee | 60.0 | 3.0 |
| foot_pitch/roll | 10.0 | 1.0 |

### 23 vs 12 关节

桥接节点声明 `kNumMotors = 23`，但 URDF 中仅 12 个驱动关节。**前 12 个**位置索引对应实际腿部驱动关节（6L + 6R），索引 12-22 的位置值被记录但不发送到控制器。代码通过 `active_joint_count_` 处理这种情况，仅对实际存在的关节执行 PD 控制。

### 控制循环

200 Hz 定时器持续保持目标位置。即使没有新的服务请求，节点也会以最近一次的目标位置持续下发 PD 力矩，确保机器人维持站立。

### 启动流程

```
sim_bringup.launch.py
├── Gazebo（empty.sdf）
├── robot_state_publisher（zrobot.urdf）
├── Spawn Entity（z=1.05m）
├── ros_gz_bridge（/clock, /imu/data）
└── GazeboMotorBridgeNode
    └── [可选] controller_manager → joint_state_broadcaster → joint_group_effort_controller
```
