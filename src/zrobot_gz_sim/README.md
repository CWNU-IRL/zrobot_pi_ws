# zrobot_gz_sim

## 概述

ZRobot 的 Gazebo 仿真桥接包。实现 `rs_interface` 的三个服务接口，连接 Gazebo 仿真环境中通过 `ros2_control` 和 `joint_group_effort_controller` 控制的机器人模型。在 200 Hz 控制循环中通过 PD 公式将位置目标转换为力矩指令，输出到仿真关节。

包含完整的 URDF 机器人模型（含 12 个驱动关节、24 个 STL 碰撞 / 视觉网格）和 Gazebo 启动配置。

## 依赖

### ROS 2

- `rclcpp`
- `rs_interface`（自定义服务接口）
- `sensor_msgs`
- `std_msgs`
- `controller_manager`
- `gz_ros2_control`（Gazebo - ROS 2 控制桥接）
- `ros_gz_bridge`（Gazebo - ROS 2 话题桥接）
- `robot_state_publisher`
- `xacro`

### 安装

```bash
# Jazzy
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control ros-jazzy-ros2controllers

# Humble
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
    ros-humble-gz-ros2-control ros-humble-ros2controllers
```

## 构建

```bash
colcon build --packages-select rs_interface zrobot_gz_sim
```

## 使用

```bash
ros2 launch zrobot_gz_sim sim_bringup.launch.py

# 另一终端启动 FSM 控制器
source install/setup.bash
ros2 run zrobot_deploy main
```

### 启动文件说明

`sim_bringup.launch.py` 按顺序启动：

1. Gazebo 仿真器（空世界，`empty.sdf`）
2. `robot_state_publisher`（发布 URDF 的 TF 树）
3. 机器人加载到 Gazebo（通过 `spawn_entity.py`）
4. 控制器管理器加载（`joint_state_broadcaster`, `joint_group_effort_controller`）
5. ROS-Gazebo 桥接（`/clock`, `/imu/data` 等话题）
6. `gazebo_motor_bridge_node`（本包的核心节点）

## ROS 2 接口

### 提供的服务

| 服务名                | 服务类型                         | 说明                                    |
| --------------------- | -------------------------------- | --------------------------------------- |
| `/rob_stride_control` | `rs_interface/srv/RobStrideMsgs` | 接收位置目标，PD 计算力矩发送到仿真关节 |
| `/get_positions`      | `rs_interface/srv/GetPositions`  | 读取仿真关节当前位置                    |
| `/set_zeros`          | `rs_interface/srv/SetZeros`      | 当前仿真位置归零校准                    |

### 订阅的话题

| 话题名          | 类型                         | 说明                            |
| --------------- | ---------------------------- | ------------------------------- |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo 发布的关节状态（200 Hz） |

### 发布的话题

| 话题名                                    | 类型                             | 说明                         |
| ----------------------------------------- | -------------------------------- | ---------------------------- |
| `/joint_group_effort_controller/commands` | `std_msgs/msg/Float64MultiArray` | PD 计算后的力矩指令（12 维） |

## 项目结构

```
zrobot_gz_sim/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── controllers.yaml          # 控制器配置
│   └── bridge_params.yaml        # 桥接节点参数
├── launch/
│   └── sim_bringup.launch.py     # 完整仿真启动
├── worlds/
│   └── empty.sdf                 # 空仿真世界
├── resources/zrobot/
│   ├── urdf/
│   │   ├── zrobot.urdf           # URDF 模型（1410 行）
│   │   └── zrobot.csv            # 惯性参数
│   ├── mjcf/
│   │   └── zrobot.xml            # MuJoCo 兼容 MJCF
│   └── meshes/
│       ├── *.STL                 # 24 个网格文件
│       └── zrobot.urdf / .xml    # 渲染模型引用
├── include/zrobot_gz_sim/
│   └── gazebo_motor_bridge_node.hpp
└── src/
    └── gazebo_motor_bridge_node.cpp
```

## 配置

### bridge_params.yaml

| 参数                   | 类型       | 默认值        | 说明                 |
| ---------------------- | ---------- | ------------- | -------------------- |
| `joint_names`          | string[12] | 12 个腿部关节 | 受控关节列表         |
| `kp`                   | double[12] | 见 YAML       | 12 个关节的 PD Kp 值 |
| `kd`                   | double[12] | 见 YAML       | 12 个关节的 PD Kd 值 |
| `feedback_temperature` | float      | 35.0          | 仿真反馈温度（常量） |

### controllers.yaml

| 控制器                          | 类型                       | 说明                           |
| ------------------------------- | -------------------------- | ------------------------------ |
| `joint_state_broadcaster`       | `JointStateBroadcaster`    | 发布关节状态到 `/joint_states` |
| `joint_group_effort_controller` | `ForwardCommandController` | 接收力矩指令，应用到仿真关节   |

## 技术细节

### PD 控制

```
tau = (target_q - current_q) * kp + (0 - current_dq) * kd
```

不包含目标速度项（target_dq 始终为 0），因为 `zrobot_deploy` 发送的是位置目标而非轨迹。

### 200 Hz 控制保持

桥接节点内部以 200 Hz 定时器运行 `control_loop()`，在没有新服务请求时将上一帧的目标位置持续写到仿真关节。这确保 `rs_interface` 服务端接收位置目标后，即使 `zrobot_deploy` 切换到不同状态机，仿真关节也不会掉回零位。

### URDF 模型

- **浮动基座**（6-DOF）通过 `gz_ros2_control/GazeboSimSystem` 硬件接口连接
- **12 个驱动关节**：左右各 6（hip_roll, hip_yaw, hip_pitch, knee, foot_pitch, foot_roll）
- **固定关节**：双臂、胸部、头部固定在 T-pose
- **24 个 STL 网格**：碰撞和视觉使用同一网格，路径通过 `package://zrobot_gz_sim/...` 引用
