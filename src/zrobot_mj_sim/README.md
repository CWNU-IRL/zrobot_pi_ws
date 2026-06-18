# zrobot_mj_sim

## 概述

ZRobot 的 MuJoCo 仿真桥接包。实现 `rs_interface` 的三个服务接口，加载 MJCF 格式机器人模型，使用 MuJoCo 物理引擎进行实时仿真。提供两种控制模式：力矩 PD 控制（默认）和直接位置控制。包含独立的 GLUT 3D 渲染线程，支持鼠标 / 键盘交互查看。

## 依赖

### 系统依赖

- `freeglut3-dev`（GLUT 渲染）
- `libgl-dev`（OpenGL）

### 第三方预编译库

- MuJoCo 3.9.0（`third_party/mujoco-3.9.0/`）

### ROS 2

- `rclcpp`
- `rs_interface`
- `sensor_msgs`
- `std_msgs`
- `rosgraph_msgs`
- `ament_index_cpp`

## 构建

```bash
sudo apt install freeglut3-dev
colcon build --packages-select rs_interface zrobot_mj_sim
```

## 使用

### 启动

```bash
ros2 launch zrobot_mj_sim mujoco_bringup.launch.py

# 另一终端启动 FSM 控制器
source install/setup.bash
ros2 run zrobot_deploy main
```

### 渲染窗口操作

| 操作     | 鼠标 / 按键 |
| -------- | ----------- |
| 旋转视角 | 左键拖动    |
| 平移视角 | 中键拖动    |
| 缩放     | 右键拖动    |
| 退出仿真 | `ESC`       |
| 重置仿真 | `r`         |
| 重置相机 | `c`         |

## ROS 2 接口

### 提供的服务

| 服务名                | 服务类型                         | 说明                                  |
| --------------------- | -------------------------------- | ------------------------------------- |
| `/rob_stride_control` | `rs_interface/srv/RobStrideMsgs` | 接收位置目标，PD 或位置控制后返回反馈 |
| `/get_positions`      | `rs_interface/srv/GetPositions`  | 读取仿真关节位置（减去零位偏移）      |
| `/set_zeros`          | `rs_interface/srv/SetZeros`      | 当前仿真位置归零校准                  |

### 发布的话题

| 话题名          | 类型                         | 频率   | 说明                                    |
| --------------- | ---------------------------- | ------ | --------------------------------------- |
| `/joint_states` | `sensor_msgs/msg/JointState` | 200 Hz | 关节位置、速度、力矩反馈                |
| `/imu/data`     | `sensor_msgs/msg/Imu`        | 200 Hz | 仿真 IMU 数据（方向 / 角速度 / 加速度） |
| `/clock`        | `rosgraph_msgs/msg/Clock`    | 200 Hz | 仿真时间                                |

### 参数

| 参数名                 | 类型       | 默认值        | 说明                                 |
| ---------------------- | ---------- | ------------- | ------------------------------------ |
| `joint_names`          | string[12] | 12 个腿部关节 | 受控关节列表                         |
| `kp`                   | double[12] | 见 YAML       | PD Kp 值                             |
| `kd`                   | double[12] | 见 YAML       | PD Kd 值                             |
| `control_mode`         | string     | `torque_pd`   | 控制模式（`torque_pd` / `position`） |
| `control_frequency`    | double     | 200.0         | 控制频率 (Hz)                        |
| `feedback_temperature` | float      | 35.0          | 仿真反馈温度                         |
| `publish_joint_states` | bool       | true          | 是否发布 `/joint_states`             |
| `publish_imu`          | bool       | true          | 是否发布 `/imu/data`                 |
| `publish_clock`        | bool       | true          | 是否发布 `/clock`                    |

## 项目结构

```
zrobot_mj_sim/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── mujoco_bridge_params.yaml     # 节点参数
├── launch/
│   └── mujoco_bringup.launch.py      # 仿真启动
├── resources/zrobot/
│   ├── mjcf/
│   │   ├── zrobot.xml                # 力矩 PD 控制模型
│   │   └── zrobot_position.xml       # 位置控制模型
│   └── meshes/
│       ├── *.STL                     # 24 个网格文件
│       └── zrobot.urdf / .xml
├── include/zrobot_mj_sim/
│   └── mujoco_motor_bridge_node.hpp
└── src/
    └── mujoco_motor_bridge_node.cpp
```

## 技术细节

### 控制模式

| 模式                | 原理                                                | 模型文件              |
| ------------------- | --------------------------------------------------- | --------------------- |
| `torque_pd`（默认） | C++ PD 公式计算力矩，写入 `<motor>` 执行器          | `zrobot.xml`          |
| `position`          | 直接写目标位置，MuJoCo 内置 `<position>` 执行器伺服 | `zrobot_position.xml` |

PD 公式：`tau = (target_q - current_q) * kp + (0 - current_dq) * kd`

### MuJoCo 集成

- 使用 `mj_loadXML()` 加载 MJCF 模型
- 每个控制周期调用 `mj_step()` 推进物理仿真，通过 `sim_substeps_` 控制每步内的物理子步数
- 关节映射：通过 `mj_name2id()` 将 ROS 关节名称转换为 MuJoCo 内部的 joint / qpos / dof / actuator ID
- 传感器：从 MuJoCo 传感器数据缓冲中读取 IMU 数据（四元数方向、角速度、线加速度）

### 渲染

- 在独立的 GLUT 线程中运行 60 FPS 渲染循环
- 使用 MuJoCo 原生的 `mjvCamera` / `mjvScene` / `mjrContext` 渲染管线
- 鼠标交互通过 `glutMouseFunc` 和 `glutMotionFunc` 映射到 `mjvCamera` 的平移 / 旋转 / 缩放操作
- 线程安全：通过 `state_mutex_` 保护 MuJoCo 模型 / 数据结构的跨线程访问

### 仿真时间

- 仿真时间步长 = `1.0 / control_frequency / sim_substeps_`
- 每步推进后通过 `current_sim_time_locked()` 计算 ROS 时间
- 发布 `/clock` 实现与 ROS 节点的时间同步

### MJCF 模型

- **浮动基座**（freejoint）：6-DOF 自由运动
- **12 个腿部关节**：左右各 6，均使用 revolute 类型
- **执行器**：
  - `zrobot.xml`：12 个 `<motor>` 执行器，接收 PD 计算的力矩
  - `zrobot_position.xml`：12 个 `<position>` 执行器，直接接收位置指令
- **传感器**：`framequat`（方向）、`velocimeter`（角速度）、`accelerometer`（线加速度）
- **网格**：24 个 STL 文件，通过 `meshdir` 指定路径
