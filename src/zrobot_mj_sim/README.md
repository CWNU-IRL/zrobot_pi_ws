# zrobot_mj_sim

## 概述

zrobot 双足机器人的 MuJoCo 仿真桥接包。`MujocoMotorBridgeNode` 加载 MJCF 模型文件，运行物理仿真，并通过 ROS 2 服务接口暴露与实机 `zrobot_bridge` 完全一致的控制方式。支持力矩控制（默认，用户空间 PD）和位置控制（MuJoCo 内置位置伺服）两种模式。包含 GLUT 渲染窗口用于可视化。

## 依赖

### 系统依赖
- MuJoCo 3.9.0（从 `../../third_party/mujoco-3.9.0/` 获取）
- GLUT、OpenGL

### ROS 2
- `rclcpp`
- `sensor_msgs`
- `std_msgs`
- `rosgraph_msgs`
- `ament_index_cpp`
- `rs_interface`（自定义服务定义包）

## 构建

```bash
colcon build --packages-select rs_interface zrobot_mj_sim
```

## 使用

```bash
# 使用仿真时间启动
ros2 launch zrobot_mj_sim mujoco_bringup.launch.py

# 不使用仿真时间
ros2 launch zrobot_mj_sim mujoco_bringup.launch.py use_sim_time:=false
```

启动后：
1. 加载 MJCF 模型，机器人从 0.8m 高度自由落体到地面
2. GLUT 窗口显示仿真画面（1200×900）
3. `/rob_stride_control` 等服务就绪

### 测试控制

```bash
# 发送站立指令
ros2 service call /rob_stride_control rs_interface/srv/RobStrideMsgs \
  "{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

结合 `zrobot_deploy` 运行完整运动控制：

```bash
# 终端 1：仿真
ros2 launch zrobot_mj_sim mujoco_bringup.launch.py

# 终端 2：FSM 控制器
ros2 run zrobot_deploy main
```

### 渲染窗口快捷键

| 按键 | 功能 |
|------|------|
| ESC | 退出仿真 |
| `r` | 重置仿真 |
| `c` | 重置相机视角 |

鼠标左键拖拽旋转视野，滚轮缩放，中键平移。

## ROS 2 接口

### 提供的服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/rob_stride_control` | `rs_interface/srv/RobStrideMsgs` | 发送 23 关节位置命令，返回完整反馈 |
| `/get_positions` | `rs_interface/srv/GetPositions` | 读取当前关节位置 |
| `/set_zeros` | `rs_interface/srv/SetZeros` | 设置零位偏移 |

### 发布的话题

| 话题名 | 类型 | 频率 | 说明 |
|--------|------|------|------|
| `/joint_states` | `sensor_msgs/msg/JointState` | 200 Hz | 关节位置/速度/力矩 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 200 Hz | 机身 IMU（四元数、角速度、线加速度） |
| `/clock` | `rosgraph_msgs/msg/Clock` | 200 Hz | 仿真时间（支持 `use_sim_time`） |

### 节点参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `joint_names` | string[] | 23 个默认名 | 关节名称列表（前 12 个为驱动关节） |
| `kp` | double[] | 40.0 | 各关节 PD 比例增益 |
| `kd` | double[] | 1.0 | 各关节 PD 微分增益 |
| `control_mode` | string | "torque_pd" | 控制模式：`torque_pd` 或 `position` |
| `control_frequency` | double | 200.0 | 控制循环频率（Hz） |
| `sim_substeps` | int | 8 | 每步 `mj_step()` 迭代次数 |
| `feedback_temperature` | double | 35.0 | 模拟电机温度反馈 |

## 项目结构

```
zrobot_mj_sim/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── mujoco_bridge_params.yaml    # 默认参数配置
├── include/zrobot_mj_sim/
│   └── mujoco_motor_bridge_node.hpp
├── launch/
│   └── mujoco_bringup.launch.py     # 启动文件
├── resources/zrobot/
│   ├── mjcf/
│   │   ├── zrobot.xml               # 力矩模式模型（<motor> 执行器）
│   │   └── zrobot_position.xml      # 位置模式模型（<position> 执行器）
│   └── meshes/
│       ├── *.STL                    # 22 个 STL 网格文件
│       └── zrobot.urdf              # 原始 SolidWorks 导出 URDF
└── src/
    └── mujoco_motor_bridge_node.cpp # 完整实现（854 行）
```

## 技术细节

### 控制模式

| 模式 | MJCF 执行器类型 | 控制方式 | 默认激活 |
|------|---------------|---------|---------|
| `torque_pd` | `<motor>` | C++ PD 计算力矩写入 `data_->ctrl` | 是 |
| `position` | `<position>` | 直接写入目标位置，MuJoCo 内置伺服 | 否 |

**力矩 PD 公式**：
```
tau = (target_q - current_q) * kp + (0 - current_dq) * kd
data_->ctrl[actuator_id] = tau
```

### 仿真循环

由 ROS 2 定时器驱动，每次触发执行：

```
1. apply_control_locked()   # 将目标位置→PD 力矩写入 data_->ctrl
2. mj_step() × sim_substeps_ # 推进物理仿真
3. update_state_from_sim_locked()  # 读取关节/IMU 状态
4. 发布 /joint_states, /imu/data, /clock
```

### 关节映射

使用 `mj_name2id()` 将 ROS 关节名称映射到 MuJoCo 的 `qpos`、`dof`、`actuator` 索引：

```
关节名 → mj_name2id(model, mjOBJ_JOINT, name)  → qpos_adr
       → mj_name2id(model, mjOBJ_ACTUATOR, name) → actuator_id
```

### 23 vs 12 关节

MJCF 中定义了 23 个关节（12 腿部驱动 + 5 上肢被动 + 浮动基座 + 其他），但仅 12 个腿部关节配置了执行器。代码通过 `active_joint_count_` 区分：仅对 `actuator_id != -1` 的关节执行 PD 控制。

### 零位偏移

与 `zrobot_bridge` 和 `zrobot_gz_sim` 一致的偏移机制：`/set_zeros` 记录当前关节位置作为零位偏移，此后上位机位置指令会自动加上偏移量，反馈位置则减去偏移量。

### GLUT 渲染

在独立线程中运行 GLUT 主循环，使用 `mjr_render` 绘制场景。支持鼠标交互和键盘快捷键。渲染不会阻塞仿真控制循环。
