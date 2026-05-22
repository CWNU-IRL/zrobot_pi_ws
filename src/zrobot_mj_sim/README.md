# zrobot_mj_sim

基于 MuJoCo 的 zrobot 双足机器人仿真桥接包。提供与实物驱动包 `zrobot_bridge` **完全相同的 ROS 2 服务接口**（`/rob_stride_control`、`/get_positions`、`/set_zeros`），同时发布 `/joint_states` 与 `/imu/data` 仿真数据，可直接对接 `zrobot_deploy` 状态机进行运动控制算法的仿真验证。

---

## 目录结构

```
src/zrobot_mj_sim/
├── CMakeLists.txt                    # 构建配置（自动查找 MuJoCo）
├── package.xml                       # 包依赖声明
├── README.md                         # 本文件
├── config/
│   └── mujoco_bridge_params.yaml     # 默认参数配置
├── launch/
│   └── mujoco_bringup.launch.py      # 启动文件
├── include/zrobot_mj_sim/
│   └── mujoco_motor_bridge_node.hpp  # 节点头文件
├── src/
│   └── mujoco_motor_bridge_node.cpp  # 节点实现
└── resources/zrobot/
    ├── mjcf/
    │   ├── zrobot.xml                # 力矩模式 MJCF（<motor> actuator）
    │   └── zrobot_position.xml       # 位置模式 MJCF（<position> actuator）
    └── meshes/                       # 机器人 mesh 文件（STL）
        ├── base_link.STL
        ├── L_hip_roll_link.STL
        └── ...
```

---

## 功能

| 类别 | 名称 | 说明 |
|------|------|------|
| 服务 | `/rob_stride_control` | 接收 23 维位置指令，返回位置/速度/力矩/温度反馈 |
| 服务 | `/get_positions` | 获取当前 23 个关节位置 |
| 服务 | `/set_zeros` | 将当前关节位置设为零点偏移 |
| 话题 | `/joint_states` | 传感器_msgs/JointState，200 Hz 发布 |
| 话题 | `/imu/data` | 传感器_msgs/Imu，含姿态四元数、角速度、线加速度 |
| 话题 | `/clock` | 可选，发布 MuJoCo 内部仿真时间 |

### 控制模式

**torque_pd 模式（默认）**：在应用层计算 PD 力矩 $`\tau = K_p(q_{des} - q) + K_d(0 - \dot{q})`$，通过 MuJoCo `<motor>` 执行器施加于关节。与 `zrobot_gz_sim` 的 `gazebo_motor_bridge_node` 行为一致。

**position 模式**：将目标位置直接写入 `mjData->ctrl`，使用 MuJoCo 内置 `<position>` 执行器完成伺服控制；MJCF 使用 `zrobot_position.xml`，各关节设定了适当的 `kp`/`kv` 参数和限位范围。

> 两种模式均可通过参数 `control_mode` 切换到。`zrobot_deploy` 的 `FSM::sendMotorPositions()` 无需任何修改即可对接。

---

## 依赖

### 必需

- **ROS 2 Humble**（或其他兼容的 ROS 2 发行版）
- **MuJoCo 3.x+**（头文件和共享库）

### 安装 MuJoCo

**方法一（源码编译，推荐）**：

```bash
# 克隆 MuJoCo
git clone https://github.com/google-deepmind/mujoco.git -b main
cd mujoco
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
sudo cmake --build . --target install
```

**方法二（预编译包）**：

从 [MuJoCo Releases](https://github.com/google-deepmind/mujoco/releases) 下载 Linux 预编译包，解压后设置环境变量：

```bash
export MUJOCO_ROOT=/path/to/mujoco   # 包含 include/ 和 lib/ 的目录
```

> 构建系统通过 `MUJOCO_ROOT` 环境变量查找 MuJoCo。如果 MuJoCo 安装在 `/usr/local`，通常无需额外设置。

---

## 构建

```bash
cd /home/c112/Codes/zrobot_pi_ws
colcon build --packages-select zrobot_mj_sim
source install/setup.bash
```

如果 MuJoCo 未在标准路径，先设置：

```bash
export MUJOCO_ROOT=/path/to/mujoco
colcon build --packages-select zrobot_mj_sim
```

---

## 启动

### 启动仿真桥接节点

```bash
ros2 launch zrobot_mj_sim mujoco_bringup.launch.py
```

### 直接运行节点（可临时覆盖参数）

```bash
# 使用默认参数（torque_pd 模式）
ros2 run zrobot_mj_sim mujoco_motor_bridge_node

# 切换到位置控制模式
ros2 run zrobot_mj_sim mujoco_motor_bridge_node --ros-args -p control_mode:=position

# 指定自定义 MJCF 并降低控制频率
ros2 run zrobot_mj_sim mujoco_motor_bridge_node \
  --ros-args -p model_path:=/path/to/custom_model.xml \
             -p control_frequency:=100.0
```

---

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `joint_names` | string[] | 12 个腿部关节 | 活跃关节名称列表，顺序对应服务数组索引 |
| `kp` | double[] | 见 yaml | 关节 PD 比例增益，torque_pd 模式使用 |
| `kd` | double[] | 见 yaml | 关节 PD 微分增益，torque_pd 模式使用 |
| `control_mode` | string | `"torque_pd"` | 控制模式：`"torque_pd"` 或 `"position"` |
| `control_frequency` | double | 200.0 | 控制循环频率（Hz），决定仿真步进批次大小 |
| `feedback_temperature` | double | 35.0 | 反馈温度值（模拟真实电机温度） |
| `publish_joint_states` | bool | true | 是否发布 `/joint_states` |
| `publish_imu` | bool | true | 是否发布 `/imu/data` |
| `publish_clock` | bool | true | 是否发布 `/clock` |
| `base_frame` | string | `"base_link"` | 关节状态消息的 frame_id |
| `imu_frame` | string | `"imu"` | IMU 消息的 frame_id |
| `model_path` | string | `""` | 力矩模式 MJCF 路径；为空时自动加载 `resources/zrobot/mjcf/zrobot.xml` |
| `position_model_path` | string | `""` | position 模式 MJCF 路径；为空且 model_path 为空时自动加载 `resources/zrobot/mjcf/zrobot_position.xml` |

---

## API 参考

### 服务

#### /rob_stride_control

**请求**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `positions` | float32[23] | 23 个关节的目标位置（弧度） |

**响应**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `feedback_positions` | float32[23] | 当前关节位置 |
| `feedback_velocities` | float32[23] | 当前关节速度 |
| `feedback_torques` | float32[23] | 当前关节力矩 |
| `feedback_temperatures` | float32[23] | 当前关节温度（固定 35.0°） |
| `success` | bool | 是否成功 |
| `message` | string | 状态信息 |

#### /get_positions

**请求**：空

**响应**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `feedback_positions` | float32[23] | 23 个关节当前弧度位置 |
| `success` | bool | 是否成功 |
| `message` | string | 状态信息 |

#### /set_zeros

**请求**：空

**响应**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | bool | 是否成功 |
| `message` | string | 状态信息 |

### 话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/joint_states` | sensor_msgs/JointState | control_frequency | 关节名称、位置、速度、力矩 |
| `/imu/data` | sensor_msgs/Imu | control_frequency | 四元数方向、角速度、线加速度 |
| `/clock` | rosgraph_msgs/Clock | control_frequency | MuJoCo 内部仿真时间，支持 use_sim_time |

---

## 使用示例

### 查询关节位置

```bash
ros2 service call /get_positions rs_interface/srv/GetPositions
```

### 发送位置指令

```bash
# 向所有 23 个关节发送零位（需补齐 23 个值）
ros2 service call /rob_stride_control rs_interface/srv/RobStrideMsgs \
  "{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### 设置零点

```bash
ros2 service call /set_zeros rs_interface/srv/SetZeros
```

### 查看仿真输出

```bash
# 查看关节状态
ros2 topic echo /joint_states

# 查看 IMU 数据
ros2 topic echo /imu/data

# 查看仿真时间
ros2 topic echo /clock
```

### 与 zrobot_deploy 配合

由于本包提供了与 `zrobot_bridge` 相同的服务接口和命名，`zrobot_deploy` 中的 `FSM` 状态机可直接使用：

```bash
# 终端 1：启动 MuJoCo 仿真
ros2 launch zrobot_mj_sim mujoco_bringup.launch.py

# 终端 2：启动部署状态机（需 zrobot_deploy 包已构建）
ros2 run zrobot_deploy locomotion_node
```

---

## 内部实现要点

- **仿真循环**：由 ROS 2 定时器驱动，每次触发执行一次 `mj_step()` 批次（仿真步数根据控制频率和 MuJoCo timestep 自动计算）。
- **线程安全**：`mjData` 的读写均在 `state_mutex_` 保护下进行，服务回调和控制循环共享同一互斥锁。
- **关节对齐**：通过 `mj_name2id` 将 `joint_names` 参数中的名称映射为 MuJoCo 内部的 `qpos_adr`/`dof_adr`/`actuator_id`，支持任意子集的 23 通道数组语义。
- **零位偏移**：采用与 `gazebo_motor_bridge_node` 相同的 `zero_offsets_` 机制，`/set_zeros` 后将偏移应用到所有位置反馈。
- **服务直接写目标**：`/rob_stride_control` 将目标位置存入 `target_positions_`，由下一个控制定时器周期实际执行，保证控制指令与仿真步进同步。

---

## 验证

启动后执行以下检查确认节点正常工作：

```bash
# 1. 确认节点存在
ros2 node list | grep mujoco

# 2. 确认服务列表
ros2 service list | grep -E "rob_stride_control|get_positions|set_zeros"

# 3. 确认话题列表
ros2 topic list | grep -E "joint_states|imu/data|clock"

# 4. 检查反馈数据
ros2 service call /get_positions rs_interface/srv/GetPositions

# 5. 发送一条位置指令并观察 /joint_states 更新
ros2 service call /rob_stride_control rs_interface/srv/RobStrideMsgs \
  "{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```
