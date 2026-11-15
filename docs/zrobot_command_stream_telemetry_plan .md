# ZRobot 命令流下发与独立 Topic 遥测移植方案

- 状态：代码已实现；实机 candump 帧级回归与硬件联调待执行
- 日期：2026-11-15
- 目标：将 `zrobot_pi_ws` 的底层控制从“服务请求-响应”改为类似 `lz01` 的“命令流下发 + 独立 Topic 状态发布”，并保持电机通信协议字节不变

## 1. 改造边界

本次允许修改 `motor_cfg.cpp`，但只限 CAN 通信收发部分。电机通信协议相关代码不得变动。

| 层 | 允许修改 | 必须保持不变 |
| --- | --- | --- |
| CAN 传输 | socket 选项、`poll/select`、短超时、非阻塞读写、发送/接收拆分、返回值、日志、线程归属 | — |
| 电机协议 | — | CAN ID 位布局、扩展帧标志、`communication_type`、数据字节顺序、`float_to_uint` 映射、`ACTUATOR_OPERATION_MAPPING`、反馈解码、温度系数、寄存器索引（0x7005/0x7016/0x7017 等）、模式切换顺序、`usleep` 时序、零点流程 |

一句话：**帧里的每一个字节都不变，只改变“什么时候发、等多久、怎么收、失败怎么返回”。**

## 2. 目标架构

```text
zrobot_deploy (100 Hz)
   │ publish /joint_cmd  (depth 1, latest wins)
   ▼
zrobot_bridge (100 Hz tick)
   │ 1. 取最新命令
   │ 2. 23 路写命令（非阻塞）
   │ 3. poll 收状态帧（短超时/非阻塞）
   ├── publish /joint_state       (100 Hz, 含 valid[])
   └── publish /motor_telemetry   (10 Hz, 温度/错误码/反馈年龄)
   ▲
zrobot_deploy subscribe /joint_state
```

仿真桥（Gazebo / MuJoCo）实现同一套 `/joint_cmd`、`/joint_state`、`/motor_telemetry` 契约，deploy 不再区分实机和仿真。

核心原则：

1. 控制层只发布意图，不再等待命令响应。
2. 命令回调只更新缓存，所有 CAN 读写都在驱动 tick 内完成。
3. 命令 QoS 深度为 1，只保留最新目标，不补发历史命令。
4. 即使没有新命令，tick 也持续发送保持/阻尼命令，以维持状态反馈。
5. 单电机失联只影响该电机的 `valid/online`，不阻塞全局 tick。

## 3. 接口设计

### 3.1 消息定义

建议在 `rs_interface` 中新增消息，不必新建包。

```text
# rs_interface/msg/JointCmd.msg
string command              # "mit" | "enable" | "disable" | "stop" | "damp" | "set_zero" | "status"
builtin_interfaces/Time stamp
string[] name               # 可空，空表示 0..22 固定顺序
float64[] position
float64[] velocity
float64[] torque
float64[] kp
float64[] kd
```

```text
# rs_interface/msg/JointState.msg
builtin_interfaces/Time stamp
string[] name
float64[] position
float64[] velocity
float64[] torque
float64[] temperature
bool[] valid                # 每个电机反馈是否在有效期内
```

```text
# rs_interface/msg/MotorTelemetry.msg
builtin_interfaces/Time stamp
string[] name
uint8[] error_code
float64[] feedback_age_ms
bool[] online
bool command_stale
string safety_state         # "disabled" | "active" | "damping" | "preparing" | "maintenance"
```

数组长度统一为 23。仿真中未接线的关节补 0，不要因为仿真只有 12 个主动关节就改变数组长度，否则实机和仿真不能共用同一份控制代码。

### 3.2 Topic 与 QoS

| Topic | 方向 | QoS | 说明 |
| --- | --- | --- | --- |
| `/joint_cmd` | deploy → bridge | reliable, depth 1 | 最新命令覆盖，不补发 |
| `/joint_state` | bridge → deploy | best effort, depth 1 | 周期状态，靠 `valid[]` 和反馈年龄兜底 |
| `/motor_telemetry` | bridge → 监控/录包 | best effort, depth 1 | 10 Hz 诊断 |
| `/set_zeros` | 工具 → bridge | service | 标定保留，回调只入队 |

不要使用 `/joint_states`：Gazebo 和 MuJoCo 内部已经在使用这个标准话题。控制契约使用单数 `/joint_state`，避免冲突。

### 3.3 命令语义

| command | 行为 |
| --- | --- |
| `mit` | 位置 + 速度 + 前馈力矩 + Kp + Kd |
| `enable` | 模式准备并使能，先发软保持 |
| `disable` / `stop` | 清空命令缓存，零目标后失能 |
| `damp` | `kp=0`、`kd=阻尼增益`、目标取最后有效反馈位置 |
| `set_zero` | 执行现有零点流程，排入维护队列 |
| `status` | 打印/发布当前状态 |

## 4. `motor_cfg.cpp/h` 传输层改造

### 4.1 API 调整

保持现有帧构造代码在原位置，只增加“是否等待”和“等待多久”。

```cpp
// 现有签名加一个参数；帧构造、模式切换分支完全不动
std::tuple<float, float, float, float> send_motion_command(
    float torque, float position, float velocity, float kp, float kd,
    bool wait_for_reply = true);

// receive_status_frame 改为可超时、返回是否收到；解析代码原样保留
bool receive_status_frame(int timeout_ms = 5);

// 新增：非阻塞/短超时收一帧，供 tick 使用
bool poll_status(int timeout_ms);
```

- 稳态 tick 调用 `send_motion_command(..., false)`，不再在函数内部等回帧。
- tick 之后调用 `poll_status(0)`，或用一次 `poll()` 收本周期回帧。
- `enable_motor()`、`Disenable_Motor()`、`Set_ZeroPos()` 保持阻塞语义，但把内部的 `receive(5)` 改成可配置的短超时；它们只在维护阶段调用。
- `read_initial_position()` 的 10 秒轮询改为有界轮询；解析逻辑不变。

### 4.2 传输层具体改法

- `receive(timeout_sec)`：
  - 不再每次 `setsockopt(SO_RCVTIMEO)`；
  - 用 `poll()` 或 `select()` 等待可读，再 `recv`；
  - 超时返回空，不抛异常；
  - 非扩展帧、短帧改为返回失败，不再 `throw`。
- `receive_status_frame()`：
  - 保留对 `communication_type`、`data[0..7]` 的解析和所有缩放公式；
  - 只在成功收到 `MotorRequest` 时更新 `position_/velocity_/torque_/temperature_`；
  - 超时返回 `false`，保留上一帧有效值。
- 写方向：
  - 增加短 `poll(POLLOUT)` 或 `MSG_DONTWAIT`，避免 CAN 发送队列满时无限阻塞；
  - `write()` 失败返回 `false`，记录发送失败计数。
- 日志：
  - 删除每帧的 `canid` 和 8 字节 `std::cout`；
  - 改为 `RCLCPP_DEBUG_THROTTLE` 或仅错误时限流打印。CAN 层不依赖 ROS 时可用限流 `fprintf(stderr, ...)`。
- `init_socket()`：
  - socket/bind/filter 保持不变；
  - 可以增加 `O_NONBLOCK` 或发送超时，属于传输层。

### 4.3 协议不变量

以下内容不得改动：

- 所有 `frame.can_id` 的组成方式；
- `frame.data[]` 的字节布局和大小端；
- `float_to_uint` / `uint_to_float`；
- `ACTUATOR_OPERATION_MAPPING` 的缩放；
- 寄存器索引和参数写入/读取顺序；
- 模式切换流程（disable → set mode → get mode → enable）及 `usleep` 时序；
- 零点流程（disable → set speed mode → get mode → zero frame → enable）；
- 反馈解析的位掩码、缩放和温度系数；
- socket 过滤器 `(motor_id << 8) | CAN_EFF_FLAG`。

### 4.4 帧级回归测试

改完后必须验证协议未变：

1. 用 `candump -L canX` 录下旧版本执行固定命令序列的帧；
2. 新版本执行同一序列，过滤相同 CAN ID，比较 `data[]` 字节；
3. 覆盖 `send_motion_command`、`enable`、`disable`、`Set_ZeroPos` 和一次模式切换；
4. 字节完全一致才算通过。

## 5. `zrobot_bridge` 节点改造

### 5.1 tick 设计

单线程 `MutuallyExclusive` callback group，所有总线访问都在 tick 线程：

1. 取 `/joint_cmd` 缓存的最新命令（回调只写 `pending_cmd_`）。
2. 判断命令是否超时；超时则构造阻尼命令。
3. 对 23 个电机调用 `send_motion_command(..., false)`，只写不等。
4. 用 `poll_status(0)` 或一次 `poll()` 排空本周期状态帧；没赶上的帧下一周期再收。
5. 更新每个电机快照和 `valid[]`。
6. 发布 `/joint_state`。
7. 每 10 个 tick 发布 `/motor_telemetry`。

这样 tick 的耗时是 23 次写 + 若干次非阻塞读，基本在毫秒级，不会再出现 5 秒阻塞。反馈最多晚一个周期（10 ms），对 100 Hz 控制可以接受。

可选优化：用一次 `poll()` 监听全部 23 个 socket fd，设置 1~2 ms 总超时，再对就绪 fd 调用 `poll_status(0)`，把单周期收包预算控制在毫秒级。

### 5.2 命令与安全

- `command_timeout_ms` 默认 300 ms。
- 超时后进入 `damping`：`kp=0`、`kd=阻尼增益`、`position=最后有效反馈`、`velocity=0`、`torque=0`。
- 不自动 disable；只有显式 `stop`/`disable` 或可选的 `auto_disable_after_ms` 才断使能。
- `stop`/`disable` 时清空命令缓存，先发零目标再 `Disenable_Motor`。
- 启动默认 disabled；收到 `enable` 后做一次软保持，再接受 `mit`。
- 首次命令触发模式准备时，丢弃触发准备的那条命令（参考 lz01 的 `prepared_now` 逻辑），等下一条 100 Hz 命令。
- `set_zero`、`enable`、`disable` 不直接在 ROS 回调里执行，放进维护队列；维护期间 tick 暂停电机写，状态继续发布并标记 `safety_state="maintenance"`。
- 入口校验 23 长度、NaN/Inf、位置/速度/力矩/增益限幅；不能依赖 `float_to_uint` 的静默截断。

### 5.3 状态与遥测

- `/joint_state` 每个 tick 都发布，即使没有新命令；tick 会持续发保持/阻尼命令来触发 RobStride 的状态回帧。
- `valid[i] = (now - last_feedback_time) < feedback_timeout_ms`。
- `temperature` 直接使用现有反馈解析值。
- `error_code` 从现有 CAN ID 解析字段复制到快照。
- `/motor_telemetry` 发布反馈年龄、在线状态、命令是否过期、当前安全状态。
- 说明：RobStride 的反馈是命令回帧，**ROS 层独立、CAN 层仍由 tick 命令触发**；真正的独立轮询采样需要改协议，不在本次范围。

### 5.4 启动与维护

- 构造阶段只创建 `RobStrideMotor` 和 socket，不自动使能。
- 收到 `enable` 后进入 `preparing`：检查/切换模式、软保持、逐个使能，全部完成后置为 `active`。
- `set_zero` 期间暂停对应电机或全部电机的 tick 写命令；标定完成后重新做软保持。
- `disable` 完成后清空所有命令缓存；后续 `mit` 在重新使能前不生效。
- 建议增加 `live` 参数（对标 lz01 的 dry-run）：`live=false` 时不打开 CAN，发布 fallback 状态，方便在笔记本上联调 deploy。

## 6. `zrobot_deploy` 改造

- `FSM` 基类删除 `control_client_` / `get_positions_client_`，改为 `/joint_cmd` publisher 和 `/joint_state` subscriber。
- 保留 `sendMotorPositions()` / `getMotorFeedback()` 方法名，内部改为“发布命令 / 读缓存”，减少各状态机改动。
- `initialize()` 改为：发送 `enable`，等待第一帧 `/joint_state` 且 12 个主动关节 `valid=true`；不再等待服务。
- `FixStand` 用第一帧状态作为插值起点，删除 `/get_positions` 启动阻塞。
- `Damping` 可以直接发 `damp` 命令，由驱动使用自己的反馈做阻尼；如果保留软件阻尼，则读 `/joint_state` 缓存。
- `Locomotion` / `PTLocomotion`：状态年龄超限或 `valid=false` 时停止推理并发布 `damp`；增加 `cmd_vel_timeout_ms`，超时清零速度指令。
- `main.cpp`：用独立 executor 线程或 `spin_some`，去掉 `spin_until_future_complete`；退出时发布 `damp`，必要时再发布 `disable`。
- 增益迁移分两步：
  - 第一阶段 `JointCmd.kp/kd` 留空，桥接仍用 `motor_config.yaml` 的 `motor_kps_/kds`，行为不变；
  - 第二阶段把增益移到 deploy，Damping 使用 `kp=0, kd=阻尼`，起立使用准备增益，与 lz01 一致。

## 7. 仿真桥改造

- `zrobot_gz_sim`：删除三个服务；订阅 `/joint_cmd` 缓存目标，现有 200 Hz 定时器继续计算 PD 力矩；从 Gazebo `/joint_states` 更新内部状态，并发布 `/joint_state` 契约。
- `zrobot_mj_sim`：删除服务；订阅 `/joint_cmd` 缓存目标，`control_loop()` 继续推进仿真，发布 `/joint_state`；`set_zero` 映射为现有零位偏移。
- 仿真未接线的关节在数组中补 0，保持 23 长度；`valid=true`（仿真状态是权威的）。
- 所有桥统一 `state_publish_hz`，建议默认 100 Hz；仿真时间戳用节点时钟，开启 `use_sim_time` 时 deploy 的新鲜度检查也用 ROS 时间。

## 8. 文件级任务清单

| 文件 | 改动 |
| --- | --- |
| `src/rs_interface/CMakeLists.txt`、`package.xml`、`msg/*.msg` | 新增消息；`SetZeros` 保留，`RobStrideMsgs`/`GetPositions` 过渡期后删除 |
| `src/zrobot_bridge/include/zrobot_bridge/motor_cfg.h` | 新增 `poll_status`；`receive_status_frame` 返回 bool；`send_motion_command` 增加 `wait_for_reply` |
| `src/zrobot_bridge/src/motor_cfg.cpp` | 只改传输：poll/短超时/非阻塞写/日志/错误返回；帧构造和解析不动 |
| `src/zrobot_bridge/include/zrobot_bridge/motor_controller.h` | 删除 service 成员；新增 pub/sub、命令缓存、维护队列、快照、tick 状态 |
| `src/zrobot_bridge/src/motor_controller_node.cpp` | 删除 3 个服务回调；实现 tick、命令处理、状态/遥测、超时阻尼、启动使能 |
| `src/zrobot_bridge/config/motor_config.yaml` | 新增 joint_names、topic 名、控制频率、超时、阻尼增益、`live` |
| `src/zrobot_deploy/include/zrobot_deploy/FSM.h`、`src/FSM.cpp` | client → pub/sub；缓存反馈；首帧等待；状态新鲜度 |
| `src/zrobot_deploy/src/FixStand.cpp`、`Damping.cpp` | 删除 `/get_positions`；使用状态缓存或 `damp` 命令 |
| `src/zrobot_deploy/src/Locomotion.cpp`、`PTLocomotion.cpp` | 状态门控、`cmd_vel` 超时 |
| `src/zrobot_deploy/src/main.cpp` | executor 线程；退出 `damp` |
| `src/zrobot_gz_sim/*`、`src/zrobot_mj_sim/*` | 同一套 topic 契约 |
| `README.md`、各包 README | 更新接口和启动方式 |

## 9. 实施阶段

1. **传输层硬化 + 帧回归**：改 `motor_cfg.cpp` 的收发，跑 `candump` 字节对比；此时 ROS 接口仍是服务，先证明协议没变。
2. **bridge 双接口**：新增 topic 接口，保留旧服务一版；用 `ros2 topic pub` 验证命令行、状态频率、超时阻尼。
3. **deploy 切 topic**：先在 MuJoCo 跑 FixStand → Locomotion → Damping，再跑 Gazebo。
4. **仿真桥切 topic**：统一契约，删除服务。
5. **删除旧服务**：保留 `/set_zeros` 或改成 `set_zero` 命令；更新文档。

## 10. 验收标准

- `/joint_cmd` 稳定 100 Hz，`/joint_state` 稳定在配置频率，反馈年龄正常时 < 20~50 ms。
- 拔掉任意电机：其他 22 个电机继续按 tick 运行；该电机 `valid=false`、`online=false`，不出现秒级停顿。
- `kill` deploy 后，驱动在 `command_timeout_ms` 内进入阻尼，telemetry 标记 `command_stale=true`。
- 帧级回归通过：新旧版本同一命令序列的 CAN ID 和 `data[]` 完全一致。
- 同一份 deploy 可在实机、Gazebo、MuJoCo 间切换，只改参数/launch。

## 11. 风险与注意事项

- **不要顺手改协议**：模式切换里的 `usleep`、寄存器索引、零点流程即使看起来可以优化，也不要动；这些属于电机状态机时序。
- `drw.run_mode.data` 目前没有显式初始化，`send_motion_command` 的模式切换条件存在不确定性；这属于既有问题，本次不要静默修改，先加日志和 telemetry 观察。
- `Set_ZeroPos()` 内部会再次 `enable_motor()`，必须放在维护阶段，不能在 tick 里并发调用。
- `read_initial_position()` 里硬编码了 `mid == 0x01`，新流程不要依赖它，用 `/joint_state` 的首帧代替。
- 反馈帧可能晚于本周期到达，tick 用“上一周期排空 + 本周期非阻塞收”即可；不要把“没收到”当成错误抛出。
- 命令 QoS 必须 depth 1；否则 deploy 卡顿后会把过期目标补发出去。
- `JointState` 必须带 `valid[]`，不能用旧快照冒充新状态，否则 deploy 会拿失效反馈继续推理。
- `motor_config.yaml` 注释中的关节顺序与 `Locomotion::default_pose_` 的顺序不一致；迁移前先定义唯一权威的 0..22 顺序，并通过 `JointState.name` 显式带出，不要在迁移中重排电机。
- 实机 23 个电机、仿真 12 个主动关节：数组长度统一为 23，仿真未接线关节补 0。
- 若保留 `SetZeros` 服务，其回调必须只入队，不能直接调用电机 API 与 tick 竞争。
- 删除每帧 `std::cout` 是 100 Hz 化的前提；否则控制台 I/O 会吞掉大部分周期。