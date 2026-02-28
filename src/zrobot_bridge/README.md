# RobStride电机控制节点

## 概述

这是一个ROS2节点，用于控制多达23个RobStride电机。该节点通过CAN总线与电机通信，并提供了一个ROS2服务接口来批量控制所有电机。

## 功能特性

- ✅ 同时控制23个电机
- ✅ 支持不同CAN ID的电机配置
- ✅ 支持多种电机型号（ROBSTRIDE_00 至 ROBSTRIDE_06）
- ✅ 通过ROS2服务提供位置控制
- ✅ 实时获取电机反馈（位置、速度、扭矩、温度）
- ✅ 线程安全的电机访问
- ✅ 完整的错误处理和日志记录

## 节点信息

### 节点名称
`motor_controller_node`

### 提供的服务
`/motor_controller_node/rob_stride_control` (type: `rs_interface/RobStrideMsgs`)

### 服务接口

**请求 (Request):**
```
float32[23] positions  # 23个电机的目标位置 (单位: rad)
```

**响应 (Response):**
```
float32[23] feedback_positions    # 电机反馈位置
float32[23] feedback_velocities   # 电机反馈速度
float32[23] feedback_torques      # 电机反馈扭矩
float32[23] feedback_temperatures # 电机反馈温度
bool success                      # 执行是否成功
string message                    # 执行状态消息
```

## 配置文件

配置文件位置: `config/motor_config.yaml`

### 主要参数

| 参数名 | 类型 | 描述 | 默认值 |
|--------|------|------|--------|
| can_interface | string | CAN网络接口名称 | "can0" |
| master_id | int | 主机ID | 0 |
| motor_can_ids | array[23] | 各电机的CAN ID | [1,2,...,23] |
| motor_types | array[23] | 各电机的型号类型 | [0,0,...,0] |

### 电机类型定义

| 类型值 | 电机型号 | 范围 | 最大速度 | 最大扭矩 |
|--------|---------|------|---------|---------|
| 0 | ROBSTRIDE_00 | 4π rad | 50 rad/s | 17 Nm |
| 1 | ROBSTRIDE_01 | 4π rad | 44 rad/s | 17 Nm |
| 2 | ROBSTRIDE_02 | 4π rad | 44 rad/s | 17 Nm |
| 3 | ROBSTRIDE_03 | 4π rad | 50 rad/s | 60 Nm |
| 4 | ROBSTRIDE_04 | 4π rad | 15 rad/s | 120 Nm |
| 5 | ROBSTRIDE_05 | 4π rad | 33 rad/s | 17 Nm |
| 6 | ROBSTRIDE_06 | 4π rad | 20 rad/s | 60 Nm |

## 编译

```bash
# 在工作空间根目录执行
colcon build --packages-select zrobot_bridge

# 编译特定包和依赖
colcon build --packages-select rs_interface zrobot_bridge
```

## 运行

### 方式1: 使用launch文件

```bash
# 首先source环境
source install/setup.bash

# 运行launch文件
ros2 launch zrobot_bridge motor_controller.launch.py
```

### 方式2: 直接运行节点

```bash
# 首先source环境
source install/setup.bash

# 运行节点
ros2 run zrobot_bridge motor_controller_node
```

## 使用示例

### Python客户端示例

```python
import rclpy
from rs_interface.srv import RobStrideMsgs
import math

def main():
    rclpy.init()
    node = rclpy.create_node('motor_client')
    
    # 创建服务客户端
    client = node.create_client(RobStrideMsgs, '/motor_controller_node/rob_stride_control')
    
    # 等待服务就绪
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('等待服务...')
    
    # 创建请求
    request = RobStrideMsgs.Request()
    
    # 设置所有电机的目标位置（这里设置为0）
    request.positions = [0.0] * 23
    
    # 或者设置不同的位置
    # request.positions[0] = math.pi / 2  # 电机0设置为90度
    # request.positions[1] = -math.pi / 2 # 电机1设置为-90度
    
    # 调用服务
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    # 处理响应
    response = future.result()
    
    if response.success:
        print("服务调用成功!")
        for i in range(23):
            print(f"电机{i}: 位置={response.feedback_positions[i]:.3f}, "
                  f"速度={response.feedback_velocities[i]:.3f}, "
                  f"扭矩={response.feedback_torques[i]:.3f}, "
                  f"温度={response.feedback_temperatures[i]:.1f}")
    else:
        print(f"服务调用失败: {response.message}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### ROS2 CLI 调用

```bash
# 调用服务，所有电机位置设为0
ros2 service call /motor_controller_node/rob_stride_control \
  rs_interface/RobStrideMsgs \
  "positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

## 工作原理

1. **初始化阶段**: 节点启动时，从参数服务器加载电机配置（CAN ID、电机类型）
2. **电机创建**: 为每个电机创建一个`RobStrideMotor`对象，初始化CAN通信socket
3. **服务监听**: 节点创建ROS2服务，监听来自客户端的控制请求
4. **控制流程**: 当收到服务请求时：
   - 验证请求数据（确保有23个位置信息）
   - 向每个电机发送运动控制指令（`send_motion_command`）
   - 接收电机反馈（位置、速度、扭矩、温度）
   - 将反馈数据组织成响应返回给客户端

## 重要注意事项

### CAN配置

运行此节点前，需要确保CAN接口已正确配置：

```bash
# 查看可用的CAN接口
ip link

# 启用CAN接口（如果还没启用）
sudo ip link set can0 up type can bitrate 1000000

# 禁用CAN接口（不再使用时）
sudo ip link set can0 down
```

### 电机连接

- 所有电机必须连接到同一条CAN总线
- 每个电机必须有唯一的CAN ID
- 电机CAN ID必须在0-255范围内

### 参数配置

编辑`config/motor_config.yaml`文件：
- `motor_can_ids`: 设置为你实际使用的电机CAN ID
- `motor_types`: 设置为对应电机的实际型号
- `can_interface`: 确保与系统中的CAN接口名称一致

## 故障排除

### 1. "socket: Permission denied"
**原因**: 没有足够权限访问CAN接口  
**解决**: 使用sudo运行，或将用户添加到can用户组

```bash
sudo usermod -a -G can $USER
```

### 2. "ioctl: No such device"
**原因**: CAN接口不存在或未启用  
**解决**: 检查并启用CAN接口

```bash
ip link show can0
sudo ip link set can0 up type can bitrate 1000000
```

### 3. 服务请求失败
**原因**: 电机通信故障或未配置  
**解决**:
- 检查CAN总线连接
- 确认电机CAN ID配置正确
- 查看节点日志了解详细错误信息

```bash
ros2 node list  # 确认节点已运行
ros2 node info /motor_controller_node  # 查看节点信息
```

## 代码结构

```
zrobot_bridge/
├── include/
│   └── zrobot_bridge/
│       ├── motor_cfg.h              # 电机驱动类定义
│       └── motor_controller.h       # 控制节点类定义
├── src/
│   ├── main.cpp                     # 空的主程序（已被motor_controller_node.cpp替代）
│   ├── motor_cfg.cpp                # 电机驱动实现
│   └── motor_controller_node.cpp    # 控制节点实现
├── launch/
│   └── motor_controller.launch.py   # Launch配置文件
├── config/
│   └── motor_config.yaml            # 参数配置文件
├── CMakeLists.txt                   # CMake构建配置
└── package.xml                      # ROS2包元信息
```

## 扩展和自定义

### 修改控制参数

在`motor_controller_node.cpp`中的`handle_rob_stride_service`函数中，可以修改发送给电机的参数：

```cpp
auto [pos, vel, torq, temp] = motors_[i]->send_motion_command(
    0.0f,                  // torque - 扭矩命令
    target_position,       // position - 目标位置
    5.0f,                  // velocity - 目标速度
    0.5f,                  // kp - 比例增益
    0.1f                   // kd - 微分增益
);
```

### 支持其他控制模式

`RobStrideMotor`类还提供了其他控制模式：
- `send_velocity_mode_command()` - 速度控制
- `RobStrite_Motor_PosPP_control()` - 位置控制（PP模式）
- `RobStrite_Motor_PosCSP_control()` - 位置控制（CSP模式）
- `RobStrite_Motor_Current_control()` - 电流控制

可根据需要在服务回调中调用不同的控制函数。

## 许可证

Apache-2.0

## 作者

根据RobStride电机通信协议和motor_cfg库开发
