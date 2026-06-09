# rs_interface

## 概述

ROS 2 服务接口包，定义 zrobot 机器人电机控制所需的三个服务：发送 23 个电机位置指令并获取反馈、读取当前电机位置、设置零位偏移。所有服务基于 CAN 总线的 RobStride 电机协议，电机 CAN ID 到数组索引的映射由服务端节点参数配置。

## 依赖

### ROS 2
- `rosidl_default_generators`

## 构建

```bash
colcon build --packages-select rs_interface
```

## 使用

下游包在 `package.xml` 中声明 `<depend>rs_interface</depend>`，在 `CMakeLists.txt` 中 `find_package(rs_interface REQUIRED)` 后即可使用。

## 服务定义

### RobStrideMsgs.srv

发送 23 个电机位置指令并获取完整反馈。

```
float32[23] positions
---
float32[23] feedback_positions
float32[23] feedback_velocities
float32[23] feedback_torques
float32[23] feedback_temperatures
bool success
string message
```

### GetPositions.srv

读取当前所有电机位置。

```
---
float32[23] feedback_positions
bool success
string message
```

### SetZeros.srv

将当前电机位置设置为零位（此后上报的位置均减去该偏移量）。

```
---
bool success
string message
```

## 项目结构

```
rs_interface/
├── CMakeLists.txt
├── package.xml
└── srv/
    ├── RobStrideMsgs.srv
    ├── GetPositions.srv
    └── SetZeros.srv
```
