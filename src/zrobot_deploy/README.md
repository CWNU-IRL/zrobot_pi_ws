# ZRobot Deploy - 机器人运动控制FSM系统

## 项目简介

这个软件包使用有限状态机（FSM）实现机器人的运动控制算法。底层通过向服务 `/rob_stride_control` 发送 `rs_interface/srv/RobStrideMsgs` 消息中的电机位置进行控制。

## 功能特性

- **FSM基类**: 提供了状态机的基础框架，包含ROS2服务调用、电机控制等通用功能
- **FixStand状态机**: 实现机器人缓慢移动到机械零位并保持的功能
- **键盘控制**: 通过键盘按键切换不同的状态机
- **可扩展**: 易于添加新的状态机类型（如行走、小跑等）

## 架构设计

### FSM基类 (FSM.h/FSM.cpp)

- 提供与ROS2服务的接口
- 管理电机位置发送和反馈接收
- 定义状态机的通用接口

### FixStand状态机 (FixStand.h/FixStand.cpp)

实现三个内部状态：
1. **INIT**: 初始化阶段
2. **MOVING**: 从当前位置缓慢移动到机械零位（3秒）
3. **STANDING**: 保持在零位

### 主程序 (main.cpp)

- 创建ROS2节点
- 处理键盘输入
- 管理状态机的启动和停止
- 以100Hz频率运行控制循环

## 编译

在ROS2工作空间中编译：

```bash
cd /root/codes/zrobot_pi_ws
colcon build --packages-select zrobot_deploy
```

## 运行

1. 首先source工作空间：

```bash
source /root/codes/zrobot_pi_ws/install/setup.bash
```

2. 确保 `/rob_stride_control` 服务正在运行

3. 运行控制器：

```bash
ros2 run zrobot_deploy main
```

## 键盘控制

程序运行后，可以使用以下按键：

- **F**: 启动 FixStand 状态机（将机器人缓慢移动到机械零位）
- **S**: 停止当前正在运行的状态机
- **Q**: 退出程序

## 电机控制说明

- 系统通过 `/rob_stride_control` 服务发送23个电机的位置指令
- 控制频率: 100Hz
- FixStand状态机会在3秒内线性插值从当前位置移动到零位
- 机械零位定义为所有电机位置为0弧度

## 添加新的状态机

要添加新的状态机类型，需要：

1. 创建新的头文件继承自 `FSM` 类
2. 实现 `initialize()`, `run()`, `exit()` 方法
3. 在 `main.cpp` 中添加对应的键盘控制逻辑
4. 在 `CMakeLists.txt` 中添加新的源文件

示例：

```cpp
// MyNewFSM.h
#include "zrobot_deploy/FSM.h"

class MyNewFSM : public FSM
{
public:
    MyNewFSM(std::shared_ptr<rclcpp::Node> node);
    void initialize() override;
    void run() override;
    void exit() override;
};
```

## 安全注意事项

⚠️ **重要**: 
- 运行前确保机器人处于安全状态
- FixStand会将机器人移动到零位，确保这个姿态对您的机器人是安全的
- 始终准备好紧急停止按钮
- 测试时建议先使用较慢的移动速度

## 故障排除

1. **服务未找到**: 确保 `/rob_stride_control` 服务正在运行
   ```bash
   ros2 service list | grep rob_stride_control
   ```

2. **编译错误**: 确保已安装所有依赖
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **键盘无响应**: 确保终端处于前台且具有输入焦点

## 技术参数

- ROS2版本: 需要支持服务调用的版本
- 控制频率: 100Hz
- 电机数量: 23个
- 插值时间: 3秒（可在FixStand构造函数中修改）

## 许可证

TODO: 添加许可证信息

## 作者

Created by Bill on 2026-01-16
