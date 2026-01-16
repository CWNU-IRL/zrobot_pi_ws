# ZRobot Deploy 快速使用指南

## 快速开始

### 1. 编译软件包
```bash
cd /root/codes/zrobot_pi_ws
colcon build --packages-select zrobot_deploy
```

### 2. 运行程序

**方法1 - 使用启动脚本（推荐）：**
```bash
cd /root/codes/zrobot_pi_ws/src/zrobot_deploy
./run.sh
```

**方法2 - 手动启动：**
```bash
source /root/codes/zrobot_pi_ws/install/setup.bash
ros2 run zrobot_deploy main
```

### 3. 使用键盘控制

程序运行后，使用以下按键控制机器人：

| 按键 | 功能 |
|------|------|
| `F` 或 `f` | 启动 FixStand 状态机 |
| `S` 或 `s` | 停止当前状态机 |
| `Q` 或 `q` | 退出程序 |

## FixStand 状态机说明

**功能：** 将机器人缓慢移动到机械零位并保持

**运行过程：**
1. 按下 `F` 键启动
2. 读取当前电机位置
3. 在3秒内线性插值移动到零位（所有电机位置=0）
4. 到达零位后保持站立姿态
5. 按 `S` 键可停止状态机

**安全提示：**
- ⚠️ 运行前确保机器人周围安全
- ⚠️ 确认零位姿态对机器人是安全的
- ⚠️ 准备好紧急停止措施

## 系统要求

- ROS2 环境
- rs_interface 软件包（提供 RobStrideMsgs 服务）
- `/rob_stride_control` 服务正在运行

## 检查服务状态

```bash
# 查看服务列表
ros2 service list

# 查看服务类型
ros2 service type /rob_stride_control

# 测试服务调用
ros2 service call /rob_stride_control rs_interface/srv/RobStrideMsgs "{positions: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}"
```

## 常见问题

### Q: 键盘输入无响应？
A: 确保终端窗口处于活动状态（点击终端窗口）

### Q: 服务未找到错误？
A: 
1. 检查 `/rob_stride_control` 服务是否运行：
   ```bash
   ros2 service list | grep rob_stride_control
   ```
2. 启动底层控制服务

### Q: 如何修改移动速度？
A: 编辑 [FixStand.cpp](src/FixStand.cpp#L15)，修改 `interpolation_time_` 参数（单位：秒）

### Q: 如何修改控制频率？
A: 编辑 [main.cpp](src/main.cpp#L81)，修改 `rclcpp::Rate rate(100)` 中的数值（单位：Hz）

## 日志信息

程序运行时会输出以下日志：

- `ZRobot FSM Controller started` - 程序启动
- `Waiting for /rob_stride_control service...` - 等待服务
- `FixStand initializing...` - 初始化状态机
- `Moving to zero: XX% complete` - 移动进度
- `Reached zero position, now standing` - 到达目标位置
- `Standing at zero position` - 保持站立（每5秒输出一次）

## 扩展开发

要添加新的状态机（如行走、小跑等），请参考 [README.md](README.md#添加新的状态机) 中的说明。

## 架构图

```
main.cpp
  └─> 创建 ROS2 节点
  └─> 键盘输入循环 (100Hz)
      ├─> 检测按键
      ├─> 创建/切换状态机
      └─> 调用当前状态机的 run()
  
FSM 基类
  ├─> initialize()   - 初始化服务客户端
  ├─> run()          - 虚函数，由子类实现
  ├─> exit()         - 退出清理
  └─> sendMotorPositions() - 发送电机控制指令

FixStand 状态机 (继承 FSM)
  ├─> INIT     - 初始化
  ├─> MOVING   - 插值移动到零位
  └─> STANDING - 保持零位
```

## 相关文件

- [FSM.h](include/zrobot_deploy/FSM.h) - FSM基类头文件
- [FSM.cpp](src/FSM.cpp) - FSM基类实现
- [FixStand.h](include/zrobot_deploy/FixStand.h) - FixStand状态机头文件
- [FixStand.cpp](src/FixStand.cpp) - FixStand状态机实现
- [main.cpp](src/main.cpp) - 主程序
- [CMakeLists.txt](CMakeLists.txt) - 编译配置
- [package.xml](package.xml) - ROS2包配置

## 技术支持

如有问题，请查看：
1. 日志输出
2. ROS2 服务状态
3. 电机反馈数据

---
Created by Bill on 2026-01-16
