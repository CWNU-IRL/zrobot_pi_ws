# ZRobot Deploy - 变更日志

## [1.0.0] - 2026-01-16

### 新增功能
- ✨ 实现了FSM（有限状态机）基础框架
- ✨ 实现了FixStand状态机（缓慢移动到机械零位并保持）
- ✨ 添加了键盘控制界面（F/S/Q按键）
- ✨ 集成了ROS2服务调用功能
- ✨ 实现了100Hz控制循环
- ✨ 添加了线性插值运动规划
- ✨ 实现了电机位置反馈获取

### 文件结构
```
新增文件：
├── include/zrobot_deploy/
│   ├── FSM.h          - FSM基类定义
│   └── FixStand.h     - FixStand状态机定义
├── src/
│   ├── FSM.cpp        - FSM基类实现
│   ├── FixStand.cpp   - FixStand状态机实现  
│   └── main.cpp       - 主程序
├── README.md          - 详细文档
├── QUICK_START.md     - 快速使用指南
├── IMPLEMENTATION.md  - 实现技术细节
├── CHANGELOG.md       - 本文件
└── run.sh             - 启动脚本

更新文件：
├── CMakeLists.txt     - 添加依赖和编译配置
└── package.xml        - 添加ROS2依赖
```

### 技术细节

**FSM基类：**
- 封装了ROS2节点和服务客户端
- 提供电机位置发送接口 `sendMotorPositions()`
- 提供电机反馈获取接口 `getMotorFeedback()`
- 定义了状态机生命周期方法：`initialize()`, `run()`, `exit()`

**FixStand状态机：**
- 三个内部状态：INIT, MOVING, STANDING
- 3秒线性插值从当前位置移动到零位
- 到达零位后持续保持姿态
- 实时日志输出运动进度

**主程序：**
- 键盘输入处理（非阻塞模式）
- 100Hz控制循环
- 状态机生命周期管理
- 用户友好的菜单界面

### 依赖项
- rclcpp (ROS2 C++客户端库)
- rs_interface (提供RobStrideMsgs服务定义)
- std_msgs (标准消息类型)

### 编译测试
- ✅ 编译通过（无警告和错误）
- ✅ 生成可执行文件：install/zrobot_deploy/lib/zrobot_deploy/main

### 已知问题
- 无

### 待办事项
- [ ] 添加更多状态机（Walk, Trot, Jump等）
- [ ] 实现配置文件支持
- [ ] 添加状态机平滑过渡
- [ ] 增加遥控器支持
- [ ] 添加可视化界面
- [ ] 实现紧急停止功能
- [ ] 添加单元测试
- [ ] 添加集成测试

### 使用说明

**编译：**
```bash
cd /root/codes/zrobot_pi_ws
colcon build --packages-select zrobot_deploy
```

**运行：**
```bash
# 方法1 - 使用脚本
./src/zrobot_deploy/run.sh

# 方法2 - 手动启动
source install/setup.bash
ros2 run zrobot_deploy main
```

**控制：**
- 按 `F` 启动FixStand状态机
- 按 `S` 停止当前状态机
- 按 `Q` 退出程序

### 安全提示
⚠️ **重要安全提醒**
1. 运行前确保机器人周围安全
2. 确认机械零位姿态对机器人是安全的
3. 准备好紧急停止措施
4. 首次使用建议在仿真环境测试

### 性能指标
- 控制频率：100 Hz
- 移动时间：3秒（可配置）
- 服务调用超时：100ms
- 键盘响应延迟：< 10ms

### 贡献者
- Bill (@billw) - 初始实现

### 许可证
TODO: 添加许可证信息

---

## 版本计划

### [1.1.0] - 计划中
- 添加Walk步态状态机
- 实现YAML配置文件支持
- 添加状态机过渡动画

### [1.2.0] - 计划中  
- 添加Trot步态状态机
- 实现手柄控制支持
- 添加RViz可视化

### [2.0.0] - 未来
- 重构为插件式架构
- 支持自定义状态机加载
- 添加Web控制界面

---
**文档版本**: 1.0.0  
**最后更新**: 2026-01-16  
**维护者**: Bill
