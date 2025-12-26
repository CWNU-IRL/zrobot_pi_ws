# RobStride电机控制节点 - 快速开始指南

## 1. 前置条件

- ROS2环境已安装（Foxy、Galactic或Humble）
- rs_interface包已编译
- CAN接口已配置

## 2. 编译包

```bash
# 进入工作空间
cd ~/ros2_ws  # 或你的工作空间路径

# 编译包
colcon build --packages-select rs_interface zrobot_bridge

# 编译成功后，应该看到类似输出：
# [100%] Linking CXX executable /home/user/ros2_ws/install/zrobot_bridge/lib/zrobot_bridge/motor_controller_node
# [100%] Built target motor_controller_node
```

## 3. 配置电机参数

编辑 `config/motor_config.yaml` 文件，配置你的电机CAN ID和类型：

```yaml
motor_controller_node:
  ros__parameters:
    can_interface: "can0"     # 确保CAN接口名称正确
    master_id: 0
    motor_can_ids: [1, 2, 3, ..., 23]   # 设置实际的电机CAN ID
    motor_types: [0, 0, 0, ..., 0]      # 设置每个电机的型号
```

## 4. 启动CAN接口

```bash
# 查看可用的CAN接口
ip link show

# 启用CAN接口（需要sudo权限）
sudo ip link set can0 up type can bitrate 1000000

# 验证CAN接口已启用
ip link show can0
# 输出应该显示: ... state UP ...
```

## 5. 运行节点

### 方式A: 使用launch文件（推荐）

```bash
source install/setup.bash
ros2 launch zrobot_bridge motor_controller.launch.py
```

### 方式B: 直接运行节点

```bash
source install/setup.bash
ros2 run zrobot_bridge motor_controller_node
```

## 6. 测试服务

### 使用ROS2 CLI测试

```bash
# 在另一个终端，source环境后运行以下命令调用服务

# 示例1: 所有电机回到零位
ros2 service call /motor_controller_node/rob_stride_control \
  rs_interface/RobStrideMsgs \
  "positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

# 示例2: 电机0和1分别转到90度和-90度
ros2 service call /motor_controller_node/rob_stride_control \
  rs_interface/RobStrideMsgs \
  "positions: [1.5708, -1.5708, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

### 使用C++示例程序

```bash
source install/setup.bash

# 运行示例1: 所有电机回到零位
ros2 run zrobot_bridge motor_client_example 1

# 运行示例2: 设置不同电机的不同位置
ros2 run zrobot_bridge motor_client_example 2

# 运行示例3: 所有电机做正弦波运动
ros2 run zrobot_bridge motor_client_example 3
```

### 使用Python示例程序

```bash
source install/setup.bash

# 先给Python脚本添加执行权限
chmod +x install/share/zrobot_bridge/examples/motor_client_example.py

# 运行示例1: 所有电机回到零位
python3 install/share/zrobot_bridge/examples/motor_client_example.py 1

# 运行示例2: 设置不同电机的不同位置
python3 install/share/zrobot_bridge/examples/motor_client_example.py 2

# 运行示例3: 所有电机做正弦波运动
python3 install/share/zrobot_bridge/examples/motor_client_example.py 3
```

## 7. 验证节点运行

```bash
# 在另一个终端查看已运行的节点
ros2 node list

# 输出应该包含:
# /motor_controller_node

# 查看节点信息
ros2 node info /motor_controller_node

# 查看提供的服务
ros2 service list | grep rob_stride
# 输出应该包含:
# /motor_controller_node/rob_stride_control
```

## 8. 监听日志

```bash
# 查看节点的运行日志
ros2 run rclcpp_components list_components zrobot_bridge

# 或者在启动节点时，日志会直接输出到终端
```

## 常见问题排查

### 问题1: "CAN socket: Permission denied"

**解决方案:**
```bash
# 方法1: 使用sudo运行节点
sudo -s
source /path/to/install/setup.bash
ros2 run zrobot_bridge motor_controller_node

# 方法2: 将用户添加到can用户组
sudo usermod -a -G can $USER
# 然后重新登录
```

### 问题2: "ioctl: No such device"

**原因**: CAN接口未启用  
**解决方案:**
```bash
# 查看CAN接口
ip link show

# 启用CAN接口
sudo ip link set can0 up type can bitrate 1000000

# 验证
ip link show can0
```

### 问题3: 服务调用超时

**原因**: 可能是电机未响应或CAN总线故障  
**解决方案:**
- 检查CAN总线连接
- 确认电机已供电
- 确认电机CAN ID配置正确
- 查看节点日志了解详细错误信息

### 问题4: 编译失败

**原因**: 缺少依赖或编译选项问题  
**解决方案:**
```bash
# 清理编译输出
rm -rf build/ install/ log/

# 重新编译
colcon build --packages-select rs_interface zrobot_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 性能优化建议

1. **编译优化**: 使用Release模式编译
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

2. **CAN带宽**: 根据需要调整CAN波特率
   ```bash
   sudo ip link set can0 up type can bitrate 1000000  # 1Mbps
   ```

3. **日志级别**: 在生产环境中降低日志输出级别
   ```bash
   ROS_LOG_LEVEL=warn ros2 run zrobot_bridge motor_controller_node
   ```

## 下一步

- 查看 [README_CN.md](README_CN.md) 获取详细的API文档
- 查看 [examples/](examples/) 目录获取更多示例代码
- 根据需要修改 [config/motor_config.yaml](config/motor_config.yaml) 中的参数

## 获取帮助

- 检查节点日志了解详细的错误信息
- 使用 `ros2 service call` 命令行工具测试服务
- 参考 [README_CN.md](README_CN.md) 中的故障排除部分
