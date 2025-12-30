# 多CAN接口配置指南

## 概述

代码已更新以支持将23个电机分配到4个CAN接口（can0, can1, can2, can3）。每个电机可以独立配置使用哪个CAN接口。

## 主要修改

### 1. 新增参数：motor_can_interfaces

在 `motor_config.yaml` 中新增了 `motor_can_interfaces` 参数数组，用于指定每个电机使用的CAN接口。

### 2. 代码架构变更

- **motor_controller.h**: 
  - 移除了单一的 `can_interface_` 成员变量
  - 添加了 `motor_can_interfaces_` 数组，存储每个电机的CAN接口

- **motor_controller_node.cpp**:
  - 更新参数声明和加载逻辑
  - 修改电机初始化，使用各自的CAN接口

## 配置文件示例

### 默认配置（前6个can0，接下来6个can1，再6个can2，最后5个can3）

```yaml
motor_controller_node:
  ros__parameters:
    master_id: 0
    
    motor_can_ids: [1, 2, 3, ..., 23]
    motor_types: [0, 0, 0, ..., 0]
    
    motor_can_interfaces:
      - "can0"    # 电机0-5使用can0
      - "can0"
      - "can0"
      - "can0"
      - "can0"
      - "can0"
      - "can1"    # 电机6-11使用can1
      - "can1"
      - "can1"
      - "can1"
      - "can1"
      - "can1"
      - "can2"    # 电机12-17使用can2
      - "can2"
      - "can2"
      - "can2"
      - "can2"
      - "can2"
      - "can3"    # 电机18-22使用can3
      - "can3"
      - "can3"
      - "can3"
      - "can3"
```

### 分组说明

| 电机索引 | CAN接口 | 数量 |
|---------|--------|------|
| 0-5     | can0   | 6个  |
| 6-11    | can1   | 6个  |
| 12-17   | can2   | 6个  |
| 18-22   | can3   | 5个  |

## 硬件准备

### 1. 启用所有CAN接口

在启动节点前，需要启用所有使用的CAN接口：

```bash
# 启用can0
sudo ip link set can0 up type can bitrate 1000000

# 启用can1
sudo ip link set can1 up type can bitrate 1000000

# 启用can2
sudo ip link set can2 up type can bitrate 1000000

# 启用can3
sudo ip link set can3 up type can bitrate 1000000
```

### 2. 验证CAN接口状态

```bash
# 查看所有CAN接口
ip link show | grep can

# 应该看到类似输出：
# 3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP
# 4: can1: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP
# 5: can2: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP
# 6: can3: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP
```

### 3. 检查CAN接口统计

```bash
# 查看can0的统计信息
ip -s link show can0

# 查看所有CAN接口的接收/发送情况
for i in 0 1 2 3; do
  echo "=== can$i ==="
  ip -s link show can$i
done
```

## 自定义配置

### 场景1: 不均匀分配

如果你需要不同的分配方式，例如：
- can0: 电机0-9 (10个)
- can1: 电机10-15 (6个)
- can2: 电机16-19 (4个)
- can3: 电机20-22 (3个)

修改配置文件：

```yaml
motor_can_interfaces:
  - "can0"  # 0
  - "can0"  # 1
  - "can0"  # 2
  - "can0"  # 3
  - "can0"  # 4
  - "can0"  # 5
  - "can0"  # 6
  - "can0"  # 7
  - "can0"  # 8
  - "can0"  # 9
  - "can1"  # 10
  - "can1"  # 11
  - "can1"  # 12
  - "can1"  # 13
  - "can1"  # 14
  - "can1"  # 15
  - "can2"  # 16
  - "can2"  # 17
  - "can2"  # 18
  - "can2"  # 19
  - "can3"  # 20
  - "can3"  # 21
  - "can3"  # 22
```

### 场景2: 只使用2个CAN接口

如果你只有2个CAN接口（can0和can1）：

```yaml
motor_can_interfaces:
  - "can0"  # 电机0-11使用can0
  - "can0"
  - "can0"
  - "can0"
  - "can0"
  - "can0"
  - "can0"
  - "can0"
  - "can0"
  - "can0"
  - "can0"
  - "can0"
  - "can1"  # 电机12-22使用can1
  - "can1"
  - "can1"
  - "can1"
  - "can1"
  - "can1"
  - "can1"
  - "can1"
  - "can1"
  - "can1"
  - "can1"
```

## CAN总线负载考虑

### 计算每条总线的负载

每个电机的通信包括：
- **发送**: 运动控制指令 (~8字节 + CAN帧开销)
- **接收**: 电机反馈数据 (~8字节 + CAN帧开销)

假设控制频率为100Hz，每个CAN帧约需：
- 标准帧: ~130位 (包括开销)
- 扩展帧: ~150位 (包括开销)

**单个电机的带宽需求** ≈ 150位 × 2 × 100Hz = 30kbps

**建议每条CAN总线的电机数量**:
- CAN 1Mbps: 最多10-15个电机
- CAN 500kbps: 最多5-8个电机

默认配置（6-6-6-5）在1Mbps波特率下是安全的。

## 运行和测试

### 1. 编译
```bash
cd ~/ros2_ws
colcon build --packages-select zrobot_bridge
```

### 2. 启动节点
```bash
source install/setup.bash
ros2 launch zrobot_bridge motor_controller.launch.py
```

### 3. 验证日志输出

启动后，日志应该显示每个电机使用的CAN接口：

```
[INFO] [motor_controller_node]: 初始化电机控制节点
[INFO] [motor_controller_node]: 主机ID: 0
[INFO] [motor_controller_node]: 开始初始化23个电机...
[DEBUG] [motor_controller_node]: 电机0初始化成功 (CAN接口: can0, CAN ID: 1, 类型: 0)
[DEBUG] [motor_controller_node]: 电机1初始化成功 (CAN接口: can0, CAN ID: 2, 类型: 0)
...
[DEBUG] [motor_controller_node]: 电机6初始化成功 (CAN接口: can1, CAN ID: 7, 类型: 0)
...
[DEBUG] [motor_controller_node]: 电机12初始化成功 (CAN接口: can2, CAN ID: 13, 类型: 0)
...
[DEBUG] [motor_controller_node]: 电机18初始化成功 (CAN接口: can3, CAN ID: 19, 类型: 0)
```

### 4. 监控CAN流量

可以使用 `candump` 工具监控每条CAN总线的流量：

```bash
# 在不同终端监控不同的CAN接口
candump can0
candump can1
candump can2
candump can3
```

## 故障排除

### 问题1: "No such device" 错误

**原因**: CAN接口未启用  
**解决**: 
```bash
sudo ip link set can0 up type can bitrate 1000000
# 对can1, can2, can3重复
```

### 问题2: 部分电机无响应

**检查**:
1. 确认对应的CAN接口已启用
2. 检查CAN ID是否正确
3. 使用 `candump` 查看是否有数据传输

### 问题3: CAN总线错误率高

**可能原因**:
- 总线负载过高
- 波特率设置不匹配
- 硬件连接问题

**解决**:
- 重新分配电机到不同的CAN接口
- 检查所有设备的波特率设置
- 检查CAN总线的终端电阻

## 启动脚本示例

创建一个自动启用所有CAN接口的脚本：

```bash
#!/bin/bash
# setup_can_interfaces.sh

echo "正在启用CAN接口..."

for i in 0 1 2 3; do
    echo "启用 can$i..."
    sudo ip link set can$i down 2>/dev/null
    sudo ip link set can$i up type can bitrate 1000000
    
    if [ $? -eq 0 ]; then
        echo "✓ can$i 已启用"
    else
        echo "✗ can$i 启用失败"
    fi
done

echo "所有CAN接口启用完成"
ip link show | grep can
```

使用方法：
```bash
chmod +x setup_can_interfaces.sh
./setup_can_interfaces.sh
```

## 性能优化建议

1. **平衡负载**: 尽量均匀分配电机到各CAN接口
2. **优先级分组**: 将关键电机放在同一CAN总线上
3. **降低频率**: 如果不需要高频控制，可以降低服务调用频率
4. **批量处理**: 服务已经支持批量控制所有23个电机

## 总结

✅ 已支持多CAN接口配置  
✅ 每个电机可独立指定CAN接口  
✅ 默认配置平均分配到4个CAN接口  
✅ 易于自定义和调整  
✅ 编译成功，无错误  

根据你的实际硬件连接情况修改 `config/motor_config.yaml` 中的 `motor_can_interfaces` 数组即可。
