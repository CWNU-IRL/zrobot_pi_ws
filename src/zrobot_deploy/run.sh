#!/bin/bash

# ZRobot Deploy 启动脚本
# 使用此脚本启动机器人运动控制系统

echo "======================================"
echo "  ZRobot 运动控制系统启动脚本"
echo "======================================"
echo ""

# 检查是否在ROS2工作空间中
if [ ! -d "/root/codes/zrobot_pi_ws/install/zrobot_deploy" ]; then
    echo "错误: 找不到 zrobot_deploy 软件包"
    echo "请先编译工作空间: colcon build --packages-select zrobot_deploy"
    exit 1
fi

# Source工作空间
echo "正在加载ROS2环境..."
source /root/codes/zrobot_pi_ws/install/setup.bash

echo ""
echo "检查 /rob_stride_control 服务..."
if ros2 service list | grep -q "rob_stride_control"; then
    echo "✓ /rob_stride_control 服务已运行"
else
    echo "⚠ 警告: /rob_stride_control 服务未找到"
    echo "  请确保底层控制服务正在运行"
    echo ""
    read -p "是否继续? (y/n): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "======================================"
echo "启动控制器..."
echo "======================================"
echo ""

# 运行控制器
ros2 run zrobot_deploy main
