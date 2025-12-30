#!/bin/bash
# setup_can_interfaces.sh
# 自动启用所有CAN接口的脚本

echo "========================================="
echo "  启用多个CAN接口（can0-can3）"
echo "========================================="
echo ""

# 默认波特率 (1Mbps)
BITRATE=${1:-1000000}

echo "使用波特率: ${BITRATE} bps"
echo ""

# 启用can0-can3
for i in 0 1 2 3; do
    echo -n "启用 can$i ... "
    
    # 先关闭接口（如果已经启用）
    sudo ip link set can$i down 2>/dev/null
    
    # 启用接口
    if sudo ip link set can$i up type can bitrate ${BITRATE} 2>/dev/null; then
        echo "✓ 成功"
    else
        echo "✗ 失败 (可能不存在)"
    fi
done

echo ""
echo "========================================="
echo "  CAN接口状态"
echo "========================================="
ip link show | grep -A 1 "^[0-9]*: can"

echo ""
echo "完成！"
