#!/usr/bin/env bash

# 机器人网络连接诊断和配置脚本
# 使用方法: sudo ./fix_robot_network.sh

set -e

echo "=========================================="
echo "机器人网络连接诊断和配置"
echo "=========================================="
echo ""

# 检查网线连接
echo "1. 检查网线连接..."
if [ -f /sys/class/net/eno1/carrier ]; then
    CARRIER=$(cat /sys/class/net/eno1/carrier)
    if [ "$CARRIER" = "1" ]; then
        echo "   ✓ 网线已连接"
    else
        echo "   ✗ 网线未连接！请检查网线"
        exit 1
    fi
else
    echo "   ? 无法检测网线状态"
fi
echo ""

# 设置临时IP
echo "2. 设置临时IP地址 (192.168.0.77)..."
ip addr flush dev eno1 2>/dev/null || true
ip addr add 192.168.0.77/24 dev eno1
ip link set eno1 up
sleep 2
echo "   ✓ IP已设置"
echo ""

# 显示当前IP
echo "3. 当前网络配置:"
ip addr show eno1 | grep "inet " || echo "   警告: 未检测到IPv4地址"
echo ""

# 测试连接
echo "4. 测试连接到机器人 (192.168.0.100)..."
if ping -c 2 -W 2 192.168.0.100 >/dev/null 2>&1; then
    echo "   ✓ 成功连接到机器人！"
    ping -c 3 192.168.0.100
else
    echo "   ✗ 无法ping通机器人"
    echo ""
    echo "5. 扫描网络查找设备..."
    if command -v nmap >/dev/null 2>&1; then
        echo "   正在扫描 192.168.0.0/24 网段..."
        nmap -sn 192.168.0.0/24 2>/dev/null | grep -E "Nmap scan|192.168.0" | head -20
    else
        echo "   nmap未安装，跳过扫描"
        echo "   可以手动安装: sudo apt install nmap"
    fi
    echo ""
    echo "6. 检查ARP表:"
    arp -a | grep 192.168.0 || echo "   未发现192.168.0网段的设备"
    echo ""
    echo "可能的原因:"
    echo "  - 机器人IP不是192.168.0.100（请检查机器人示教器）"
    echo "  - 机器人未开机或网络未配置"
    echo "  - 需要交换机/路由器但未连接"
    echo "  - 防火墙阻止"
fi
echo ""

# 询问是否配置永久IP
echo "=========================================="
echo "如果需要永久配置静态IP，运行以下命令:"
echo "  sudo nmcli connection modify 'Wired connection 1' ipv4.method manual"
echo "  sudo nmcli connection modify 'Wired connection 1' ipv4.addresses '192.168.0.77/24'"
echo "  sudo nmcli connection modify 'Wired connection 1' ipv4.gateway '192.168.0.1'"
echo "  sudo nmcli connection down 'Wired connection 1'"
echo "  sudo nmcli connection up 'Wired connection 1'"
echo "=========================================="

