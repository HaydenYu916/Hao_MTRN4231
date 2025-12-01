#!/usr/bin/env bash

# 永久配置有线网卡静态IP (192.168.0.77)
# 使用方法: sudo ./set_permanent_ip.sh

echo "正在配置永久静态IP..."

# 先设置所有参数，最后再设置method（一次性设置避免错误）
nmcli connection modify 'Wired connection 1' \
    ipv4.addresses '192.168.0.77/24' \
    ipv4.gateway '192.168.0.1' \
    ipv4.dns '8.8.8.8' \
    ipv4.method manual

echo "重新激活连接..."
# 检查连接是否活动
if nmcli connection show --active | grep -q "Wired connection 1"; then
    nmcli connection down 'Wired connection 1'
    sleep 1
fi
nmcli connection up 'Wired connection 1'

sleep 2
echo ""
echo "配置完成！"
echo "当前IP配置:"
ip addr show eno1 | grep "inet "

echo ""
echo "测试连接机器人..."
if ping -c 2 -W 2 192.168.0.100 >/dev/null 2>&1; then
    echo "✓ 成功连接到机器人！"
else
    echo "✗ 无法连接，请检查网络"
fi

