#!/usr/bin/env bash

# 配置有线网卡连接到机器人网络（192.168.0.xxx）
# 使用方法: sudo ./configure_robot_network.sh

echo "正在配置有线网卡连接到机器人网络..."

# 修改现有的有线连接，添加静态IP
nmcli connection modify "Wired connection 1" \
    ipv4.addresses 192.168.0.77/24 \
    ipv4.method manual \
    ipv4.gateway 192.168.0.1 \
    ipv4.dns "8.8.8.8,8.8.4.4"

# 激活连接
nmcli connection up "Wired connection 1"

echo "配置完成！"
echo "有线网卡IP已设置为: 192.168.0.77"
echo "请确保机器人IP为: 192.168.0.100"
echo ""
echo "查看IP配置: ip addr show eno1"

