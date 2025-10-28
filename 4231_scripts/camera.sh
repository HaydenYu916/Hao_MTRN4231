#!/bin/bash

# RealSense相机启动脚本
# 使用方法: ./camera.sh

echo "=== 启动RealSense相机 ==="

# 设置ROS2环境
source /opt/ros/humble/setup.bash

echo "启动RealSense相机..."
# 检查RealSense包是否可用
if ros2 pkg list | grep -q "realsense2_camera"; then
    ros2 launch realsense2_camera rs_launch.py \
        enable_color:=true \
        enable_depth:=true \
        color_width:=640 \
        color_height:=480 \
        color_fps:=30.0 \
        align_depth.enable:=true
else
    echo "错误: RealSense包未安装"
    echo "请安装: sudo apt install ros-humble-realsense2-camera"
    exit 1
fi
