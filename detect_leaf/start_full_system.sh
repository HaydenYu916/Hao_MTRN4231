#!/bin/bash

# 完整的叶子检测和可视化启动脚本 (新版本)
# 集成机械臂系统 - 只启动一个RViz窗口
echo "=== 启动完整系统：叶子检测 + 坐标转换 + 机械臂 ==="

# 设置环境
source /opt/ros/humble/setup.bash
cd /home/hao/Desktop/4231SuppliedCode/detect_leaf
source install/setup.bash

# 清理现有进程
echo "清理现有进程..."
pkill -f realsense2_camera 2>/dev/null || true
pkill -f leaf_detector 2>/dev/null || true
pkill -f leaf_to_arm_transformer 2>/dev/null || true
pkill -f ur_robot_driver 2>/dev/null || true
pkill -f moveit 2>/dev/null || true
pkill -f index_colab 2>/dev/null || true
pkill -f rviz2 2>/dev/null || true
sleep 2

# 启动RealSense相机
echo "启动RealSense相机..."
cd /home/hao/Desktop/4231SuppliedCode
./4231_scripts/camera.sh &
CAMERA_PID=$!
echo "✓ 相机PID: $CAMERA_PID"

# 等待相机启动
echo "等待相机启动 (15秒)..."
sleep 15

# 检查相机话题
echo ""
echo "=== 检查相机话题 ==="
CAMERA_READY=0
for i in {1..10}; do
    if ros2 topic list | grep -q "/camera/camera/color/image_raw"; then
        echo "✓ RealSense相机话题已就绪"
        CAMERA_READY=1
        break
    fi
    echo "⏳ 等待相机话题... ($i/10)"
    sleep 1
done

if [ $CAMERA_READY -eq 0 ]; then
    echo "⚠️  相机话题超时，但继续启动..."
fi

# 启动叶子检测
echo ""
echo "=== 启动叶子检测系统 ==="
cd /home/hao/Desktop/4231SuppliedCode/detect_leaf

# 启动ROS2叶子检测节点
echo "启动叶子检测节点 (leaf_detector)..."
ros2 run detect_leaf_pkg leaf_detector &
DETECTOR_PID=$!
echo "✓ 检测节点PID: $DETECTOR_PID"

# 启动坐标转换节点
echo "启动坐标转换节点 (leaf_to_arm_transformer)..."
ros2 run detect_leaf_pkg leaf_to_arm_transformer &
TRANSFORMER_PID=$!
echo "✓ 转换节点PID: $TRANSFORMER_PID"

# 等待检测节点启动并发布话题
echo "等待检测节点启动和话题发布 (20秒)..."
TOPICS_READY=0
for i in {1..20}; do
    if ros2 topic list | grep -q "/leaf_detection/annotated_image"; then
        echo "✓ 检测话题已发布"
        TOPICS_READY=1
        break
    fi
    echo "⏳ 等待检测话题... ($i/20)"
    sleep 1
done

if [ $TOPICS_READY -eq 0 ]; then
    echo "⚠️  检测话题超时，检查节点是否运行..."
    echo "活跃节点:"
    ros2 node list
fi

# 启动机械臂系统 (使用setupFakeur5e.sh)
echo ""
echo "=== 启动机械臂系统 (使用setupFakeur5e.sh) ==="
echo "启动UR 5e驱动和MoveIt..."

# 使用setupFakeur5e.sh来启动机械臂和MoveIt
# 但不启动它的RViz窗口（我们只用一个）
cd /home/hao/Desktop/4231SuppliedCode/4231_scripts

# 创建临时脚本调用setupFakeur5e但不启动RViz
bash -c 'source /opt/ros/humble/setup.bash && \
  ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy \
  initial_joint_controller:=joint_trajectory_controller \
  use_fake_hardware:=true launch_rviz:=false &
  sleep 5 && \
  ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e launch_rviz:=false use_fake_hardware:=true &' &

DRIVER_PID=$!
echo "✓ 机械臂系统已启动（后台）"

# 等待驱动启动
sleep 10

# 显示系统状态
echo ""
echo "=== 系统状态 ==="
echo "相机话题:"
ros2 topic list | grep image || echo "⚠️  未找到image话题"

echo ""
echo "检测话题:"
ros2 topic list | grep leaf || echo "⚠️  未找到leaf话题"

echo ""
echo "坐标转换话题:"
ros2 topic list | grep arm_target || echo "⚠️  未找到arm_target话题"

# 启动单个RViz2 (集成所有显示)
echo ""
echo "=== 启动统一的RViz2可视化 ==="
echo "启动RViz2..."

CONFIG_FILE="/home/hao/Desktop/4231SuppliedCode/detect_leaf/annotated_image_with_robot.rviz"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "⚠️  集成配置文件不存在，使用默认配置: $CONFIG_FILE"
    CONFIG_FILE="/home/hao/Desktop/4231SuppliedCode/detect_leaf/annotated_image.rviz"
fi

echo "✓ 使用配置文件: $CONFIG_FILE"
rviz2 -d "$CONFIG_FILE" &

RVIZ_PID=$!
echo "✓ RViz2 PID: $RVIZ_PID"

# 等待RViz启动
echo "等待RViz2启动 (10秒)..."
sleep 10

# 启动坐标信息监听器
echo "启动坐标信息监听器..."
ros2 run detect_leaf_pkg coordinates_display &
COORDS_PID=$!
echo "✓ 坐标显示器 PID: $COORDS_PID"

echo ""
echo "=== 系统已启动 ==="
echo "进程ID:"
echo "  相机: $CAMERA_PID"
echo "  检测: $DETECTOR_PID"
echo "  转换: $TRANSFORMER_PID"
echo "  机械臂驱动: $DRIVER_PID"
echo "  MoveIt: $MOVEIT_PID"
echo "  RViz2: $RVIZ_PID"
echo "  坐标显示: $COORDS_PID"
echo ""
echo "功能说明:"
echo "• 完整集成系统（叶子检测 + 坐标转换 + 机械臂）"
echo "• 发布话题:"
echo "  - /leaf_detection/coordinates (像素坐标)"
echo "  - /leaf_detection/arm_target (机械臂坐标) ⭐"
echo "  - /leaf_detection/annotated_image (标注图像)"
echo ""
echo "RViz显示:"
echo "  - Camera Image: /camera/camera/color/image_raw"
echo "  - Annotated Image: /leaf_detection/annotated_image"
echo "  - Robot Model: UR 5e机械臂"
echo "  - Leaf Targets: 叶子目标位置"
echo "  - TF Frames: 所有坐标系"
echo ""
echo "诊断命令:"
echo "  ros2 topic list                # 列出所有话题"
echo "  ros2 topic echo /leaf_detection/arm_target  # 查看机械臂坐标"
echo "  ros2 node list                # 列出所有节点"
echo ""
echo "按 Ctrl+C 停止所有进程"

# 清理函数
cleanup() {
    echo ""
    echo "=== 清理进程 ==="
    kill $CAMERA_PID 2>/dev/null || true
    kill $DETECTOR_PID 2>/dev/null || true
    kill $TRANSFORMER_PID 2>/dev/null || true
    kill $DRIVER_PID 2>/dev/null || true
    kill $MOVEIT_PID 2>/dev/null || true
    kill $RVIZ_PID 2>/dev/null || true
    kill $COORDS_PID 2>/dev/null || true
    pkill -f realsense2_camera 2>/dev/null || true
    pkill -f leaf_detector 2>/dev/null || true
    pkill -f leaf_to_arm_transformer 2>/dev/null || true
    pkill -f ur_robot_driver 2>/dev/null || true
    pkill -f moveit 2>/dev/null || true
    pkill -f coordinates_display 2>/dev/null || true
    pkill -f index_colab 2>/dev/null || true
    pkill -f rviz2 2>/dev/null || true
    echo "✓ 所有进程已停止"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 等待进程
wait
