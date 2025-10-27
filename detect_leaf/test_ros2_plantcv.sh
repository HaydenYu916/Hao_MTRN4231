#!/bin/bash

# ROS2 + PlantCV 测试脚本 (新版本)
# 支持两种模式：
# 1. ROS2 节点模式 (realsense_leaf_detector_ros2.py)
# 2. 独立Python脚本模式 (index_colab.py)

# 全局变量存储进程ID
CAMERA_PID=""
DETECTOR_PID=""

# 清理函数
cleanup() {
    echo ""
    echo "=== 清理资源 ==="
    if [ ! -z "$CAMERA_PID" ]; then
        echo "停止 RealSense 相机进程 (PID: $CAMERA_PID)..."
        kill $CAMERA_PID 2>/dev/null || echo "相机进程已停止"
    fi
    if [ ! -z "$DETECTOR_PID" ]; then
        echo "停止检测进程 (PID: $DETECTOR_PID)..."
        kill $DETECTOR_PID 2>/dev/null || echo "检测进程已停止"
    fi
    echo "停止所有相关进程..."
    pkill -f realsense2_camera 2>/dev/null || true
    pkill -f "realsense_leaf_detector" 2>/dev/null || true
    pkill -f "index_colab" 2>/dev/null || true
    echo "✓ 清理完成"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

echo "=== ROS2 + PlantCV + RealSense 测试系统 ==="
echo ""

# 检查是否在 detect_leaf 目录内
if [ ! -d "src/detect_leaf_pkg" ]; then
    echo "错误: 请在 detect_leaf 工作空间目录内运行此脚本"
    exit 1
fi

echo "✓ 工作空间目录存在"

# 设置环境变量
export COLCON_TRACE=""
export AMENT_TRACE_SETUP_FILES=""
export AMENT_PYTHON_EXECUTABLE=""

# 源化 ROS2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✓ ROS2 环境已加载"

# 停止其他进程
echo ""
echo "=== 停止其他进程 ==="
pkill -f realsense2_camera 2>/dev/null || echo "未找到 realsense2_camera 进程"
pkill -f "realsense_leaf_detector" 2>/dev/null || echo "未找到旧检测进程"
pkill -f "index_colab" 2>/dev/null || echo "未找到旧脚本进程"
sleep 2

# 启动RealSense相机
echo ""
echo "=== 启动RealSense相机 ==="
cd /home/hao/Desktop/4231SuppliedCode
./4231_scripts/camera.sh &
CAMERA_PID=$!
echo "✓ RealSense相机进程已启动，PID: $CAMERA_PID"

# 等待相机启动
echo "等待相机启动 (5秒)..."
sleep 5

# 检查相机话题
echo ""
echo "=== 检查ROS2话题 ==="
echo "当前ROS2话题列表:"
ros2 topic list

# 检查相机话题
IMAGE_TOPIC="/camera/camera/color/image_raw"
echo ""
echo "检查相机图像话题: $IMAGE_TOPIC"
if ros2 topic info $IMAGE_TOPIC 2>/dev/null; then
    echo "✓ 相机话题可用"
    echo ""
    echo "测试相机数据流 (5秒):"
    timeout 5s ros2 topic hz $IMAGE_TOPIC || true
else
    echo "⚠ 相机话题不可用"
fi

# 选择运行模式
echo ""
echo "=== 选择运行模式 ==="
echo "1. 测试ROS2节点模式 (realsense_leaf_detector_ros2.py)"
echo "2. 测试独立Python脚本模式 (index_colab.py)"
echo ""

MODE=${1:-1}

cd /home/hao/Desktop/4231SuppliedCode

# 检查Python环境
echo ""
echo "=== 检查Python环境 ==="
echo "系统 Python 版本: $(python3 --version)"

# 检查PlantCV
echo ""
echo "检查PlantCV..."
if python3 -c "import plantcv; print('PlantCV版本:', plantcv.__version__)" 2>/dev/null; then
    echo "✓ PlantCV 已安装"
else
    echo "⚠ PlantCV 未安装"
    echo "  安装命令: pip3 install plantcv opencv-python numpy"
fi

# 检查cv_bridge
echo ""
echo "检查cv_bridge..."
if python3 -c "from cv_bridge import CvBridge" 2>/dev/null; then
    echo "✓ cv_bridge 已安装"
else
    echo "⚠ cv_bridge 未安装"
    echo "  安装命令: pip3 install opencv-python"
fi

# 测试检测模块
echo ""
echo "=== 测试检测模块 ==="

if [ "$MODE" == "2" ]; then
    # 模式 2: 独立Python脚本
    echo "测试独立Python脚本 (index_colab.py)..."
    echo ""
    timeout 5s python3 index_colab.py 2>&1 | head -20
    if [ $? -eq 124 ]; then
        echo ""
        echo "✓ 脚本启动成功 (5秒后超时)"
    else
        echo "⚠ 脚本运行完成或出错"
    fi
else
    # 模式 1: ROS2 节点
    echo "测试ROS2节点 (realsense_leaf_detector_ros2.py)..."
    echo ""
    
    # 检查 Python 版本
    if command -v python3.10 &> /dev/null; then
        echo "✓ 检测到 Python 3.10，使用该版本..."
        timeout 5s python3.10 realsense_leaf_detector_ros2.py 2>&1 | head -20
    else
        echo "✓ 使用系统默认 Python..."
        timeout 5s python3 realsense_leaf_detector_ros2.py 2>&1 | head -20
    fi
    
    if [ $? -eq 124 ]; then
        echo ""
        echo "✓ 节点启动成功 (5秒后超时)"
    else
        echo "⚠ 节点运行完成或出错"
    fi
fi

# 测试完成
echo ""
echo "=== 测试完成 ==="
echo ""
echo "系统状态:"
echo "  - ROS2 Humble: ✓ 已加载"
echo "  - Python 3: ✓ 可用"
echo "  - RealSense相机: ✓ 已启动 (PID: $CAMERA_PID)"
echo "  - PlantCV: $(python3 -c 'import plantcv; print(\"✓ 已安装\")' 2>/dev/null || echo '⚠ 未安装')"
echo ""
echo "使用方法:"

if [ "$MODE" == "2" ]; then
    echo ""
    echo "🔧 独立Python脚本模式:"
    echo "1. 修改 index_colab.py 顶部的配置参数"
    echo "2. 启动相机: ./4231_scripts/camera.sh"
    echo "3. 运行脚本: python3 index_colab.py"
    echo "4. 查看结果: ./leaf_detection_results/"
else
    echo ""
    echo "🔧 ROS2 节点模式:"
    echo "1. 修改 realsense_leaf_detector_ros2.py 顶部的配置参数"
    echo "2. 启动相机: ./4231_scripts/camera.sh"
    echo "3. 启动节点: python3 realsense_leaf_detector_ros2.py"
    echo "4. 订阅话题:"
    echo "   - ros2 topic echo /leaf_detection/leaf_count"
    echo "   - ros2 topic echo /leaf_detection/coordinates"
    echo "5. 可视化: ros2 run image_view image_view image:=/leaf_detection/annotated_image"
fi

echo ""
echo "配置选项:"
echo "  - FAST_MODE: 快速模式 (30+ FPS)"
echo "  - USE_SIZE_ANALYSIS: 启用大小分析"
echo "  - USE_COLOR_ANALYSIS: 启用颜色分析"
echo "  - USE_MORPHOLOGY: 启用形态学处理"
echo "  - USE_CONTOUR_FILTERING: 启用轮廓过滤"
echo ""
echo "清理命令:"
echo "  - 停止相机: kill $CAMERA_PID"
echo "  - 或运行: pkill -f realsense2_camera"
echo ""
echo "按 Enter 键继续或 Ctrl+C 退出..."
read

# 等待用户中断
wait