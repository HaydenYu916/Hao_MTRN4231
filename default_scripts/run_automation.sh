#!/bin/bash
# Automation task runner script
# Run automated leaf detection and robot arm movement

echo "=========================================="
echo "启动自动化任务"
echo "=========================================="

# Get script directory, then get workspace root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WORKSPACE_DIR"

# Fix Conda and ROS2 library conflicts: prioritize system libraries
fix_library_path() {
    if [ -n "$CONDA_PREFIX" ]; then
        SYSTEM_LIB_PATH="/usr/lib/x86_64-linux-gnu"
        if [ -n "$LD_LIBRARY_PATH" ]; then
            export LD_LIBRARY_PATH="${SYSTEM_LIB_PATH}:${CONDA_PREFIX}/lib:${LD_LIBRARY_PATH}"
        else
            export LD_LIBRARY_PATH="${SYSTEM_LIB_PATH}:${CONDA_PREFIX}/lib"
        fi
    fi
}

# Fix library path at script startup
fix_library_path

# Parse command line arguments first (before checking system)
MIN_AREA="0.0"
CONFIDENCE="0.0"
OFFSET_Z="0.05"
HOME_X="0.25"
HOME_Y="0.10"
HOME_Z="0.55"

while [[ $# -gt 0 ]]; do
    case $1 in
        --min-area)
            MIN_AREA="$2"
            shift 2
            ;;
        --confidence)
            CONFIDENCE="$2"
            shift 2
            ;;
        --offset-z)
            OFFSET_Z="$2"
            shift 2
            ;;
        --home-x)
            HOME_X="$2"
            shift 2
            ;;
        --home-y)
            HOME_Y="$2"
            shift 2
            ;;
        --home-z)
            HOME_Z="$2"
            shift 2
            ;;
        --help)
            echo "用法: $0 [选项]"
            echo ""
            echo "选项:"
            echo "  --min-area VALUE     叶子检测最小面积阈值 (默认: 0.0)"
            echo "  --confidence VALUE   叶子检测置信度阈值 (默认: 0.0)"
            echo "  --offset-z VALUE     机械臂Z轴偏移量/米 (默认: 0.05)"
            echo "  --home-x VALUE       原位X坐标/米 (默认: 0.25)"
            echo "  --home-y VALUE       原位Y坐标/米 (默认: 0.10)"
            echo "  --home-z VALUE       原位Z坐标/米 (默认: 0.55)"
            echo "  --help               显示此帮助信息"
            echo ""
            echo "示例:"
            echo "  $0                              # 使用默认参数"
            echo "  $0 --min-area 2000 --offset-z 0.1   # 自定义参数"
            echo "  $0 --home-x 0.3 --home-y 0.2 --home-z 0.6   # 自定义原位"
            exit 0
            ;;
        *)
            echo "未知参数: $1"
            echo "使用 --help 查看帮助信息"
            exit 1
            ;;
    esac
done

# Source workspace
echo "加载工作空间..."
if [ ! -f "install/setup.bash" ]; then
    echo "❌ 错误: install/setup.bash 文件不存在！"
    echo "请先运行 ./default_scripts/start_all.sh 启动系统"
    exit 1
fi
source install/setup.bash || {
    echo "❌ 错误: 加载工作空间失败！"
    exit 1
}

# Check if required services are running
echo "检查系统状态..."
sleep 2

if ! ros2 service list | grep -q "leaf_detection_srv"; then
    echo "⚠️  警告: 叶子检测服务未运行"
    echo "正在等待服务启动... (最多30秒)"
    
    # Wait up to 30 seconds for service
    for i in {1..30}; do
        if ros2 service list | grep -q "leaf_detection_srv"; then
            echo "✓ 叶子检测服务已就绪"
            break
        fi
        sleep 1
        if [ $i -eq 30 ]; then
            echo "❌ 错误: 叶子检测服务超时未响应"
            echo "请确保已运行 ./default_scripts/start_all.sh 启动所有系统组件"
            exit 1
        fi
    done
else
    echo "✓ 叶子检测服务正常运行"
fi

# Start automation task
echo ""
echo "=========================================="
echo "启动自动化任务"
echo "=========================================="
echo "参数:"
echo "  - 最小面积: $MIN_AREA"
echo "  - 置信度: $CONFIDENCE"
echo "  - Z轴偏移: $OFFSET_Z m"
echo "  - 原位坐标: x=$HOME_X, y=$HOME_Y, z=$HOME_Z m"
echo "=========================================="
echo ""

# Run in new terminal or current terminal
if command -v gnome-terminal &> /dev/null; then
    echo "在新终端中启动自动化任务..."
    gnome-terminal -t "AutomationTask" -e "bash -c 'cd \"$WORKSPACE_DIR\" && fix_library_path() { if [ -n \"\$CONDA_PREFIX\" ]; then SYSTEM_LIB_PATH=\"/usr/lib/x86_64-linux-gnu\"; if [ -n \"\$LD_LIBRARY_PATH\" ]; then export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib:\${LD_LIBRARY_PATH}\"; else export LD_LIBRARY_PATH=\"\${SYSTEM_LIB_PATH}:\${CONDA_PREFIX}/lib\"; fi; fi; }; fix_library_path && source install/setup.bash && ros2 launch task_automation automation_task.launch.py min_area:=$MIN_AREA confidence:=$CONFIDENCE offset_z:=$OFFSET_Z home_x:=$HOME_X home_y:=$HOME_Y home_z:=$HOME_Z; exec bash'"
    echo "自动化任务已在新终端中启动"
else
    echo "在当前终端中启动自动化任务..."
    ros2 launch task_automation automation_task.launch.py \
        min_area:=$MIN_AREA \
        confidence:=$CONFIDENCE \
        offset_z:=$OFFSET_Z \
        home_x:=$HOME_X \
        home_y:=$HOME_Y \
        home_z:=$HOME_Z
fi

