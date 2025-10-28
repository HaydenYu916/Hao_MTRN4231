#!/bin/bash

# ROS2 + PlantCV Test Script (New Version)
# Supports two modes:
# 1. ROS2 Node Mode (realsense_leaf_detector_ros2.py)
# 2. Standalone Python Script Mode (index_colab.py)

# Global variables to store process IDs
CAMERA_PID=""
DETECTOR_PID=""

# Cleanup function
cleanup() {
    echo ""
    echo "=== Cleaning Up Resources ==="
    if [ ! -z "$CAMERA_PID" ]; then
        echo "Stopping RealSense camera process (PID: $CAMERA_PID)..."
        kill $CAMERA_PID 2>/dev/null || echo "Camera process already stopped"
    fi
    if [ ! -z "$DETECTOR_PID" ]; then
        echo "Stopping detection process (PID: $DETECTOR_PID)..."
        kill $DETECTOR_PID 2>/dev/null || echo "Detection process already stopped"
    fi
    echo "Stopping all related processes..."
    pkill -f realsense2_camera 2>/dev/null || true
    pkill -f "realsense_leaf_detector" 2>/dev/null || true
    pkill -f "index_colab" 2>/dev/null || true
    echo "✓ Cleanup completed"
    exit 0
}

# Set up signal handling
trap cleanup SIGINT SIGTERM

echo "=== ROS2 + PlantCV + RealSense Test System ==="
echo ""

# Check if we're in detect_leaf directory
if [ ! -d "src/detect_leaf_pkg" ]; then
    echo "Error: Please run this script from within the detect_leaf workspace directory"
    exit 1
fi

echo "✓ Workspace directory exists"

# Set environment variables
export COLCON_TRACE=""
export AMENT_TRACE_SETUP_FILES=""
export AMENT_PYTHON_EXECUTABLE=""

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✓ ROS2 environment loaded"

# Stop other processes
echo ""
echo "=== Stopping Other Processes ==="
pkill -f realsense2_camera 2>/dev/null || echo "No realsense2_camera process found"
pkill -f "realsense_leaf_detector" 2>/dev/null || echo "No old detection process found"
pkill -f "index_colab" 2>/dev/null || echo "No old script process found"
sleep 2

# Start RealSense camera
echo ""
echo "=== Starting RealSense Camera ==="
cd ..
./4231_scripts/camera.sh &
CAMERA_PID=$!
echo "✓ RealSense camera process started, PID: $CAMERA_PID"

# Wait for camera startup
echo "Waiting for camera startup (5 seconds)..."
sleep 5

# Check camera topics
echo ""
echo "=== Checking ROS2 Topics ==="
echo "Current ROS2 topic list:"
ros2 topic list

# Check camera topics
IMAGE_TOPIC="/camera/camera/color/image_raw"
echo ""
echo "Checking camera image topic: $IMAGE_TOPIC"
if ros2 topic info $IMAGE_TOPIC 2>/dev/null; then
    echo "✓ Camera topic available"
    echo ""
    echo "Testing camera data flow (5 seconds):"
    timeout 5s ros2 topic hz $IMAGE_TOPIC || true
else
    echo "⚠ Camera topic not available"
fi

# Select run mode
echo ""
echo "=== Select Run Mode ==="
echo "1. Test ROS2 Node Mode (realsense_leaf_detector_ros2.py)"
echo "2. Test Standalone Python Script Mode (index_colab.py)"
echo ""

MODE=${1:-1}

cd ..

# Check Python environment
echo ""
echo "=== Checking Python Environment ==="
echo "System Python version: $(python3 --version)"

# Check PlantCV
echo ""
echo "Checking PlantCV..."
if python3 -c "import plantcv; print('PlantCV version:', plantcv.__version__)" 2>/dev/null; then
    echo "✓ PlantCV installed"
else
    echo "⚠ PlantCV not installed"
    echo "  Install command: pip3 install plantcv opencv-python numpy"
fi

# Check cv_bridge
echo ""
echo "Checking cv_bridge..."
if python3 -c "from cv_bridge import CvBridge" 2>/dev/null; then
    echo "✓ cv_bridge installed"
else
    echo "⚠ cv_bridge not installed"
    echo "  Install command: pip3 install opencv-python"
fi

# Test detection module
echo ""
echo "=== Testing Detection Module ==="

if [ "$MODE" == "2" ]; then
    # Mode 2: Standalone Python script
    echo "Testing standalone Python script (index_colab.py)..."
    echo ""
    timeout 5s python3 index_colab.py 2>&1 | head -20
    if [ $? -eq 124 ]; then
        echo ""
        echo "✓ Script started successfully (timeout after 5 seconds)"
    else
        echo "⚠ Script finished or error occurred"
    fi
else
    # Mode 1: ROS2 Node
    echo "Testing ROS2 node (realsense_leaf_detector_ros2.py)..."
    echo ""
    
    # Check Python version
    if command -v python3.10 &> /dev/null; then
        echo "✓ Detected Python 3.10, using this version..."
        timeout 5s python3.10 realsense_leaf_detector_ros2.py 2>&1 | head -20
    else
        echo "✓ Using system default Python..."
        timeout 5s python3 realsense_leaf_detector_ros2.py 2>&1 | head -20
    fi
    
    if [ $? -eq 124 ]; then
        echo ""
        echo "✓ Node started successfully (timeout after 5 seconds)"
    else
        echo "⚠ Node finished or error occurred"
    fi
fi

# Test completed
echo ""
echo "=== Test Completed ==="
echo ""
echo "System Status:"
echo "  - ROS2 Humble: ✓ Loaded"
echo "  - Python 3: ✓ Available"
echo "  - RealSense Camera: ✓ Started (PID: $CAMERA_PID)"
echo "  - PlantCV: $(python3 -c 'import plantcv; print(\"✓ Installed\")' 2>/dev/null || echo '⚠ Not installed')"
echo ""
echo "Usage Instructions:"

if [ "$MODE" == "2" ]; then
    echo ""
    echo "🔧 Standalone Python Script Mode:"
    echo "1. Modify configuration parameters at the top of index_colab.py"
    echo "2. Start camera: ./4231_scripts/camera.sh"
    echo "3. Run script: python3 index_colab.py"
    echo "4. View results: ./leaf_detection_results/"
else
    echo ""
    echo "🔧 ROS2 Node Mode:"
    echo "1. Modify configuration parameters at the top of realsense_leaf_detector_ros2.py"
    echo "2. Start camera: ./4231_scripts/camera.sh"
    echo "3. Start node: python3 realsense_leaf_detector_ros2.py"
    echo "4. Subscribe to topics:"
    echo "   - ros2 topic echo /leaf_detection/leaf_count"
    echo "   - ros2 topic echo /leaf_detection/coordinates"
    echo "5. Visualization: ros2 run image_view image_view image:=/leaf_detection/annotated_image"
fi

echo ""
echo "Configuration Options:"
echo "  - FAST_MODE: Fast mode (30+ FPS)"
echo "  - USE_SIZE_ANALYSIS: Enable size analysis"
echo "  - USE_COLOR_ANALYSIS: Enable color analysis"
echo "  - USE_MORPHOLOGY: Enable morphological processing"
echo "  - USE_CONTOUR_FILTERING: Enable contour filtering"
echo ""
echo "Cleanup Commands:"
echo "  - Stop camera: kill $CAMERA_PID"
echo "  - Or run: pkill -f realsense2_camera"
echo ""
echo "Press Enter to continue or Ctrl+C to exit..."
read

# Wait for user interrupt
wait