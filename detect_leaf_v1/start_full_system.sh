#!/bin/bash

# Complete leaf detection and visualization startup script (new version)
echo "=== Starting Complete Leaf Detection and Visualization System (RealSense + PlantCV) ==="

# Set up environment
source /opt/ros/humble/setup.bash
cd /home/hao/Desktop/4231SuppliedCode/detect_leaf
source install/setup.bash

# Clean up existing processes
echo "Cleaning up existing processes..."
pkill -f realsense2_camera 2>/dev/null || true
pkill -f leaf_detector 2>/dev/null || true
pkill -f index_colab 2>/dev/null || true
pkill -f rviz2 2>/dev/null || true
sleep 2

# Start RealSense camera
echo "Starting RealSense camera..."
cd /home/hao/Desktop/4231SuppliedCode
./4231_scripts/camera.sh &
CAMERA_PID=$!
echo "✓ Camera PID: $CAMERA_PID"

# Wait for camera startup
echo "Waiting for camera startup (15 seconds)..."
sleep 15

# Check camera topics
echo ""
echo "=== Checking Camera Topics ==="
CAMERA_READY=0
for i in {1..10}; do
    if ros2 topic list | grep -q "/camera/camera/color/image_raw"; then
        echo "✓ RealSense camera topics ready"
        CAMERA_READY=1
        break
    fi
    echo "⏳ Waiting for camera topics... ($i/10)"
    sleep 1
done

if [ $CAMERA_READY -eq 0 ]; then
    echo "⚠️  Camera topics timeout, but continuing startup..."
fi

# Startup mode selection
echo ""
echo "=== Select Startup Mode ==="
echo "1. ROS2 Node Mode (leaf_detector + RViz) - Recommended for RViz display"
echo "2. Standalone Python Script Mode (index_colab.py)"
echo "Using ROS2 Node Mode by default..."

MODE=${1:-1}

if [ "$MODE" == "2" ]; then
    # Mode 2: Standalone Python script
    echo ""
    echo "Starting standalone Python script (index_colab.py)..."
    cd /home/hao/Desktop/4231SuppliedCode
    python3 index_colab.py &
    DETECTOR_PID=$!
    echo "✓ Detection script PID: $DETECTOR_PID"
else
    # Mode 1: ROS2 Node (using leaf_detector)
    echo ""
    echo "Starting ROS2 leaf detection node (leaf_detector)..."
    cd /home/hao/Desktop/4231SuppliedCode/detect_leaf
    
    # Start ROS2 leaf detection node
    ros2 run detect_leaf_pkg leaf_detector &
    DETECTOR_PID=$!
    echo "✓ Detection node PID: $DETECTOR_PID"
    
    # Wait for detection node to start and publish topics
    echo "Waiting for detection node startup and topic publishing (20 seconds)..."
    TOPICS_READY=0
    for i in {1..20}; do
        if ros2 topic list | grep -q "/leaf_detection/annotated_image"; then
            echo "✓ Detection topics published"
            TOPICS_READY=1
            break
        fi
        echo "⏳ Waiting for detection topics... ($i/20)"
        sleep 1
    done
    
    if [ $TOPICS_READY -eq 0 ]; then
        echo "⚠️  Detection topics timeout, checking if nodes are running..."
        echo "Active nodes:"
        ros2 node list
    fi
fi

# Display system status
echo ""
echo "=== System Status ==="
echo "Camera topics:"
ros2 topic list | grep image || echo "⚠️  No image topics found"

echo ""
echo "Detection topics:"
ros2 topic list | grep leaf || echo "⚠️  No leaf topics found"

# Start RViz2 (if in ROS2 mode)
echo ""
echo "=== Starting RViz2 Visualization ==="
if [ "$MODE" == "2" ]; then
    echo "✓ Using standalone script mode (no ROS2 topics, RViz not started)"
else
    echo "Starting RViz2..."
    
    # Use absolute path configuration file
    CONFIG_FILE="/home/hao/Desktop/4231SuppliedCode/detect_leaf/annotated_image.rviz"
    
    if [ ! -f "$CONFIG_FILE" ]; then
        echo "⚠️  Configuration file does not exist: $CONFIG_FILE"
        echo "Starting RViz2 with default configuration..."
        rviz2 &
    else
        echo "✓ Using configuration file: $CONFIG_FILE"
        rviz2 -d "$CONFIG_FILE" &
    fi
    
    RVIZ_PID=$!
    echo "✓ RViz2 PID: $RVIZ_PID"
    
    # Wait for RViz startup
    echo "Waiting for RViz2 startup (10 seconds)..."
    sleep 10
    
    # Start coordinate information listener
    echo "Starting coordinate information listener..."
    ros2 run detect_leaf_pkg coordinates_display &
    COORDS_PID=$!
    echo "✓ Coordinate display PID: $COORDS_PID"
fi

echo ""
echo "=== System Started ==="
echo "Process IDs:"
echo "  Camera: $CAMERA_PID"
echo "  Detection: $DETECTOR_PID"
if [ ! -z "$RVIZ_PID" ]; then
    echo "  RViz2: $RVIZ_PID"
fi
echo "  Coordinate Display: $COORDS_PID"
echo ""
echo "Feature Description:"
if [ "$MODE" == "2" ]; then
    echo "• Mode: Standalone Python script"
    echo "• Output: Real-time display window + coordinate information"
    echo "• Save: ./leaf_detection_results/"
else
    echo "• Mode: ROS2 Node (leaf_detector)"
    echo "• Published Topics:"
    echo "  - /leaf_detection/annotated_image (📊 with colored bounding boxes and labels)"
    echo "  - /leaf_detection/leaf_count"
    echo "  - /leaf_detection/coordinates"
    echo "  - /leaf_bounding_boxes"
    echo ""
    echo "RViz Display:"
    echo "  - Camera Image: /camera/camera/color/image_raw"
    echo "  - Annotated Image: /leaf_detection/annotated_image (✨ colored annotations)"
    echo "  - Leaf Bounding Boxes: /leaf_bounding_boxes"
fi
echo ""
echo "Diagnostic Commands:"
echo "  ros2 topic list                # List all topics"
echo "  ros2 topic hz /leaf_detection/annotated_image  # Check frame rate"
echo ""
echo "Press Ctrl+C to stop all processes"

# Cleanup function
cleanup() {
    echo ""
    echo "=== Cleaning Up Processes ==="
    kill $CAMERA_PID 2>/dev/null || true
    kill $DETECTOR_PID 2>/dev/null || true
    if [ ! -z "$RVIZ_PID" ]; then
        kill $RVIZ_PID 2>/dev/null || true
    fi
    if [ ! -z "$COORDS_PID" ]; then
        kill $COORDS_PID 2>/dev/null || true
    fi
    pkill -f realsense2_camera 2>/dev/null || true
    pkill -f leaf_detector 2>/dev/null || true
    pkill -f coordinates_display 2>/dev/null || true
    pkill -f index_colab 2>/dev/null || true
    pkill -f rviz2 2>/dev/null || true
    echo "✓ All processes stopped"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 等待进程
wait
