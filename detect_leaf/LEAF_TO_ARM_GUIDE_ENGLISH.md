# 🌿 Leaf Detection to Robot Arm Coordinate System Transformation Guide

## 📋 Overview

This guide explains how to transform leaf coordinates detected by RealSense camera to UR 5e robot arm coordinate system, enabling the robot arm to grasp or manipulate detected leaves.

## 🏗️ Overall Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        Complete System Flow                       │
└──────────────────────────────────────────────────────────────────┘

1️⃣ Data Acquisition Layer
   ├─ RealSense camera → RGB image + depth image
   └─ Camera intrinsics (CameraInfo)

2️⃣ Leaf Detection Layer
   ├─ PlantCV image processing
   ├─ Leaf recognition → pixel coordinates (x_pixel, y_pixel)
   └─ Publish: /leaf_detection/coordinates (JSON format)

3️⃣ Coordinate Transformation Layer ⭐ (Focus of this guide)
   ├─ Pixel coordinates → 3D camera coordinates
   │  ├─ Get pixel depth value
   │  └─ Use RealSense intrinsics for back-projection
   │
   ├─ Camera coordinates → robot arm base coordinates
   │  ├─ Use TF2 framework
   │  └─ Requires camera-robot arm transformation relationship
   │
   └─ Publish: /leaf_detection/arm_target (PoseStamped)

4️⃣ Execution Layer
   ├─ MoveIt motion planning
   ├─ Robot arm motion control
   └─ Execute grasping/manipulation actions
```

## 🔄 Coordinate Transformation Flow Details

### Step 1: Pixel Coordinates → Camera Coordinates (3D Projection)

```python
# Input: Leaf center pixel coordinates detected in image
pixel_x = 360  # pixel x coordinate
pixel_y = 240  # pixel y coordinate

# Get depth value at this pixel from depth map
depth = depth_image[pixel_y, pixel_x]  # unit: mm

# Use RealSense intrinsics for back-projection
intrinsics = camera_info_from_ros
x, y, z = rs.rs2_deproject_pixel_to_point(
    intrinsics,
    (pixel_x, pixel_y),
    depth_in_meters
)

# Output: 3D point in camera coordinate system
# (x, y, z) - unit: meters
```

**RealSense Back-projection Formula:**
```
x_camera = (pixel_x - cx) * depth / fx
y_camera = (pixel_y - cy) * depth / fy
z_camera = depth

Where:
  cx, cy: camera principal point (obtained from intrinsics)
  fx, fy: camera focal length (obtained from intrinsics)
  depth: depth value at that pixel
```

### Step 2: Camera Coordinates → Robot Arm Coordinates (TF2 Transformation)

```python
# Use TF2 to get transformation relationship
transform = tf_buffer.lookup_transform(
    target_frame='base_link',        # target: robot arm base coordinate system
    source_frame='camera_color_optical_frame',  # source: camera coordinate system
    time=now
)

# Apply transformation
point_arm = tf2_ros.do_transform_point(point_camera, transform)

# Result: point in robot arm coordinate system
arm_x = point_arm.x
arm_y = point_arm.y
arm_z = point_arm.z
```

## 🚀 Quick Start

### Prerequisites

```bash
# 1. System must run the following components
source /opt/ros/humble/setup.bash
source /home/hao/Desktop/4231SuppliedCode/detect_leaf/install/setup.bash

# 2. Ensure necessary packages are installed
pip3 install pyrealsense2 cv-bridge
```

### Start Complete System

#### Method 1: Use Startup Script (Recommended)

```bash
cd /home/hao/Desktop/4231SuppliedCode/detect_leaf

# Start complete system (camera + detection + transformation + visualization)
./start_full_system.sh

# This script automatically starts:
# ✓ RealSense camera driver
# ✓ Leaf detection node (leaf_detector)
# ✓ Coordinate transformation node (leaf_to_arm_transformer) - NEW
# ✓ RViz2 visualization
```

#### Method 2: Manually Start Each Component

**Terminal 1 - Start Camera:**
```bash
cd /home/hao/Desktop/4231SuppliedCode
./4231_scripts/camera.sh
```

**Terminal 2 - Start Leaf Detection:**
```bash
cd /home/hao/Desktop/4231SuppliedCode/detect_leaf
source install/setup.bash
ros2 run detect_leaf_pkg leaf_detector
```

**Terminal 3 - Start Coordinate Transformation Node:**
```bash
cd /home/hao/Desktop/4231SuppliedCode/detect_leaf
source install/setup.bash
ros2 run detect_leaf_pkg leaf_to_arm_transformer
```

**Terminal 4 - Start Robot Arm Driver (choose one):**

Fake Robot Arm (Simulation):
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=yyy.yyy.yyy.yyy \
  initial_joint_controller:=joint_trajectory_controller \
  use_fake_hardware:=true \
  launch_rviz:=false
```

Real Robot Arm (need to modify IP):
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=actual_robot_arm_IP_address \
  initial_joint_controller:=joint_trajectory_controller
```

**Terminal 5 - Start MoveIt:**
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true \
  use_fake_hardware:=true
```

**Terminal 6 - Start RViz Visualization (if not started):**
```bash
rviz2 -d /home/hao/Desktop/4231SuppliedCode/detect_leaf/annotated_image.rviz
```

## 📡 ROS Topic Monitoring

### Monitor Leaf Detection Coordinates

```bash
# View leaf detection coordinates (camera coordinate system)
ros2 topic echo /leaf_detection/coordinates
```

Output example:
```json
{
  "frame": 123,
  "timestamp": "2025-10-21 16:44:00",
  "num_leaves": 2,
  "coordinates": [
    {
      "id": 0,
      "center": {"x": 360, "y": 240},
      "bounding_box": {"x": 330, "y": 210, "width": 60, "height": 60},
      "area": 2850.5,
      "perimeter": 189.3
    }
  ]
}
```

### Monitor Robot Arm Coordinates

```bash
# View transformed robot arm coordinates
ros2 topic echo /leaf_detection/arm_target
```

Output example:
```
header:
  stamp: {sec: 1698000000, nsec: 123456789}
  frame_id: base_link
pose:
  position:
    x: 0.123
    y: -0.045
    z: 0.567
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

### Monitor TF2 Transformations

```bash
# View all available TF frames
ros2 run tf2_tools view_frames

# Check specific transformation
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

## 🔧 Configuration Parameters

Edit `/home/hao/Desktop/4231SuppliedCode/detect_leaf/src/detect_leaf_pkg/detect_leaf_pkg/leaf_to_arm_transformer.py`

### Main Parameters:

```python
# 1. Camera coordinate system name (according to actual setup)
camera_frame = 'camera_color_optical_frame'  # RealSense optical coordinate system
# or 'camera_link' (if using other camera)

# 2. Robot arm base coordinate system name
arm_base_frame = 'base_link'  # UR 5e base coordinate system

# 3. Depth offset (offset from camera to robot arm gripper)
# This value depends on how camera is mounted on robot arm
depth_offset = 0.038  # unit: meters (3.8cm)

# 4. TF transformation query timeout
timeout = 0.5  # seconds
```

## 🎯 Common Coordinate System Descriptions

| Coordinate System | Description | Parent Frame | Purpose |
|------------------|-------------|--------------|---------|
| `base_link` | Robot arm base | Fixed | Robot arm reference point |
| `camera_color_optical_frame` | RealSense optical center | camera_link | Image processing |
| `camera_link` | RealSense physical center | base_link | Physical installation point |
| `tool0` | Tool endpoint | shoulder_link | End effector |

## 📊 RViz Visualization Configuration

Recommended RViz displays to add:

1. **TF** - View coordinate system relationships
   - Fixed frame: `base_link`
   - Show all transformations

2. **Marker Array** - Show leaf targets
   - Topic: `/leaf_detection/arm_target_markers`
   - Green spheres represent detected leaf positions

3. **Image** - Camera image
   - Topic: `/camera/camera/color/image_raw`

4. **Image** - Detection results
   - Topic: `/leaf_detection/annotated_image`

## 🐛 Troubleshooting

### Issue 1: TF Transformation Failed
```
⚠️ Coordinate transformation failed: Cannot transform frame
```

**Solution:**
```bash
# Check TF frame tree
ros2 run tf2_tools view_frames

# Ensure these two coordinate systems are connected:
# base_link → ... → camera_color_optical_frame

# If transformation is missing, need to start robot arm driver and sensor driver
ros2 launch ur_robot_driver ur_control.launch.py use_fake_hardware:=true
```

### Issue 2: Cannot Get Depth Value
```
⚠️ Leaf #0 cannot get depth information
```

**Solution:**
```bash
# Check depth topic
ros2 topic list | grep depth

# Check if depth image is being sent
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw

# If no depth data, restart camera driver
pkill -f realsense2_camera
./4231_scripts/camera.sh
```

### Issue 3: Camera Intrinsics Not Obtained
```
⚠️ Camera intrinsics not available
```

**Solution:**
```bash
# Check camera info topic
ros2 topic echo /camera/camera/aligned_depth_to_color/camera_info

# Wait for node startup to complete (usually takes 5-10 seconds)
sleep 10
```

## 🎓 Advanced Configuration

### Custom Coordinate Transformation

If robot arm and camera transformation is not automatically set, you can manually define it:

```python
# Add static transformation in leaf_to_arm_transformer.py
def __init__(self):
    # ... existing code ...
    
    # If TF is not automatically published, manually publish
    if not self.tf_buffer.can_transform('base_link', 'camera_link', now):
        self.publish_static_transform()

def publish_static_transform(self):
    """Publish static transformation from camera to robot arm base"""
    from geometry_msgs.msg import TransformStamped
    
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'camera_link'
    
    # Adjust based on actual installation position
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.15  # Camera height from base
    
    # Rotation (quaternion)
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0
    
    self.tf_broadcaster.sendTransform(transform)
```

### Integration with MoveIt

```python
# Add additional publisher in leaf_to_arm_transformer.py
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

def add_leaf_as_collision_object(self, arm_pose, leaf_id):
    """Add detected leaf as collision object to scene"""
    collision_object = CollisionObject()
    collision_object.id = f"leaf_{leaf_id}"
    collision_object.header.frame_id = self.arm_base_frame
    
    # Define as sphere
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [0.02]  # diameter 2cm
    
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(arm_pose.pose)
    collision_object.operation = CollisionObject.ADD
    
    self.collision_object_publisher.publish(collision_object)
```

## 📝 Complete Workflow Example

```bash
# 1. Start all systems
cd /home/hao/Desktop/4231SuppliedCode/detect_leaf
./start_full_system.sh

# 2. In new terminal, monitor leaf detection
ros2 topic echo /leaf_detection/coordinates | grep -A 10 "center"

# 3. Monitor robot arm targets
ros2 topic echo /leaf_detection/arm_target

# 4. Use MoveIt interactive markers to verify
# - Open interactive markers in RViz
# - Verify transformed position is correct

# 5. Execute planning and motion (use MoveIt API or other controllers)
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

# 6. Execute grasping through MoveIt (refer to lab3_moveit examples)
```

## 📚 Reference Resources

- **RealSense SDK**: https://github.com/IntelRealSense/librealsense
- **TF2 Documentation**: https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Tf2.html
- **PlantCV**: https://plantcv.danforthcenter.org/
- **MoveIt**: https://moveit.ros.org/

## ✅ Checklist

Before startup confirmation:

- [ ] RealSense camera is connected
- [ ] ROS2 Humble is installed
- [ ] detect_leaf workspace is compiled
- [ ] UR driver is installed
- [ ] TF transformations are correctly published
- [ ] Camera intrinsics are obtained

After startup monitoring:

- [ ] `/camera/camera/color/image_raw` has data
- [ ] `/camera/camera/aligned_depth_to_color/image_raw` has data
- [ ] `/leaf_detection/coordinates` has leaf detection data
- [ ] `/leaf_detection/arm_target` has transformed coordinates
- [ ] RViz shows correct coordinate systems and target positions

## 🎯 Next Steps

After checking that transformation is correct, you can:

1. **Use MoveIt Planning**: Use MoveIt to plan motion from current position to leaf position
2. **Implement Grasping**: Integrate grasp success detection (force sensor/visual feedback)
3. **Integrate Control**: Connect complete leaf collection workflow
4. **Optimize Performance**: Tune parameters to improve accuracy and speed

See example code in `lab3_moveit` and `4231_demo_packages`.
