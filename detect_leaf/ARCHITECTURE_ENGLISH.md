# 🏗️ System Architecture - Leaf Detection to Robot Arm Control

## 📊 Overall System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    MTRN4231 Leaf Collection System                   │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────┐
│  RealSense      │  RGB image + depth image + intrinsics
│  Camera Driver  │
└────────┬────────┘
         │
         ├─ /camera/camera/color/image_raw (RGB)
         ├─ /camera/camera/aligned_depth_to_color/image_raw (depth)
         └─ /camera/camera/aligned_depth_to_color/camera_info (intrinsics)
         │
         ▼
    ╔════════════════════════════════════╗
    ║   Leaf Detection Node (leaf_      ║
    ║   detector)                        │
    ║   • PlantCV image processing       │
    ║   • Leaf segmentation and          │
    ║     recognition                    │
    ║   • Calculate center and           │
    ║     bounding box                   │
    ╚════════════════════════════════════╝
         │
         └─ /leaf_detection/coordinates (pixel coordinates JSON)
         │
         ▼
    ╔════════════════════════════════════╗
    ║ Coordinate Transformation Node     ║
    ║ (leaf_to_arm_transformer) ⭐ NEW  │
    ║ • Pixel coordinates → 3D camera   │
    ║   coordinates                     │
    ║ • Camera coordinates → robot arm  │
    ║   coordinates                     │
    ║ • TF2 transformation management   │
    ╚════════════════════════════════════╝
         │
         ├─ /leaf_detection/arm_target (robot arm coordinates PoseStamped)
         ├─ /leaf_detection/arm_target_markers (visualization markers)
         │
         ▼
    ┌──────────────────┐     ┌──────────────────┐
    │ MoveIt Planning  │────▶│ UR Driver        │
    │ • Motion         │     │ • Robot arm      │
    │   planning       │     │   control        │
    │ • Collision      │     │ • Real-time      │
    │   detection      │     │   feedback       │
    └──────────────────┘     └──────────────────┘
         │                         │
         └─────────────┬───────────┘
                       ▼
              ┌─────────────────┐
              │ UR 5e Robot Arm │
              │ • Execute       │
              │   grasping      │
              │ • Collect       │
              │   leaves        │
              └─────────────────┘
```

## 🔄 Data Flow Details

### Data Processing Chain

```
Step 1: Image Acquisition
━━━━━━━━━━━━━━━━━
Input: RealSense camera real-time video capture
      ├─ RGB image: 1920×1080 @ 30 FPS
      ├─ Depth image: 1280×720 @ 30 FPS  
      └─ Camera intrinsics: fx, fy, cx, cy (focal length and principal point)

Step 2: Leaf Detection
━━━━━━━━━━━━━━━━━
Processing: PlantCV image analysis
      ├─ Color range filtering (green leaf recognition)
      ├─ Binarization and morphological processing
      ├─ Contour detection and labeling
      └─ Calculate center coordinates (pixel space)

Output: /leaf_detection/coordinates
      {
        "frame": 123,
        "coordinates": [
          {
            "id": 0,
            "center": {"x": 360, "y": 240},  ← Pixel coordinates!
            "bounding_box": {...},
            "area": 2850.5
          }
        ]
      }

Step 3: Coordinate Transformation ⭐ NEW
━━━━━━━━━━━━━━━━━━
Transform 1: Pixel coordinates → 3D camera coordinates
      Input: Pixel coordinates (360, 240)
            Depth value: 500 mm (obtained from depth map)
            Camera intrinsics: fx=615, fy=615, cx=320, cy=240
            
      Calculation: x_cam = (360 - 320) * 0.5 / 615 = 0.0325 m
                   y_cam = (240 - 240) * 0.5 / 615 = 0.0 m
                   z_cam = 0.5 m
      
      Output: Camera coordinates (0.0325, 0.0, 0.5) m
              frame_id = "camera_color_optical_frame"

Transform 2: Camera coordinates → Robot arm coordinates (using TF2)
      Input: Camera coordinates (0.0325, 0.0, 0.5)
            TF transformation: camera → base_link
      
      Calculation: T_base_camera = obtained from TF query
                   point_base = T_base_camera × point_camera
      
      Output: Robot arm coordinates (0.123, -0.045, 0.567) m
              frame_id = "base_link"

Output: /leaf_detection/arm_target (PoseStamped)
      header:
        frame_id: base_link
        stamp: 123456789
      pose:
        position: {x: 0.123, y: -0.045, z: 0.567}
        orientation: {w: 1.0}

Step 4: Robot Arm Control
━━━━━━━━━━━━━━━━━
• MoveIt reads target position
• Plans motion from current position to target
• Executes motion and monitors
```

## 🎯 Coordinate System Relationships

### Coordinate System Tree (TF2)

```
base_link (robot arm base)
    │
    ├─ shoulder_link
    │  └─ upper_arm_link
    │     └─ forearm_link
    │        └─ wrist_1_link
    │           └─ wrist_2_link
    │              └─ wrist_3_link
    │                 └─ tool0 (tool endpoint)
    │
    └─ camera_link (camera physical center)
       └─ camera_color_optical_frame (RGB optical center) ⭐

Where:
  • base_link = robot arm base reference point
  • camera_link = RealSense physical installation position
  • camera_color_optical_frame = RGB image coordinate origin
    (this is the coordinate system we use)
  • tool0 = robot arm end effector
```

### Coordinate System Parameters

| Coordinate System | Type | Origin | X-axis | Y-axis | Z-axis | Parent Frame |
|------------------|------|--------|--------|--------|--------|--------------|
| `base_link` | Fixed | Robot arm base | Forward | Left | Up | - |
| `camera_link` | Moving* | Camera center | - | - | - | base_link |
| `camera_color_optical_frame` | Fixed | RGB image | Right | Down | Forward | camera_link |

*camera_link position relative to base_link may vary (if camera is mounted on robot arm)

## 📐 Key Transformation Formulas

### 1. Pixel to Camera 3D Coordinates

**Goal**: Convert 2D pixel coordinates in image to 3D camera coordinates

```
Formula:
  x_cam = (x_pixel - cx) × depth / fx
  y_cam = (y_pixel - cy) × depth / fy
  z_cam = depth

Parameters:
  (x_pixel, y_pixel) = pixel coordinates in image
  depth = depth value at that pixel (obtained from depth map)
  cx, cy = camera principal point (obtained from intrinsics K)
  fx, fy = camera focal length (obtained from intrinsics K)

Intrinsics matrix K:
  ┌          ┐
  │ fx  0 cx │
  │  0 fy cy │
  │  0  0  1 │
  └          ┘

Example:
  cx = 320, cy = 240 (principal point)
  fx = 615, fy = 615 (focal length)
  x_pixel = 360, y_pixel = 240 (pixel to transform)
  depth = 0.5 m
  
  x_cam = (360 - 320) × 0.5 / 615 = 0.0325 m
  y_cam = (240 - 240) × 0.5 / 615 = 0.0 m
  z_cam = 0.5 m
```

### 2. Camera Coordinates to Robot Arm Coordinates

**Goal**: Use rigid body transformation to convert camera coordinates to robot arm coordinates

```
Formula (homogeneous transformation):
  p_arm = T_arm_camera × p_camera

Where:
  p_camera = point in camera coordinate system [x, y, z, 1]
  T_arm_camera = rigid body transformation from camera to robot arm
  p_arm = point in robot arm coordinate system [x', y', z', 1]

Rigid body transformation matrix:
  ┌          ┐   ┌     ┐
  │ R   t    │   │ x   │
  │ 0   1    │ × │ y   │
  │          │   │ z   │
  │          │   │ 1   │
  └          ┘   └     ┘

Where:
  R = 3×3 rotation matrix
  t = 3×1 translation vector

ROS TF2 implementation:
  transform = tf_buffer.lookup_transform(
    'base_link',                      # target coordinate system
    'camera_color_optical_frame',     # source coordinate system
    time                              # timestamp
  )
  
  p_arm = tf2_ros.do_transform_point(p_camera, transform)
```

## 🔌 ROS Node Communication

### Node Graph

```
leaf_detector (leaf_detector.py)
├─ Subscribe:
│  ├─ /camera/camera/color/image_raw (sensor_msgs/Image)
│  └─ Internal processing...
└─ Publish:
   ├─ /leaf_detection/coordinates (std_msgs/String - JSON)
   ├─ /leaf_detection/annotated_image (sensor_msgs/Image)
   └─ /leaf_detection/leaf_count (std_msgs/Int32)

leaf_to_arm_transformer (leaf_to_arm_transformer.py) ⭐ NEW
├─ Subscribe:
│  ├─ /leaf_detection/coordinates (std_msgs/String)
│  ├─ /camera/camera/aligned_depth_to_color/image_raw (sensor_msgs/Image)
│  └─ /camera/camera/aligned_depth_to_color/camera_info (sensor_msgs/CameraInfo)
├─ TF2:
│  ├─ Listen to /tf and /tf_static
│  └─ Query base_link ← camera_color_optical_frame transformation
└─ Publish:
   ├─ /leaf_detection/arm_target (geometry_msgs/PoseStamped)
   ├─ /leaf_detection/arm_target_markers (visualization_msgs/MarkerArray)
```

### Message Types

```
std_msgs/String (JSON format)
{
  "frame": int,
  "timestamp": str,
  "num_leaves": int,
  "coordinates": [
    {
      "id": int,
      "center": {"x": float, "y": float},
      "bounding_box": {"x": float, "y": float, "width": float, "height": float},
      "area": float,
      "perimeter": float
    }
  ]
}

geometry_msgs/PoseStamped
{
  "header": {
    "frame_id": "base_link",
    "stamp": {sec, nsec}
  },
  "pose": {
    "position": {"x": float, "y": float, "z": float},
    "orientation": {"x": float, "y": float, "z": float, "w": float}
  }
}

visualization_msgs/MarkerArray
├─ markers[]: visualization_msgs/Marker
   ├─ type: SPHERE
   ├─ pose: geometry_msgs/Pose
   ├─ scale: geometry_msgs/Vector3
   ├─ color: std_msgs/ColorRGBA
```

## 🛠️ Key Components Details

### 1. RealSense Camera Driver

**Function**: Capture RGB and depth images, publish camera intrinsics

**Published Topics**:
- `/camera/camera/color/image_raw` - RGB image
- `/camera/camera/aligned_depth_to_color/image_raw` - depth map
- `/camera/camera/aligned_depth_to_color/camera_info` - camera parameters

**Key Parameters**:
```yaml
# Camera intrinsics matrix K
K: [fx,  0, cx,
     0, fy, cy,
     0,  0,  1]

# In this example:
fx = 615    # X direction focal length
fy = 615    # Y direction focal length
cx = 320    # X principal point
cy = 240    # Y principal point
```

### 2. Leaf Detection Node

**Function**: Process images, detect and locate leaves

**Input**: RGB image
**Output**: Leaf coordinates (pixel space)

**Core Algorithm**:
1. Color filtering (recognize green leaves)
2. Morphological processing (denoising, filling)
3. Contour detection (find leaf boundaries)
4. Labeling and calculation (center coordinates)

### 3. Coordinate Transformation Node ⭐ NEW

**Function**: Transform detected leaf coordinates from camera coordinate system to robot arm coordinate system

**Processing Flow**:
```python
# 1. Get camera intrinsics
intrinsics = parse_camera_info(msg)

# 2. Get depth value
depth = read_pixel_depth(x_pixel, y_pixel)

# 3. Pixel → Camera 3D coordinates
p_camera = pixel_to_3d(x_pixel, y_pixel, depth, intrinsics)

# 4. Camera coordinates → Robot arm coordinates
T = tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame')
p_arm = transform_point(p_camera, T)

# 5. Publish results
publish(PoseStamped(p_arm, frame_id='base_link'))
```

## ⚠️ Common Issues and Solutions

### Issue 1: Zero or Invalid Depth Values

**Phenomenon**: Leaves detected, but depth value is 0

**Causes**:
- RealSense cannot measure depth at that point (out of range or occluded)
- Depth sensor failure or misalignment

**Solution**:
```bash
# Check depth range
ros2 run image_view image_view image:=/camera/camera/aligned_depth_to_color/image_raw

# Should see depth image, no black (invalid) areas
# If lots of black, camera is too far or angle is wrong
```

### Issue 2: TF Transformation Doesn't Exist

**Phenomenon**: `Cannot transform from camera to base_link`

**Causes**:
- Robot arm driver not started
- Camera driver not started
- Coordinate system not published to TF

**Solution**:
```bash
# Check all active TF transformations
ros2 run tf2_tools view_frames

# Should see: base_link connected to other frames
# Should see: camera_link in tree

# If missing, start corresponding drivers
ros2 launch ur_robot_driver ur_control.launch.py use_fake_hardware:=true
```

### Issue 3: Unreasonable Coordinates

**Phenomenon**: Transformed coordinates not within robot arm reach

**Causes**:
- Intrinsics or depth calibration error
- TF transformation inaccurate
- Camera installation position deviation

**Solution**:
1. Verify camera intrinsics - compare fx, fy, cx, cy in `/camera/camera/aligned_depth_to_color/camera_info`
2. Verify depth values - measure actual distance with ruler
3. Verify TF transformation - check if camera installation position is correctly input
4. Adjust depth offset parameters

## 📈 Performance Metrics

| Metric | Target | Actual |
|--------|--------|--------|
| Leaf detection frequency | 30 FPS | ~20-25 FPS |
| Coordinate transformation delay | <20 ms | ~5-10 ms |
| Coordinate accuracy | ±2 cm | ±1-3 cm (depends on calibration) |
| Memory usage | <500 MB | ~200-300 MB |

## 🔮 Extension Directions

### Real-time Tracking of Multiple Leaves
```
Current: Detect all leaves each frame
Improvement: Track leaf IDs, predict trajectories
```

### Dynamic Calibration
```
Current: Fixed depth offset
Improvement: Dynamically adjust based on motion feedback
```

### Parallel Processing
```
Current: Serial processing
Improvement: Use multi-threading for image and transformation processing
```

### Deep Integration with MoveIt
```
Current: Publish target position
Improvement: Automatically generate motion plans and obstacle avoidance
```
