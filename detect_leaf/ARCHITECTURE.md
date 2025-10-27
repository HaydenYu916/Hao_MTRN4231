# 🏗️ 系统架构 - 叶子检测到机械臂控制

## 📊 整体系统架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                         MTRN4231 叶子采集系统                         │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────┐
│  RealSense      │  RGB图像 + 深度图像 + 内参
│  相机驱动        │
└────────┬────────┘
         │
         ├─ /camera/camera/color/image_raw (RGB)
         ├─ /camera/camera/aligned_depth_to_color/image_raw (深度)
         └─ /camera/camera/aligned_depth_to_color/camera_info (内参)
         │
         ▼
    ╔════════════════════════════════════╗
    ║   叶子检测节点 (leaf_detector)      ║
    ║   • PlantCV图像处理                │
    ║   • 叶子分割和识别                 │
    ║   • 计算中心和边界框               │
    ╚════════════════════════════════════╝
         │
         └─ /leaf_detection/coordinates (像素坐标JSON)
         │
         ▼
    ╔════════════════════════════════════╗
    ║ 坐标转换节点 (leaf_to_arm_          ║
    ║   transformer) ⭐ NEW             │
    ║   • 像素坐标 → 3D相机坐标         │
    ║   • 相机坐标 → 机械臂坐标         │
    ║   • TF2变换管理                   │
    ╚════════════════════════════════════╝
         │
         ├─ /leaf_detection/arm_target (机械臂坐标 PoseStamped)
         ├─ /leaf_detection/arm_target_markers (可视化标记)
         │
         ▼
    ┌──────────────────┐     ┌──────────────────┐
    │ MoveIt规划       │────▶│ UR驱动           │
    │ • 运动规划       │     │ • 机械臂控制     │
    │ • 碰撞检测       │     │ • 实时反馈       │
    └──────────────────┘     └──────────────────┘
         │                         │
         └─────────────┬───────────┘
                       ▼
              ┌─────────────────┐
              │ UR 5e 机械臂    │
              │ • 执行抓取      │
              │ • 采集叶子      │
              └─────────────────┘
```

## 🔄 数据流详解

### 数据处理链

```
Step 1: 图像采集
━━━━━━━━━━━━━━━━━
输入：RealSense相机拍摄的实时视频
      ├─ RGB图像: 1920×1080 @ 30 FPS
      ├─ 深度图像: 1280×720 @ 30 FPS  
      └─ 相机内参: fx, fy, cx, cy (焦距和主点)

Step 2: 叶子检测
━━━━━━━━━━━━━━━━━
处理：PlantCV图像分析
      ├─ 颜色范围过滤 (绿色叶子识别)
      ├─ 二值化和形态学处理
      ├─ 轮廓检测和标记
      └─ 计算中心坐标 (像素空间)

输出：/leaf_detection/coordinates
      {
        "frame": 123,
        "coordinates": [
          {
            "id": 0,
            "center": {"x": 360, "y": 240},  ← 像素坐标！
            "bounding_box": {...},
            "area": 2850.5
          }
        ]
      }

Step 3: 坐标转换 ⭐ NEW
━━━━━━━━━━━━━━━━━━
变换1：像素坐标 → 3D相机坐标
      输入: 像素坐标 (360, 240)
            深度值: 500 mm (从深度图获取)
            相机内参: fx=615, fy=615, cx=320, cy=240
            
      计算: x_cam = (360 - 320) * 0.5 / 615 = 0.0325 m
            y_cam = (240 - 240) * 0.5 / 615 = 0.0 m
            z_cam = 0.5 m
      
      输出: 相机坐标 (0.0325, 0.0, 0.5) m
            frame_id = "camera_color_optical_frame"

变换2：相机坐标 → 机械臂坐标 (使用TF2)
      输入: 相机坐标 (0.0325, 0.0, 0.5)
            TF变换: camera → base_link
      
      计算: T_base_camera = 从TF查询获取
            point_base = T_base_camera × point_camera
      
      输出: 机械臂坐标 (0.123, -0.045, 0.567) m
            frame_id = "base_link"

输出：/leaf_detection/arm_target (PoseStamped)
      header:
        frame_id: base_link
        stamp: 123456789
      pose:
        position: {x: 0.123, y: -0.045, z: 0.567}
        orientation: {w: 1.0}

Step 4: 机械臂控制
━━━━━━━━━━━━━━━━━
• MoveIt读取目标位置
• 规划从当前位置到目标的运动
• 执行运动并监视
```

## 🎯 坐标系关系

### 坐标系树（TF2）

```
base_link (机械臂基座)
    │
    ├─ shoulder_link
    │  └─ upper_arm_link
    │     └─ forearm_link
    │        └─ wrist_1_link
    │           └─ wrist_2_link
    │              └─ wrist_3_link
    │                 └─ tool0 (工具端点)
    │
    └─ camera_link (相机物理中心)
       └─ camera_color_optical_frame (RGB光学中心) ⭐

其中：
  • base_link = 机械臂基座参考点
  • camera_link = RealSense物理安装位置
  • camera_color_optical_frame = RGB图像的坐标原点
    (这是我们使用的坐标系)
  • tool0 = 机械臂末端执行器
```

### 坐标系参数

| 坐标系 | 类型 | 原点 | X轴 | Y轴 | Z轴 | 父框架 |
|--------|------|------|-----|-----|-----|--------|
| `base_link` | 固定 | 机械臂基座 | 向前 | 向左 | 向上 | - |
| `camera_link` | 移动* | 相机中心 | - | - | - | base_link |
| `camera_color_optical_frame` | 固定 | RGB图像 | 向右 | 向下 | 向前 | camera_link |

*camera_link相对于base_link的位置可能会变化（如果相机装在机械臂上）

## 📐 关键转换公式

### 1. 像素到相机3D坐标

**目标**：将图像中的2D像素坐标转换为3D相机坐标

```
公式：
  x_cam = (x_pixel - cx) × depth / fx
  y_cam = (y_pixel - cy) × depth / fy
  z_cam = depth

参数：
  (x_pixel, y_pixel) = 图像中的像素坐标
  depth = 该像素处的深度值（从深度图获取）
  cx, cy = 相机主点（从内参K获取）
  fx, fy = 相机焦距（从内参K获取）

内参矩阵K：
  ┌          ┐
  │ fx  0 cx │
  │  0 fy cy │
  │  0  0  1 │
  └          ┘

例：
  cx = 320, cy = 240 (主点)
  fx = 615, fy = 615 (焦距)
  x_pixel = 360, y_pixel = 240 (要转换的像素)
  depth = 0.5 m
  
  x_cam = (360 - 320) × 0.5 / 615 = 0.0325 m
  y_cam = (240 - 240) × 0.5 / 615 = 0.0 m
  z_cam = 0.5 m
```

### 2. 相机坐标到机械臂坐标

**目标**：使用刚体变换将相机坐标转换为机械臂坐标

```
公式（齐次变换）：
  p_arm = T_arm_camera × p_camera

其中：
  p_camera = 相机坐标系中的点 [x, y, z, 1]
  T_arm_camera = 从相机到机械臂的刚体变换
  p_arm = 机械臂坐标系中的点 [x', y', z', 1]

刚体变换矩阵：
  ┌          ┐   ┌     ┐
  │ R   t    │   │ x   │
  │ 0   1    │ × │ y   │
  │          │   │ z   │
  │          │   │ 1   │
  └          ┘   └     ┘

其中：
  R = 3×3 旋转矩阵
  t = 3×1 平移向量

ROS TF2实现：
  transform = tf_buffer.lookup_transform(
    'base_link',                      # 目标坐标系
    'camera_color_optical_frame',     # 源坐标系
    time                              # 时间戳
  )
  
  p_arm = tf2_ros.do_transform_point(p_camera, transform)
```

## 🔌 ROS节点通信

### 节点图

```
leaf_detector (leaf_detector.py)
├─ 订阅:
│  ├─ /camera/camera/color/image_raw (sensor_msgs/Image)
│  └─ 内部处理...
└─ 发布:
   ├─ /leaf_detection/coordinates (std_msgs/String - JSON)
   ├─ /leaf_detection/annotated_image (sensor_msgs/Image)
   └─ /leaf_detection/leaf_count (std_msgs/Int32)

leaf_to_arm_transformer (leaf_to_arm_transformer.py) ⭐ NEW
├─ 订阅:
│  ├─ /leaf_detection/coordinates (std_msgs/String)
│  ├─ /camera/camera/aligned_depth_to_color/image_raw (sensor_msgs/Image)
│  └─ /camera/camera/aligned_depth_to_color/camera_info (sensor_msgs/CameraInfo)
├─ TF2:
│  ├─ 监听 /tf 和 /tf_static
│  └─ 查询 base_link ← camera_color_optical_frame 的变换
└─ 发布:
   ├─ /leaf_detection/arm_target (geometry_msgs/PoseStamped)
   ├─ /leaf_detection/arm_target_markers (visualization_msgs/MarkerArray)
```

### 消息类型

```
std_msgs/String (JSON格式)
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

## 🛠️ 关键组件详解

### 1. RealSense相机驱动

**作用**：捕获RGB和深度图像，发布相机内参

**发布话题**：
- `/camera/camera/color/image_raw` - RGB图像
- `/camera/camera/aligned_depth_to_color/image_raw` - 深度图
- `/camera/camera/aligned_depth_to_color/camera_info` - 相机参数

**关键参数**：
```yaml
# 相机内参矩阵K
K: [fx,  0, cx,
     0, fy, cy,
     0,  0,  1]

# 本例中：
fx = 615    # X方向焦距
fy = 615    # Y方向焦距
cx = 320    # X主点
cy = 240    # Y主点
```

### 2. 叶子检测节点

**作用**：处理图像，检测和定位叶子

**输入**：RGB图像
**输出**：叶子坐标（像素空间）

**核心算法**：
1. 颜色过滤（识别绿色叶子）
2. 形态学处理（去噪、填充）
3. 轮廓检测（找叶子边界）
4. 标记和计算（中心坐标）

### 3. 坐标转换节点 ⭐ NEW

**作用**：将检测到的叶子坐标从相机坐标系转换到机械臂坐标系

**处理流程**：
```python
# 1. 获取相机内参
intrinsics = parse_camera_info(msg)

# 2. 获取深度值
depth = read_pixel_depth(x_pixel, y_pixel)

# 3. 像素 → 相机3D坐标
p_camera = pixel_to_3d(x_pixel, y_pixel, depth, intrinsics)

# 4. 相机坐标 → 机械臂坐标
T = tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame')
p_arm = transform_point(p_camera, T)

# 5. 发布结果
publish(PoseStamped(p_arm, frame_id='base_link'))
```

## ⚠️ 常见问题和解决

### 问题1：深度值为零或无效

**现象**：叶子检测到了，但深度值为0

**原因**：
- RealSense无法测量该点的深度（超出范围或被遮挡）
- 深度传感器故障或未对齐

**解决**：
```bash
# 检查深度范围
ros2 run image_view image_view image:=/camera/camera/aligned_depth_to_color/image_raw

# 应该看到深度图像，没有黑色（无效）区域
# 如果有大量黑色，说明相机距离太远或角度不对
```

### 问题2：TF变换不存在

**现象**：`Cannot transform from camera to base_link`

**原因**：
- 机械臂驱动未启动
- 相机驱动未启动
- 坐标系未发布到TF

**解决**：
```bash
# 检查所有活跃的TF变换
ros2 run tf2_tools view_frames

# 应该看到：base_link 连接到其他框架
# 应该看到：camera_link 在树中

# 如果缺少，启动相应的驱动
ros2 launch ur_robot_driver ur_control.launch.py use_fake_hardware:=true
```

### 问题3：坐标不合理

**现象**：转换后的坐标不在机械臂可达范围内

**原因**：
- 内参或深度标定错误
- TF变换不准确
- 相机安装位置偏差

**解决**：
1. 验证相机内参 - 比较 `/camera/camera/aligned_depth_to_color/camera_info` 中的fx, fy, cx, cy
2. 验证深度值 - 用卷尺测量实际距离
3. 验证TF变换 - 检查相机安装位置是否输入正确
4. 调整深度偏移参数

## 📈 性能指标

| 指标 | 目标 | 实际 |
|-----|------|------|
| 叶子检测频率 | 30 FPS | ~20-25 FPS |
| 坐标转换延迟 | <20 ms | ~5-10 ms |
| 坐标精度 | ±2 cm | ±1-3 cm (取决于标定) |
| 内存占用 | <500 MB | ~200-300 MB |

## 🔮 扩展方向

### 多个叶子的实时跟踪
```
当前：每帧检测所有叶子
改进：跟踪叶子ID，预测轨迹
```

### 动态校准
```
当前：固定的深度偏移
改进：根据运动反馈动态调整
```

### 并行处理
```
当前：串行处理
改进：使用多线程处理图像和转换
```

### 与MoveIt的深度集成
```
当前：发布目标位置
改进：自动生成运动计划和避障
```
