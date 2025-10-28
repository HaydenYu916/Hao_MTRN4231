# Camera Extrinsic Parameters Configuration

## ✅ Configured Parameters

Camera extrinsic parameters extracted from URDF xacro file:

```yaml
camera_rpy: [3.12825, -3.12574, -3.13495]  # roll, pitch, yaw (radians)
camera_position: [0.0199931, -0.0028582, -0.109958]  # x, y, z (meters)
```

## ⚠️ Important Notes

### Coordinate System Source
These parameters come from xacro file definition:
```xml
<joint name="camera_joint" type="fixed">
    <parent link="flange"/>
    <child link="camera_link"/>
    <origin xyz="0.0199931 -0.0028582 -0.109958" 
            rpy="3.12825 -3.12574 -3.13495" />
</joint>
```

### Coordinate System Description
- **Parent frame**: `flange` (robot end-effector flange)
- **Child frame**: `camera_link` (camera link)

### 📍 Usage Scenarios

#### Case 1: Camera mounted on robot end-effector (Eye-in-Hand)
- Camera moves with robot arm
- TF chain: `base_link` → `flange` → `camera_link` → `camera_color_optical_frame`
- Base coordinates need full TF chain

#### Case 2: Camera fixed on base (Eye-to-Hand)
- Camera position fixed
- Need to measure camera transform relative to `base_link`
- Current parameters not applicable, need recalibration

## 🚀 Next Steps

1. **Confirm your mounting method**:
   - If camera on end-effector → current config usable
   - If camera fixed on base → need to measure transform from `base_link` to camera

2. **Start system for testing**:
   ```bash
   cd detect_leaf_v1
   ./start_full_system.sh
   ```

3. **View TF tree**:
   ```bash
   ros2 run tf2_tools view_frames
   # Generates frames.pdf to verify camera position
   ```

## 📝 Modify Parameters

If actual parameters differ, edit `camera_extrinsics.yaml`:

```yaml
camera_rpy:
  - your_roll_value
  - your_pitch_value
  - your_yaw_value

camera_position:
  - x_value(meters)
  - y_value(meters)
  - z_value(meters)
```


