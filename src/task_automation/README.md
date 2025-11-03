# 任务自动化包 (Task Automation Package)

自动化任务管理包，用于整合叶子检测和机械臂控制的完整自动化工作流。

## 📋 功能概述

此包提供自动化编排器，可以：
1. **调用叶子检测服务** - 检测图像中的叶子并获取坐标
2. **移动机械臂** - 根据检测到的叶子坐标自动移动机械臂到每个位置
3. **顺序处理** - 自动处理多个检测到的叶子，一个接一个

## 🏗️ 包结构

```
task_automation/
├── task_automation/
│   ├── __init__.py
│   └── automation_orchestrator.py  # 主要控制逻辑
├── launch/
│   └── automation_task.launch.py   # 启动文件
├── resource/
│   └── task_automation
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 🚀 使用方法

### 方式1: 使用自动化任务脚本（推荐）

**两步工作流程：**

1. **首先启动所有系统组件**：
```bash
cd /home/hao/Desktop/Hao_MTRN4231
./default_scripts/start_all.sh
```

2. **然后在另一个终端启动自动化任务**：
```bash
cd /home/hao/Desktop/Hao_MTRN4231
./default_scripts/run_automation.sh
```

脚本会自动检查系统状态，等待服务就绪后启动自动化任务。

### 方式2: 使用自定义参数

自动化任务脚本支持自定义参数：

```bash
./default_scripts/run_automation.sh --min-area 2000 --confidence 0.0 --offset-z 0.1
```

查看所有可用参数：
```bash
./default_scripts/run_automation.sh --help
```

#### 参数说明：

- `--min-area VALUE`: 叶子检测最小面积阈值（像素²），默认 `0.0`
- `--confidence VALUE`: 叶子检测置信度阈值，默认 `0.0`
- `--offset-z VALUE`: 机械臂Z轴偏移量（米），用于保持安全距离，默认 `0.05`
- `--home-x VALUE`: 原位X坐标（米），默认 `0.25`
- `--home-y VALUE`: 原位Y坐标（米），默认 `0.10`
- `--home-z VALUE`: 原位Z坐标（米），默认 `0.55`
- `--trash-x VALUE`: 垃圾桶X坐标（米），默认 `0.10`
- `--trash-y VALUE`: 垃圾桶Y坐标（米），默认 `0.50`
- `--trash-z VALUE`: 垃圾桶Z坐标（米），默认 `0.45`

### 方式3: 手动启动自动化任务

如果系统已经运行，可以直接使用ROS2命令启动：

```bash
cd /home/hao/Desktop/Hao_MTRN4231
source install/setup.bash
ros2 launch task_automation automation_task.launch.py
```

带参数：
```bash
ros2 launch task_automation automation_task.launch.py \
    min_area:=2000.0 \
    confidence:=0.0 \
    offset_z:=0.05 \
    home_x:=0.25 \
    home_y:=0.10 \
    home_z:=0.55
```

### 方式4: 直接运行可执行文件

```bash
source install/setup.bash
ros2 run task_automation automation_orchestrator --ros-args \
    -p min_area:=2000.0 \
    -p confidence:=0.0 \
    -p offset_z:=0.05 \
    -p home_x:=0.25 \
    -p home_y:=0.10 \
    -p home_z:=0.55
```

## 🔄 工作流程

自动化编排器执行以下步骤：

1. **等待服务就绪** - 等待叶子检测服务可用（超时30秒）
2. **检测叶子** - 调用叶子检测服务获取所有叶子的坐标
3. **记录结果** - 打印检测到的叶子数量和位置信息
4. **处理每个叶子** - 对于每个检测到的叶子：
   - 移动机械臂到叶子位置（添加Z轴偏移）
   - 拾取叶子后移动到垃圾桶倾倒
   - 等待2秒
   - 处理下一个叶子
5. **返回原位** - 所有叶子处理完成后，机械臂自动返回原位
6. **任务总结** - 显示成功处理的叶子数量

## 🗑️ 垃圾桶可视化

垃圾桶作为碰撞对象被添加到MoveIt场景中：
- **位置**: x=0.10m, y=0.50m, z=0.25m (中心高度)
- **尺寸**: 0.3m × 0.3m × 0.4m
- **在RViz中可见**: 启动系统后垃圾桶会以橙色碰撞框显示在场景中

## 🔍 依赖关系

此包依赖以下ROS2包和服务：

**ROS2 包:**
- `rclpy` - ROS2 Python客户端库
- `geometry_msgs` - 几何消息类型
- `arm_msgs` - 自定义消息和服务定义

**运行依赖:**
- `detect_leaf_pkg` - 提供 `leaf_detection_srv` 服务
- `arm_manipulation` - 提供 `move_arm_to_pose` 节点
- MoveIt配置和机器人驱动

## 📝 示例输出

```
================================================
🌿 叶子检测结果
================================================
状态: Detection successful
检测到叶子数量: 3
  叶子 1: X=0.250m, Y=0.100m, Z=0.550m
  叶子 2: X=0.300m, Y=-0.150m, Z=0.500m
  叶子 3: X=0.400m, Y=0.200m, Z=0.520m
================================================

处理第 1/3 个叶子...
移动机械臂到位置: x=0.250m, y=0.100m, z=0.600m
✓ 机械臂移动成功
✓ 叶子 1 处理完成
等待 3 秒后处理下一个叶子...

处理第 2/3 个叶子...
...

================================================
自动化任务完成
成功处理: 3/3 个叶子
================================================
```

## ⚠️ 注意事项

1. **系统必须先启动** - 在使用自动化任务之前，确保所有必需的系统组件都已运行：
   - 机器人驱动
   - MoveIt规划器和RViz
   - 碰撞对象
   - 相机TF描述
   - 相机节点
   - 叶子检测服务器

2. **Z轴偏移** - 默认添加5cm的Z轴偏移，确保机械臂不会撞到叶子。根据实际应用调整 `offset_z` 参数。

3. **等待时间** - 每个叶子之间等待3秒。如需修改，编辑 `automation_orchestrator.py` 中的 `wait_time` 变量。

4. **错误处理** - 如果某个叶子处理失败，任务会继续处理下一个叶子。

## 🔧 开发与调试

### 重新构建包

```bash
cd /home/hao/Desktop/Hao_MTRN4231
colcon build --packages-select task_automation --symlink-install
source install/setup.bash
```

### 查看可用节点

```bash
ros2 pkg executables task_automation
```

### 查看节点参数

```bash
ros2 param describe /automation_orchestrator
```

### 监控ROS2话题和服务

```bash
# 列出所有话题
ros2 topic list

# 列出所有服务
ros2 service list

# 查看叶子检测服务信息
ros2 service info /leaf_detection_srv
```

## 📚 相关文档

- [detect_leaf_pkg](../detect_leaf_pkg/USAGE_SERVER.md) - 叶子检测服务文档
- [arm_manipulation](../arm_manipulation/) - 机械臂控制包
- [start_all.sh](../../default_scripts/start_all.sh) - 完整系统启动脚本

## 📝 许可证

MIT License

## 👥 维护者

hao

