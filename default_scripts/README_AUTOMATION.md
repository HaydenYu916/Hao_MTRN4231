# 自动化任务快速指南

自动化叶子检测和机械臂控制系统的使用说明。

## 📋 快速开始

### 步骤1: 启动系统

在一个终端中运行：

```bash
cd /home/hao/Desktop/Hao_MTRN4231
./default_scripts/start_all.sh
```

这会启动所有必需的系统组件：
- 机器人驱动
- MoveIt规划器和RViz
- 碰撞对象
- 相机TF描述
- 相机节点
- 叶子检测服务器

### 步骤2: 启动自动化任务

等待系统完全启动后（通常10-15秒），在另一个终端运行：

```bash
cd /home/hao/Desktop/Hao_MTRN4231
./default_scripts/run_automation.sh
```

自动化任务会：
1. 检测图像中的叶子
2. 获取每个叶子的坐标
3. 移动机械臂到每个叶子位置

## ⚙️ 自定义参数

```bash
# 设置最小检测面积
./default_scripts/run_automation.sh --min-area 2000

# 设置Z轴安全偏移
./default_scripts/run_automation.sh --offset-z 0.1

# 组合使用多个参数
./default_scripts/run_automation.sh --min-area 2000 --confidence 0.8 --offset-z 0.1

# 查看帮助
./default_scripts/run_automation.sh --help
```

## 📊 工作流程

```
启动系统 → 等待所有节点就绪 → 检测叶子 → 移动机械臂 → 处理下一个叶子 → 完成
```

## 🔍 故障排除

### 错误: "叶子检测服务不可用"

**解决方法**：
- 确保已运行 `./default_scripts/start_all.sh`
- 等待15秒让所有节点完全启动
- 检查LeafDetection终端是否有错误信息

### 错误: "机械臂移动失败"

**解决方法**：
- 检查MoveitServer终端是否有规划错误
- 确保碰撞对象已正确添加
- 检查RViz中是否有碰撞警告
- 尝试增加`--offset-z`值

### 错误: "install/setup.bash 文件不存在"

**解决方法**：
- 首先运行 `./default_scripts/start_all.sh` 构建工作空间
- 或手动构建: `colcon build --symlink-install`

## 📚 更多信息

详细文档请参考：
- [task_automation包文档](../src/task_automation/README.md)
- [叶子检测服务文档](../src/detect_leaf_pkg/USAGE_SERVER.md)

## 💡 提示

- 每次修改代码后需要重新构建: `colcon build --symlink-install`
- 确保相机已正确安装和启动
- 使用 `ros2 topic list` 和 `ros2 service list` 检查系统状态
- 在RViz中可视化机器人运动和叶子检测结果

