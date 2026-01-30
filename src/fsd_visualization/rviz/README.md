# RViz 配置文件管理文档

## 目录结构

```
src/fsd_visualization/rviz/
├── README.md                    # 本文档
├── fsd_default.rviz             # 默认配置（基础模板）
├── fsd_main.rviz                # 综合主视图（待完善）
├── global_viz.rviz              # 全局俯视图
├── pointcloud_only.rviz         # 仅点云视图
├── display_high.rviz            # 直线检测可视化
├── display_skidpad.rviz         # 八字绕环可视化
└── lidar_cluster.rviz           # 激光雷达聚类可视化
```

## 配置文件说明

### 1. fsd_default.rviz（默认配置）

**用途：** 基础配置模板，所有其他配置文件的基础

**配置特点：**
- Fixed Frame: `velodyne`
- 显示元素：坐标轴(Axes)、网格(Grid)
- 背景色：深灰色(48, 48, 48)
- 帧率：30 FPS

**使用场景：** 作为新配置文件的起点

---

### 2. fsd_main.rviz（综合主视图）

**用途：** 默认主视图，包含所有必要的可视化元素

**状态：** ⚠️ 待完善（当前为空文件）

**建议配置：**
- 合并 `pointcloud_only.rviz` 和 `global_viz.rviz` 的功能
- 包含点云、路径、锥桶、车辆等所有可视化元素

---

### 3. global_viz.rviz（全局俯视图）

**用途：** 全局俯视图，显示规划结果和车辆状态

**配置特点：**
- Fixed Frame: `world`
- 显示元素：
  - `/fsd/viz/boundaries` - 边界标记
  - `/fsd/viz/cones` - 锥桶标记
  - `/fsd/viz/path` - 路径标记
  - `/fsd/viz/vehicle` - 车辆标记

**使用场景：** 查看全局路径规划和车辆状态

**启动方式：**
```bash
roslaunch fsd_launch rviz.launch mode:=global
```

---

### 4. pointcloud_only.rviz（仅点云视图）

**用途：** 仅显示点云数据，用于调试感知模块

**配置特点：**
- Fixed Frame: `velodyne`
- 显示元素：
  - `/perception/lidar_cluster` - 点云数据
  - 坐标轴(Axes)
  - 网格(Grid)
  - `/resize_img_out` - 图像显示

**使用场景：** 调试激光雷达感知算法

**启动方式：**
```bash
roslaunch fsd_launch rviz.launch mode:=pointcloud
```

---

### 5. display_high.rviz（直线检测可视化）

**用途：** 直线检测任务的可视化配置

**配置特点：**
- Fixed Frame: `velodyne`
- 显示元素：基础配置（坐标轴、网格）

**使用场景：** 直线检测任务调试

**引用位置：**
- `_deprecated/planning/line_detection/launch/play_line_detection_bag.launch:27`

---

### 6. display_skidpad.rviz（八字绕环可视化）

**用途：** 八字绕环任务的可视化配置

**配置特点：**
- Fixed Frame: `velodyne`
- 显示元素：基础配置（坐标轴、网格）

**使用场景：** 八字绕环任务调试

**引用位置：**
- `_deprecated/planning/skidpad_detection/launch/play_skidpad_bag.launch:27`

---

### 7. lidar_cluster.rviz（激光雷达聚类可视化）

**用途：** 激光雷达点云聚类算法的可视化配置

**配置特点：**
- Fixed Frame: `velodyne`
- 显示元素：基础配置（坐标轴、网格）

**使用场景：** 感知模块点云聚类调试

**引用位置：**
- `perception_ros/launch/lidar_cluster.launch:3`

**启动方式：**
```bash
roslaunch perception_ros lidar_cluster.launch
```

---

## 使用方法

### 方式1：通过主启动文件

```bash
# 综合主视图（默认）
roslaunch fsd_launch rviz.launch

# 全局俯视图
roslaunch fsd_launch rviz.launch mode:=global

# 仅点云视图
roslaunch fsd_launch rviz.launch mode:=pointcloud

# 双窗口模式
roslaunch fsd_launch rviz.launch mode:=dual
```

### 方式2：直接启动RViz

```bash
# 指定配置文件
rviz -d $(find fsd_visualization)/rviz/fsd_default.rviz
```

### 方式3：通过任务启动文件

```bash
# 感知模块
roslaunch perception_ros lidar_cluster.launch

# 直线检测（废弃代码）
roslaunch _deprecated/planning/line_detection/launch/play_line_detection_bag.launch

# 八字绕环（废弃代码）
roslaunch _deprecated/planning/skidpad_detection/launch/play_skidpad_bag.launch
```

---

## 配置规范

### 1. 命名规范

- 使用小写字母和下划线
- 描述性命名：`<用途>_<视图类型>.rviz`
- 示例：`pointcloud_only.rviz`、`global_viz.rviz`

### 2. 基础配置

所有RViz配置文件应包含以下基础元素：
- **Fixed Frame:** `velodyne` 或 `world`
- **Axes:** 坐标轴显示
- **Grid:** 网格显示
- **Background Color:** `48; 48; 48`（深灰色）
- **Frame Rate:** `30`

### 3. 话题订阅规范

| 话题类型 | 用途 | 推荐配置 |
|---------|------|---------|
| `/perception/lidar_cluster/points/passthrough` | 预处理点云 | PointCloud |
| `/fsd/viz/cones` | 锥桶标记 | MarkerArray |
| `/fsd/viz/path` | 路径标记 | MarkerArray |
| `/fsd/viz/vehicle` | 车辆标记 | MarkerArray |
| `/fsd/viz/boundaries` | 边界标记 | MarkerArray |

### 4. 颜色方案

| 元素 | 颜色 (RGB) | 用途 |
|------|-----------|------|
| 锥桶 | 255; 0; 0 | 红色 |
| 路径 | 0; 255; 0 | 绿色 |
| 边界 | 0; 0; 255 | 蓝色 |
| 车辆 | 255; 255; 0 | 黄色 |
| 点云 | 255; 255; 255 | 白色 |

---

## 维护指南

### 1. 创建新配置文件

```bash
# 1. 复制默认配置
cp /home/kerwin/2025huat/src/fsd_visualization/rviz/fsd_default.rviz \
   /home/kerwin/2025huat/src/fsd_visualization/rviz/your_new_config.rviz

# 2. 使用RViz编辑
rviz -d /home/kerwin/2025huat/src/fsd_visualization/rviz/your_new_config.rviz

# 3. 保存配置
# 在RViz中：File -> Save Config As
```

### 2. 更新现有配置

```bash
# 1. 启动RViz并加载配置
rviz -d /home/kerwin/2025huat/src/fsd_visualization/rviz/your_config.rviz

# 2. 修改配置

# 3. 保存配置
# 在RViz中：File -> Save Config
```

### 3. 配置文件版本控制

- 所有RViz配置文件应纳入版本控制
- 修改配置文件时，请在提交信息中说明修改内容
- 重大配置变更应更新本文档

### 4. 配置文件测试

修改配置文件后，应进行以下测试：
1. 确认RViz能正常加载配置
2. 确认所有话题能正常订阅
3. 确认可视化元素显示正确
4. 确认性能表现良好（帧率≥20 FPS）

---

## 常见问题

### Q1: RViz启动后显示空白？

**A:** 检查以下项：
- Fixed Frame是否正确设置
- 话题是否正常发布
- 网络连接是否正常

### Q2: 如何修改Fixed Frame？

**A:** 在RViz左侧面板的 "Global Options" -> "Fixed Frame" 中修改

### Q3: 如何添加新的可视化元素？

**A:** 在RViz左下角点击 "Add" 按钮，选择要添加的元素类型

### Q4: 配置文件修改后不生效？

**A:** 确认：
- 是否正确保存了配置文件
- launch文件中的路径是否正确
- 是否重新启动了RViz

### Q5: 如何优化RViz性能？

**A:** 可以尝试：
- 降低点云显示的点大小
- 减少历史轨迹长度
- 降低帧率设置
- 关闭不必要的可视化元素

---

## 更新日志

### 2025-01-25
- 创建统一的RViz配置文件管理文档
- 创建 `fsd_default.rviz` 默认配置文件
- 创建 `display_high.rviz` 配置文件
- 创建 `display_skidpad.rviz` 配置文件
- 创建 `lidar_cluster.rviz` 配置文件
- 更新 `perception_ros/launch/lidar_cluster.launch` 中的RViz引用路径

---

## 联系方式

如有问题或建议，请联系项目维护团队。

---

**注意：** 废弃代码已清理，所有RViz配置统一在 `fsd_visualization/rviz/` 目录下。
