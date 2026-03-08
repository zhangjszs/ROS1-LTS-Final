# use_sim_time 配置管理规范

## 概述

`/use_sim_time` 是ROS的全局参数，控制节点是否使用仿真时间（来自`/clock` topic）还是系统时间。
本文档规范化该参数的设置策略，避免重复配置和不一致问题。

## 设计原则

1. **单一真值源（Single Source of Truth）**：`/use_sim_time`应在顶层launch文件中统一设置
2. **全局参数命名**：必须使用`/use_sim_time`（带前导斜杠），确保全局可见
3. **避免重复设置**：子launch文件不应重复设置该参数
4. **条件逻辑集中**：simulation模式判断逻辑应集中在顶层

## 配置层级

```
顶层launch文件（设置/use_sim_time）
├── mission_stack.launch          ✅ 根据simulation参数设置
├── full_simulation.launch        ✅ 固定设置为true
├── debug_perception_location.launch ✅ 根据simulation参数设置
└── play_*_bag.launch            ✅ 固定设置为true（独立测试用）

子系统launch文件（不设置/use_sim_time）
├── perception_ros/lidar_cluster.launch  ✅ 已移除重复设置
├── planning_ros/high_speed_tracking.launch
├── localization_ros/location.launch
└── control_ros/controler.launch
```

## 正确用法

### 1. 顶层launch文件（推荐）

**mission_stack.launch** - 生产环境入口：
```xml
<launch>
    <arg name="simulation" default="false"/>

    <!-- 根据simulation参数动态设置 -->
    <param name="/use_sim_time" value="$(arg simulation)"/>

    <!-- 包含子系统 -->
    <include file="..."/>
</launch>
```

**play_high_speed_bag.launch** - 独立测试：
```xml
<launch>
    <arg name="bag" default="/path/to/bag.bag" />

    <!-- 独立测试场景固定为true -->
    <param name="/use_sim_time" value="true" />

    <node pkg="rosbag" type="play" name="rosbag_play" args="--clock $(arg bag)" />
</launch>
```

### 2. 子系统launch文件（禁止设置）

**perception_ros/lidar_cluster.launch**：
```xml
<launch>
    <!-- ❌ 错误：不应在此设置use_sim_time -->
    <!-- <param name="/use_sim_time" value="true" if="$(eval arg('bag') != '')" /> -->

    <!-- ✅ 正确：假设父launch已设置，直接使用 -->
    <node pkg="..." type="..." name="...">
        <!-- 节点会自动读取全局/use_sim_time参数 -->
    </node>
</launch>
```

## 常见错误

### ❌ 错误1：使用相对参数名
```xml
<!-- 错误：缺少前导斜杠，变成私有参数 -->
<param name="use_sim_time" value="true" />
```

### ❌ 错误2：子launch重复设置
```xml
<!-- 错误：在子launch中重复设置 -->
<launch>
    <param name="/use_sim_time" value="true" if="$(eval arg('bag') != '')" />
    <include file="$(find perception_ros)/launch/lidar_cluster.launch" />
</launch>
```

### ❌ 错误3：条件逻辑分散
```xml
<!-- 错误：每个launch都判断bag参数 -->
<!-- 应该在顶层统一判断simulation参数 -->
```

## 验证方法

### 1. 检查参数设置
```bash
# 启动系统后检查
rosparam get /use_sim_time
# 应返回: true (仿真) 或 false (实车)
```

### 2. 检查时间源
```bash
# 仿真模式应该有/clock topic
rostopic hz /clock
# 实车模式不应有/clock topic
```

### 3. 检查节点时间
```bash
# 查看节点是否正确使用仿真时间
rosnode info /perception/lidar_cluster/lidar_cluster_node
# 检查published topics的时间戳
```

## 修复历史

### Task #7 (P1) - 2026-02-18
**问题**：
- `play_high_speed_bag.launch`等文件使用`use_sim_time`（无斜杠）
- `lidar_cluster.launch`根据bag参数条件设置，导致重复配置
- 配置逻辑分散，难以维护

**修复**：
- 统一使用`/use_sim_time`（全局参数）
- 移除子launch文件中的重复设置
- 添加注释说明配置策略

**影响文件**：
- `planning_ros/launch/play_high_speed_bag.launch`
- `planning_ros/launch/play_line_detection_bag.launch`
- `planning_ros/launch/play_skidpad_bag.launch`
- `perception_ros/launch/lidar_cluster.launch`

## 参考资料

- [ROS Wiki: Clock](http://wiki.ros.org/Clock)
- [ROS Wiki: rosbag/Commandline](http://wiki.ros.org/rosbag/Commandline)
- CLAUDE.md - Launch File Organization
