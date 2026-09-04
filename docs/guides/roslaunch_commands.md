# Roslaunch 启动指令参考

> 用于 2025HUAT FSD 全栈仿真回放与验证。
> 对应代码版本：main 分支，统一 Planning Pipeline + FG 主后端已就绪。

---

## 前置条件

```bash
cd ~/2025huat
source devel/setup.bash
```

---

## 可用 Rosbag 清单

| 文件 | 大小 | 适用 Mission | 备注 |
|------|------|-------------|------|
| `/home/kerwin/rosbag/track.bag` | 5.1G | trackdrive / autocross | 推荐全栈验证用，121s |
| `/home/kerwin/rosbag/accel.bag` | 995M | acceleration | 直线加速赛 |
| `/home/kerwin/rosbag/skidpad.bag` | 6.2G | skidpad | 八字绕环 |
| `/home/kerwin/rosbag/2024-10-17-01-19-05.bag` | 681M | — | LiDAR only，无 INS |

---

## 1. 标准 Mission 启动（Simulation + Rosbag）

### TrackDrive（高速循迹）

```bash
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/home/kerwin/rosbag/track.bag rate:=1.0 loop:=false rviz_mode:=dual
```

### Autocross（障碍赛 / 绕桩）

```bash
roslaunch fsd_launch autocross.launch simulation:=true bag:=/home/kerwin/rosbag/track.bag rate:=1.0 rviz_mode:=dual
```

### Acceleration（直线加速）

```bash
roslaunch fsd_launch acceleration.launch simulation:=true bag:=/home/kerwin/rosbag/accel.bag rate:=1.0 rviz_mode:=dual
```

### Skidpad（八字绕环）

```bash
roslaunch fsd_launch skidpad.launch simulation:=true bag:=/home/kerwin/rosbag/skidpad.bag rate:=1.0 rviz_mode:=dual
```

---

## 2. Factor Graph（FG）主后端验证

```bash
# TrackDrive FG 主链路验证
bash scripts/validate_localization_fg_mainline_replay.sh --mission trackdrive --bag /home/kerwin/rosbag/track.bag --rate 2.0 --compare-duration 60

# Autocross
bash scripts/validate_localization_fg_mainline_replay.sh --mission autocross --bag /home/kerwin/rosbag/track.bag

# Acceleration
bash scripts/validate_localization_fg_mainline_replay.sh --mission acceleration --bag /home/kerwin/rosbag/accel.bag

# Skidpad
bash scripts/validate_localization_fg_mainline_replay.sh --mission skidpad --bag /home/kerwin/rosbag/skidpad.bag
```

**脚本关键行为：**
- 自动设置 `backend:=factor_graph`、`publish_dual_backends:=true`、`fg_shadow_mode:=false`
- 对比 `/localization/mapper/car_state` 与 `/localization/fg/car_state`
- 输出到 `perf_reports/results/localization_fg_mainline_YYYYMMDD_HHMMSS/`
- 检查 diagnostics 中 `active_backend_source=factor_graph`

---

## 3. Fusion Sanity Test（感知融合验证）

验证：无 `camera_info` 时 legacy HFOV 不运行，cone 回退到 LiDAR 几何颜色。

```bash
bash scripts/run_fusion_sanity_test.sh /home/kerwin/rosbag/track.bag
```

**脚本行为：**
- 启动 `perception_ros lidar_cluster.launch` + rosbag play
- 启动 `fake_vision_publisher.py` 发布空视觉检测
- 运行 `analyze_fusion_sanity.py` 检查 50 帧
- 通过标准：无 `LEGACY_HFOV_MATCH(8)` 状态，全部为 `CAMERA_INFO_MISSING(2)`

---

## 4. PathLimits V2 双发布验证

```bash
bash scripts/validate_pathlimits_v2_cutover_replay.sh --mission trackdrive --bag /home/kerwin/rosbag/track.bag --rate 2.0
```

验证 Planning 同时发布 V1 (`/planning/pathlimits`) 和 V2 (`/planning/pathlimits_v2`)。

---

## 5. 关键参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `simulation` | `false` | 仿真模式（必设 `true` 才播放 rosbag） |
| `bag` | `""` | rosbag 路径 |
| `rate` | `1.0` | 回放倍速 |
| `loop` | `false` | 循环播放 |
| `start` | `0` | 起始时间（秒），trackdrive 支持 |
| `backend` | `mapper` | 定位后端：`mapper` / `factor_graph` |
| `publish_dual_backends` | `false` | 是否同时发布 mapper/fg 双后端 |
| `fg_shadow_mode` | `true` | FG 是否仅影子模式 |
| `fg_mainline_enable_mapper_fallback` | `true` | FG 主链是否回退 mapper |
| `rviz_mode` | `main` | `main` / `global` / `pointcloud` / `dual` |
| `launch_rviz` | `true` | 是否启动 RViz |
| `launch_viz` | `true` | 是否启动 fsd_visualization |
| `control_mode` | 视 mission | `1`=test, `2`=line/accel, `3`=skidpad, `4`=track |

---

## 6. 常用诊断命令

```bash
# 查看所有诊断
rostopic echo /diagnostics

# 定位诊断
rostopic echo /localization/diagnostics

# 规划诊断
rostopic echo /planning/diagnostics

# 感知诊断
rostopic echo /perception/lidar_cluster/perception/diagnostics

# 查看规划输出
rostopic echo /planning/pathlimits
rostopic echo /planning/pathlimits_v2

# 查看锥桶检测
rostopic echo /perception/lidar_cluster/detections

# 查看车辆状态
rostopic echo /localization/car_state

# TF 检查
rosrun tf tf_echo velodyne base_link

# Topic 频率
rostopic hz /velodyne_points
rostopic hz /planning/pathlimits
```

---

## 7. 无图形界面快速验证（CI / Headless）

```bash
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/home/kerwin/rosbag/track.bag rate:=2.0 launch_rviz:=false launch_viz:=false backend:=factor_graph publish_dual_backends:=true fg_shadow_mode:=false
```

---

## 8. 关键文件路径

| 文件 | 路径 |
|------|------|
| Mission Launch | `src/fsd_launch/launch/{trackdrive,autocross,acceleration,skidpad}.launch` |
| Planning Pipeline Launch | `src/planning_ros/launch/planning_pipeline.launch` |
| Perception Launch | `src/perception_ros/launch/lidar_cluster.launch` |
| FG 验证脚本 | `scripts/validate_localization_fg_mainline_replay.sh` |
| Fusion Sanity 脚本 | `scripts/run_fusion_sanity_test.sh` |
| V2 Cutover 脚本 | `scripts/validate_pathlimits_v2_cutover_replay.sh` |
| FG 门禁阈值 | `perf_reports/baselines/localization/fg_gate_thresholds.yaml` |
| CI Workflow | `.github/workflows/localization_regression_ci.yaml` |
