# Perception Baselines

本目录固定维护 `track` / `accel` / `skidpad` 三种模式的回归基线与阈值约定。

## 文件说明

### 基线文件
- `track_baseline.json` / `accel_baseline.json` / `skidpad_baseline.json`:
  - 存放对应模式的基线指标（`evaluate_perception_metrics.py` 输出格式）。
  - 初始模板 `baseline_ready=false`，必须先冻结真实 baseline 才能用于回归门禁。
  - 基线类型标记：
    - `"has_gt": false`：Proxy-only 基线，仅包含稳定性代理指标
    - `"has_gt": true`：Full 基线，包含完整的 GT 精度指标

### 阈值配置文件
- `track.thresholds.env` / `accel.thresholds.env` / `skidpad.thresholds.env`:
  - 对应模式的退化阈值约定（通过环境变量传入 `check_perception_regression.sh`）。

## GT CSV 规范
详细格式定义请见 `docs/perception_regression_assets.md`，核心要求：
- 格式：`x,y,z` 每行一个锥桶，单位米
- 坐标系：LiDAR 传感器帧（`velodyne`）
- 命名：`{mode}_gt.csv`，与对应 bag 同目录

## 基线冻结

### 前置要求
冻结基线需要 bag 中包含 `/perception/lidar_cluster/detections` 话题（感知模块的输出检测结果）。如果 bag 中只有原始点云，需要先启动感知节点在线处理：

```bash
# 1. 启动感知节点（以track模式为例）
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/track.bag perception:=true localization:=false planning:=false control:=false

# 2. 新开终端运行冻结脚本（会自动从rosbag回放的检测话题中提取指标）
scripts/freeze_perception_baseline.sh --mode track --bag /path/to/track.bag --force
```

### 冻结命令
```bash
# Proxy-only 基线（不需要GT）
scripts/freeze_perception_baseline.sh --mode track --bag /path/to/track.bag --force
scripts/freeze_perception_baseline.sh --mode accel --bag /path/to/accel.bag --force
scripts/freeze_perception_baseline.sh --mode skidpad --bag /path/to/skidpad.bag --force
```

可选 GT:

```bash
scripts/freeze_perception_baseline.sh \
  --mode track \
  --bag /path/to/track.bag \
  --gt perf_reports/gt_example.csv \
  --gt-threshold 1.0 \
  --gt-max-range 50.0 \
  --force
```

## 回归检查（按模式）

```bash
scripts/check_perception_regression_mode.sh --mode track --bag /path/to/track.bag
scripts/check_perception_regression_mode.sh --mode accel --bag /path/to/accel.bag
scripts/check_perception_regression_mode.sh --mode skidpad --bag /path/to/skidpad.bag
```

## 约定

- 模式 baseline 建议每次“参数体系大改”后更新一次，不要频繁刷新。
- 刷新 baseline 前先确认当前版本是可接受的“稳定版本”。
- `track` 通常最严格，`skidpad` 可以适度放宽阈值。

## CLI 统一说明 (2026-03-06)

- `check_perception_regression_mode.sh` 与底层 `check_perception_regression.sh` CLI 已完全统一，支持所有参数传递：
  - `--topic`、`--gt`、`--gt-threshold`、`--gt-max-range` 可完整传递到评估脚本
  - `--output-json` 支持自定义输出路径
  - 阈值配置从对应模式的 `.thresholds.env` 文件加载

## 资产状态 (2026-03-06)
✅ **标准 Bag 已就绪**：
- track: `/home/kerwin/rosbag/track.bag` (5.1GB)
- accel: `/home/kerwin/rosbag/accel.bag` (995MB)
- skidpad: `/home/kerwin/rosbag/skidpad.bag` (6.2GB)

⏳ **缺失 GT CSV**：
- 三套模式的真实 GT 标注尚未完成，目前只能冻结只包含代理指标的基线
- 完整的精度指标（precision/recall/F1/RMSE）需要 GT 支持

📋 **资产清单**：详细记录请见 `docs/perception_regression_assets.md`
