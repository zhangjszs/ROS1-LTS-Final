# Perception Baselines

本目录固定维护 `track` / `accel` / `skidpad` 三种模式的回归基线与阈值约定。

## 文件说明

- `track_baseline.json` / `accel_baseline.json` / `skidpad_baseline.json`:
  - 存放对应模式的基线指标（`evaluate_perception_metrics.py` 输出格式）。
  - 初始模板 `baseline_ready=false`，必须先冻结真实 baseline 才能用于回归门禁。
- `track.thresholds.env` / `accel.thresholds.env` / `skidpad.thresholds.env`:
  - 对应模式的退化阈值约定（通过环境变量传入 `check_perception_regression.sh`）。

## 基线冻结

```bash
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
