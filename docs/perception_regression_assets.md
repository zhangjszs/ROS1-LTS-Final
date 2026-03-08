# Perception Regression Asset Inventory
> 2026-03-06 状态更新

## 已有的资产
| 模式 | Bag 路径 | 大小 | 可用性 | GT 状态 |
|------|---------|------|--------|---------|
| track | `/home/kerwin/rosbag/track.bag` | 5.1GB | ✅ 可用 | ❌ 缺失 |
| accel | `/home/kerwin/rosbag/accel.bag` | 995MB | ✅ 可用 | ❌ 缺失 |
| skidpad | `/home/kerwin/rosbag/skidpad.bag` | 6.2GB | ✅ 可用 | ❌ 缺失 |

## GT CSV 输入契约（正式规范）
### 文件格式：
- **扩展名**：`.csv`
- **编码**：UTF-8
- **行分隔符**：`\n`
- **列分隔符**：英文逗号 `,`
- **注释行**：以 `#` 开头的行会被忽略

### 必填列（按顺序）：
| 列号 | 字段 | 含义 | 单位 | 坐标系 |
|------|------|------|------|--------|
| 1 | x | 锥桶X轴坐标（车辆正前方为正） | 米（m） |  LiDAR 传感器坐标系（`velodyne` 帧） |
| 2 | y | 锥桶Y轴坐标（车辆左侧为正） | 米（m） |  LiDAR 传感器坐标系（`velodyne` 帧） |
| 3 | z | 锥桶Z轴坐标（向上为正） | 米（m） |  LiDAR 传感器坐标系（`velodyne` 帧） |

### 命名规范：
- 命名格式：`{mode}_gt.csv`
- 示例：`track_gt.csv`、`accel_gt.csv`、`skidpad_gt.csv`

### 推荐路径：
```
~/rosbag/
├── track.bag
├── track_gt.csv
├── accel.bag
├── accel_gt.csv
├── skidpad.bag
└── skidpad_gt.csv
```

## 基线类型定义
### 1. Proxy-only Baseline
- 生成条件：冻结时不提供 `--gt` 参数
- 包含指标：仅稳定性/质量类代理指标（平均检测数、置信度、对称性等10项）
- JSON 标记：`"baseline_ready": true`, `"has_gt": false`
- 适用场景：快速回归验证感知 pipeline 稳定性，不需要精度指标

### 2. Full Baseline
- 生成条件：冻结时提供有效的 `--gt` 参数
- 包含指标：代理指标 + GT精度指标（precision/recall/F1/RMSE等6项）
- JSON 标记：`"baseline_ready": true`, `"has_gt": true`
- 适用场景：正式版本冻结、CI门禁、精度回归验证

## 当前基线冻结状态
- 三套模式的 baseline 模板已就位：`perf_reports/baselines/perception/{mode}_baseline.json`
- 当前状态：`baseline_ready = false`（模板状态）
- 冻结工具已就绪：`scripts/freeze_perception_baseline.sh`
- 回归验证工具已就绪：`scripts/check_perception_regression_mode.sh`
- CLI 已统一，参数传递链路通畅

## 下一步需要补充的资产
1. `track_gt.csv`：对应 track.bag 的锥桶真实坐标
2. `accel_gt.csv`：对应 accel.bag 的锥桶真实坐标
3. `skidpad_gt.csv`：对应 skidpad.bag 的锥桶真实坐标