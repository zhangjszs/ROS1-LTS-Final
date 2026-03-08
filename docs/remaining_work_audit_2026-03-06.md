# Remaining Work Audit (2026-03-06)

> **目的**: 统一记录当前仓库中“真正未完成”的事项，避免继续受历史计划文档和阶段性交接文档的状态漂移影响。
> **核对范围**: 顶层交接文档、`docs/` 下计划文档、launch/config、核心源码、测试、脚本。
> **判定原则**: 以仓库现状为准；若文档与代码冲突，则同时记录“当前代码状态”和“文档提及但未实现/未收口”的差异。

---

## 1. 真实未完成的任务

### 1.1 EBS / VCU / 实车切换
- `VCU` 协议映射仍未完成：命令话题、状态话题、制动字段、EBS arm 字段、触发条件、心跳超时、failsafe 行为均未定稿。
- `ebs_test.launch` 仍是接口预留形态：`planner:=none`，`control_mode=5`，没有 dedicated EBS controller。
- 上车前检查仍未完成：
  - 低速台架验证 `enable_ebs_control:=true`
  - EBS 场景回归包（触发/不触发/超时/断链）
  - A13 外参与定位误差复测
- 额外风险：当前 `acceleration.launch` 使用 `control_mode=1`，控制侧会落到 `TestController`；这与部分旧文档中的“accel=2/LineController”不一致，需在控制/车端协议层重新确认。

### 1.2 感知回归基线落地
- ✅ `check_perception_regression.sh` / `freeze_perception_baseline.sh` / `check_perception_regression_mode.sh` 已存在，且 CLI 已统一，参数可以完整传递。
- ✅ 原始传感器 bag 已就绪：`~/rosbag/` 目录下已存在 `track.bag` / `accel.bag` / `skidpad.bag` 三套模式的测试 bag，包含原始点云数据
- ⚠️ 现有 bag 仅包含原始 `/velodyne_points` 话题，没有录制感知检测输出 `/perception/lidar_cluster/detections`，冻结基线时需要启动感知节点在线处理
- ⏳ `perf_reports/baselines/perception/*_baseline.json` 全部仍为模板，`baseline_ready=false`，等待 GT CSV 标注完成后可一键冻结
- ⏳ 缺少对应三套模式的 GT CSV 文件，导致无法冻结包含精度指标（precision/recall/F1）的完整 Full Baseline
- ✅ 已新增资产清单文档：`docs/perception_regression_assets.md`，记录当前资产状态、GT 输入契约和基线类型定义
- ✅ 冻结和回归流程已完全打通，支持两种基线类型：
  - Proxy-only Baseline：无需 GT，仅包含稳定性指标，可随时冻结
  - Full Baseline：需要 GT，包含完整精度指标
- ✅ 基线语义已明确：`baseline_ready` 标记基线是否可用，`has_gt` 标记是否包含 GT 精度指标
- ✅ 回归脚本支持 GT 状态不一致检测，会给出明确警告
- ✅ 模式包装脚本和底层脚本的 CLI 已统一，可直接用于门禁。

### 1.3 颜色语义全链路收口
- 目标语义是 `LEFT=RED / RIGHT=BLUE`，但当前仓库仍处于“消息层支持 + 部分模块接入 + 部分模块保留旧语义”的混合态。
- 需要统一收口的模块：
  - `perception_ros`
  - `localization_core/localization_ros`
  - `planning_core`
  - `planning_ros/high_speed_tracking`
  - `fsd_visualization`
  - 测试与验证脚本
- 当前更准确的状态是“颜色管道代码已大部分落地，但语义一致性和端到端验证未完成”。

### 1.4 Localization FG 主后端迁移
- `factor_graph` backend 已有较多实现，但 `location.cpp` 仍明确将其视为 `shadow mode`。
- 未完成项包括：
  - 驱动字段补全（acc / Roll / NSV / Age）
  - 主后端切换方案
  - 与 mapper 的自动对比验证
  - 回归 KPI 和失败场景验收

### 1.5 Planning 设计项真正未落地的部分
- `HUAT_PathLimitsV2` 未实现。
- V1/V2 双发布过渡未实现。
- `PLANNING_M2_AUTOCROSS_DESIGN.md` 中的 Boundary Graph / Corridor / Centerline Optimizer 仍未实现。
- `PLANNING_M3_LINEAR_ACCEL_DESIGN.md` 中的分段速度规划与 V2 完整输出未实现。
- `PLANNING_M4_SKIDPAD_DESIGN.md` 中的双圆拟合增强、圈次状态机、相位速度控制未按设计收口。
- `Planning M5` 已验证冻结，但 Phase 3 遗留清理和实车验证仍建议继续完成。

### 1.6 Localization 验证工具与风险清单配套脚本
- 仍缺少以下明确写在文档中的工具：
  - `perf_reports/scripts/inject_scenario.py`
  - `localization_health_check.sh`
  - `localization_log_check.sh`
  - `compare_backends.py`
  - `cone_gt_annotator.py`
  - `localization_regression_ci.yaml`

---

## 2. 文档中提及但仓库未实现的功能

### 2.1 独立的 `cone_color_fusion_node` / `detections_fused`
- 历史文档曾设想单独的 `cone_color_fusion_node`，输入 `HUAT_ConeDetections + Image + CameraInfo`，输出 `detections_fused`。
- 当前仓库实际实现不是这条路径，而是：
  - `vision_ros` 发布 `perception/vision/detections`
  - `perception_ros/lidar_cluster_ros` 可选订阅视觉检测并在发布 `color_types[]` 时做内联颜色注入
- 结论：文档提到的独立融合节点与 `detections_fused` 话题尚未实现。

### 2.2 Dedicated EBS controller
- 文档中多处把 `control_mode=5` 描述为 EBS 预留模式。
- 当前代码里 `mode=5` 并没有独立控制器，最终仍走 `HighController` 分支。

### 2.3 `HUAT_PathLimitsV2`
- 规划接口文档多次提到新增 `HUAT_PathLimitsV2`、双发布和 `pathlimits_v2` 系列话题。
- 当前仓库没有该 message，也没有对应的话题或控制侧兼容逻辑。

### 2.4 文档索引中提到但实际不存在的文档
- 顶层 `README.md` 旧版提到：
  - `REFACTOR_REPORT.md`
  - `EXECUTIVE_SUMMARY.md`
  - `docs/CODECOV_SETUP.md`
  - `docs/GIT_HISTORY_CLEANUP.md`
- 这些文件当前不在仓库中，已在本轮文档修正中从索引处移除。

---

## 3. 可能遗漏的重要实现步骤或依赖

### 3.1 视觉模块上线依赖
- 训练数据集与标注规范
- YOLO/ONNX/TensorRT 模型资产
- 相机标定参数
- 带图像输入的标准 rosbag
- 与 LiDAR 时间对齐和降级策略验证

### 3.2 感知回归真正成为门禁前的前置项
- 冻结稳定版 baseline
- 明确每个赛项对应的标准 bag 与 GT
- 修复/统一回归脚本 CLI
- 明确何时更新 baseline，避免每次参数小改都刷新基线

### 3.3 EBS 上车前的硬依赖
- 车端协议定稿
- failsafe 和 heartbeat 策略
- 台架验证与人工复核流程
- GNSS/INS 质量门控在 EBS 任务中的使用边界
- 触发逻辑与非触发逻辑的回归包

### 3.4 Localization FG 真正切主前的依赖
- `vehicle_interface_ros` 关键字段补全
- mapper vs FG 自动对比工具
- 场景注入脚本与标准场景矩阵
- 回归 KPI（精度、抖动、重定位成功率、延迟）统一产出

### 3.5 文档治理本身
- 当前至少存在以下容易误导开发的历史漂移：
  - `HANDOFF.md` 曾将 G4/G17/G18 全部列为未完成
  - `未完成计划总结报告` 曾将 Vision / Color Pipeline Gaps 写成未开始
  - `audit_fixes_progress.md` 顶部写 22/22 完成，但中部保留了执行期“待测试/未提交”措辞
- 本轮已修正主要索引与状态页，但后续新增计划文档仍应明确标注“设计文档”还是“当前状态文档”。

---

## 4. 已确认不应再视为未完成的事项

### 4.1 G4 输入边界防御
- 已在 `lidar_cluster_ros.cpp` 通过 `input_guard/*` 参数和空点云/超大点云/无效点过滤实现。

### 4.2 G17 perception_core 关键测试补齐
- 已有：
  - `test_confidence_scorer.cpp`
  - `test_cone_tracker.cpp`
  - `test_topology_repair.cpp`
  - `test_cluster_feature_extractor.cpp`
  - `test_ground_segmentation_perf.cpp`
  - `test_point_cloud_pool.cpp`

### 4.3 Color Pipeline Gaps 的 5 个计划任务
- 代码侧已基本落地，剩余问题不在“把代码写出来”，而在端到端验收和语义一致性收口。

### 4.4 Audit Fixes 22/22
- 审计修复进度文档顶部状态已经是完成，不应再按旧总结文档把它当作“进行中”。

---

## 5. 建议的后续优先级

### P0
1. 完成 EBS/VCU 协议与台架验证
2. 冻结 perception baseline 并打通回归闭环
3. 收口颜色语义在全链路的一致性
4. 确认 acceleration 当前 `control_mode=1 -> TestController` 是否为预期

### P1
1. 补齐 localization 验证工具和标准场景
2. 推进 FG 主后端迁移
3. 评估 `HUAT_PathLimitsV2` 是否真的进入实现阶段

### P2
1. 持续扩展实车验证
2. 继续清理历史文档漂移，保持“状态文档”与“设计文档”分离
