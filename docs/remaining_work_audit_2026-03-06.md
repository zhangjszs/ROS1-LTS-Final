# Remaining Work Audit (2026-03-06)

> **目的**: 统一记录当前仓库中“真正未完成”的事项，避免继续受历史计划文档和阶段性交接文档的状态漂移影响。
> **核对范围**: 顶层交接文档、`docs/` 下计划文档、launch/config、核心源码、测试、脚本。
> **判定原则**: 以仓库现状为准；若文档与代码冲突，则同时记录“当前代码状态”和“文档提及但未实现/未收口”的差异。

---

## 1. 真实未完成的任务

### 1.1 EBS / VCU / 实车切换
- `VCU` 协议映射仍未完成：命令话题、状态话题、制动字段、EBS arm 字段、触发条件、心跳超时、failsafe 行为均未定稿。
- `ebs_test.launch` 仍是接口预留形态：`planner:=none`，`control_mode=5`，但控制侧已接入 dedicated `EbsController`。
- 上车前检查仍未完成：
  - 低速台架验证 `enable_ebs_control:=true`
  - EBS 场景回归包（触发/不触发/超时/断链）
  - A13 外参与定位误差复测
- 额外风险：`acceleration.launch` 默认值已改为 `control_mode=2`，但仍需和车端协议联调结果做一次一致性确认。
- 2026-03-18（本轮）代码契约核对进展：
  - 已将 `acceleration.launch(control_mode=2)`、`ebs_test.launch(control_mode=5)`、`mission_stack` 透传关系固化为文本合约测试。
  - 已将 `control_node` 的 `mode(kEbs)->working_mode=2` 映射，以及 `vehicle_interface` 超时 failsafe 的 `working_mode=2` 行为固化为测试。
  - 已新增 `docs/system_specifications/05_vehicle_control_protocol_contract.md`，固化当前 `VCU/EBS` 字段与字节位契约。
  - 结论：仓库内“控制模式编号 -> 控制节点输出 -> 车端 working_mode”链路在代码层保持一致；剩余风险在车端协议联调与台架验收。

### 1.2 感知回归基线落地
- ✅ `check_perception_regression.sh` / `freeze_perception_baseline.sh` / `check_perception_regression_mode.sh` 已存在，且 CLI 已统一，参数可以完整传递。
- ✅ 原始传感器 bag 已就绪：`~/rosbag/` 目录下已存在 `track.bag` / `accel.bag` / `skidpad.bag` 三套模式的测试 bag，包含原始点云数据
- ✅ 已补录检测输出 bag：`/tmp/perception_baseline_seed/*_detections.bag`（`/perception/lidar_cluster/detections`）
- ✅ 已冻结三套 Full baseline：`perf_reports/baselines/perception/*_baseline.json` 均为 `baseline_ready=true` 且 `has_gt=true`
- ✅ 已生成并部分清洗 GT：`perf_reports/gt/{track,accel,skidpad}_gt.csv`（track/skidpad 已做窗口清洗）
- ✅ 已新增资产清单文档：`docs/perception_regression_assets.md`，记录当前资产状态、GT 输入契约和基线类型定义
- ✅ 冻结和回归流程已完全打通，支持两种基线类型：
  - Proxy-only Baseline：无需 GT，仅包含稳定性指标，可随时冻结
  - Full Baseline：需要 GT，包含完整精度指标
- ✅ 基线语义已明确：`baseline_ready` 标记基线是否可用，`has_gt` 标记是否包含 GT 精度指标
- ✅ 回归脚本支持 GT 状态不一致检测，会给出明确警告
- ✅ 模式包装脚本和底层脚本的 CLI 已统一，可直接用于门禁。
- ✅ 当前三模式回归门禁已可通过（基于首版 GT 初稿 + 模式化门限）。
- ⚠️ 风险仍在：GT 初稿质量有限，后续需清洗 GT 并逐步收紧门限，避免长期门限漂移。

### 1.3 颜色语义全链路收口
- 目标语义是 `LEFT=RED / RIGHT=BLUE`，但当前仓库仍处于“消息层支持 + 部分模块接入 + 部分模块保留旧语义”的混合态。
- 2026-03-15（本轮）代码收口进展：
  - 已统一关键链路的颜色枚举边界为 `0..4`（`BLUE/YELLOW_SMALL/YELLOW_BIG/RED/NONE`），并在入口处兼容 legacy `5 -> RED`。
  - 已在 `planning_ros/high_speed` 明确 `LEFT=RED / RIGHT=BLUE`，同时保留 `YELLOW_*` 作为左边界兼容输入。
  - 已在 `localization_ros -> localization_core(FG)` 入口做颜色归一化，避免旧值渗透到 FG 关联与重定位描述子。
- 2026-03-17（本轮）回放验收进展：
  - 已完成修复后二次 replay 采样并输出 `perf_reports/results/color_semantics_replay_validation_20260317.{json,md}`。
  - 三模式均已录到 fused 消息：
    - `track`: fused `1153` 帧，overall consistency `0.7582`（较 2026-03-15 的 `0.4925` 明显提升）
    - `accel`: fused `932` 帧，overall consistency `1.0000`
    - `skidpad`: fused `1683` 帧，overall consistency `0.9834`
- 2026-03-17（同轮）门禁接入进展：
  - 已新增 `scripts/check_color_semantics_regression_mode.sh`，并冻结三模式阈值：
    - `perf_reports/baselines/perception/track.color_semantics.thresholds.env`
    - `perf_reports/baselines/perception/accel.color_semantics.thresholds.env`
    - `perf_reports/baselines/perception/skidpad.color_semantics.thresholds.env`
  - `scripts/check_perception_regression_mode.sh` 已支持 `--color-bag` 串行触发颜色语义门禁。
- 2026-03-18（本轮）CI 接入进展：
  - 已新增 `.github/workflows/perception_color_semantics_ci.yaml`，将颜色语义门禁接入常规 CI。
  - workflow 中覆盖门禁资产存在性检查、shell/python 语法检查与 `test_mainline_adapter_launch_contract.py` 合约测试。
- 需要统一收口的模块：
  - `perception_ros`
  - `localization_core/localization_ros`
  - `planning_core`
  - `planning_ros/high_speed_tracking`
  - `fsd_visualization`
  - 测试与验证脚本
- 当前更准确的状态是“fused 链路、阈值冻结、脚本门禁与常规 CI 均已落地；剩余工作是按业务目标决定是否继续提升 track 一致率”。

### 1.4 Localization FG 主后端迁移
- 2026-03-20（本轮）已完成 FG 主后端迁移回放验收（功能回归，非参数细调）：
  - 已落地切主回放脚本 `scripts/validate_localization_fg_mainline_replay.sh`
  - 已验证主后端切主参数链路：`backend:=factor_graph` + `fg_shadow_mode:=false` + `publish_dual_backends:=true`
  - `trackdrive` 回放验收 PASS（samples=339），且关键检查均命中：
    - `backend_factor_graph_seen=yes`
    - `active_source_fg_seen=yes`
    - `fg_shadow_false_seen=yes`
  - 验收记录：`perf_reports/results/localization_fg_mainline_20260320_094935/acceptance_report.md`
- 当前剩余未完成项：
  - 驱动字段补全（acc / Roll / NSV / Age）
  - 标准场景矩阵冻结（autocross / acceleration / skidpad）
  - 实 bag 回放门禁接入 CI

### 1.5 Planning 设计项真正未落地的部分
- `HUAT_PathLimitsV2` message/转换工具已落地，且 `planning_pipeline(line/skidpad/high_speed)` 已接入 V1/V2 双发布过渡主链（`planning/pathlimits` + `planning/pathlimits_v2`）。
- `control_ros` 已接入 V1/V2 并行消费开关（`pathlimits_topic/pathlimits_v2_topic` + `compat` 订阅策略），并补齐 V2 优先时的自动回退框架（V2 stale 时回退 V1，恢复后切回 V2）。
- 赛项入口 `trackdrive/autocross/acceleration/skidpad.launch` 已暴露控制侧切主开关，可直接在 mission 级别做 V2 切主/回退验证。
- 已新增端到端回放验收脚本 `scripts/validate_pathlimits_v2_cutover_replay.sh`，覆盖 `v2_primary` 与 `v2_fallback_no_v2_publish` 双场景并输出结构化验收结果。
- 2026-03-18 回放验收更新：
  - 已新增矩阵脚本 `scripts/run_pathlimits_v2_cutover_matrix.sh`，可批量执行多 mission 回放并输出总表。
  - 已形成正式验收记录 `perf_reports/results/pathlimits_v2_cutover_20260318/acceptance_report.md`。
  - `trackdrive/autocross/acceleration` 在本轮回放中稳定 PASS；`skidpad` 出现 replay 级随机抖动（存在失败与重试通过样本）。
- 当前剩余：实车联调切主验收 + `skidpad` 回放稳定性收敛。
- `PLANNING_M2_AUTOCROSS_DESIGN.md` 中的 Boundary Graph / Corridor / Centerline Optimizer 仍未实现。
- `PLANNING_M3_LINEAR_ACCEL_DESIGN.md` 中的分段速度规划与 V2 完整输出未实现。
- `PLANNING_M4_SKIDPAD_DESIGN.md` 中的双圆拟合增强、圈次状态机、相位速度控制未按设计收口。
- `Planning M5` 已验证冻结，但 Phase 3 遗留清理和实车验证仍建议继续完成。

### 1.6 Localization 验证工具与风险清单配套脚本
- 2026-03-15 复核：以下工具已在仓库落地：
  - `perf_reports/scripts/inject_scenario.py`
  - `perf_reports/scripts/localization_health_check.sh`
  - `perf_reports/scripts/localization_log_check.sh`
  - `perf_reports/scripts/compare_backends.py`
  - `perf_reports/scripts/cone_gt_annotator.py`
  - `.github/workflows/localization_regression_ci.yaml`
- 当前剩余问题不再是“缺文件”，而是：
  - 标准场景数据集与 KPI 阈值尚未冻结
  - CI 仍以脚本可执行/单测为主，尚未接入实 bag 回放验收

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
- 2026-03-15 复核：`control_ros/control_node.cpp` 已新增 `kEbs` 分支并直接下发紧急制动命令；
  但 `ebs_test.launch` 与车端协议联调、故障注入回归尚未收口。

### 2.3 `HUAT_PathLimitsV2`
- 规划接口文档多次提到新增 `HUAT_PathLimitsV2`、双发布和 `pathlimits_v2` 系列话题。
- 2026-03-18 更新：`HUAT_PathLimitsV2.msg`、V1->V2 转换/校验工具、主链双发布（line/skidpad/high_speed）、
  控制侧 V1/V2 并行消费与 V2 stale 自动回退框架均已落地；
  回放验收脚本与记录已落地（含矩阵执行）；当前剩余工作是最终实车切主验收、回退策略联调验证及 `skidpad` 稳定性收敛。

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
- 场景注入脚本与标准场景矩阵（注入脚本已具备，`trackdrive` 已完成主后端切主验收，其他 mission 待冻结）
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
2. perception baseline 已完成首版冻结与回归闭环，下一步是 GT 清洗与门限收紧
3. 颜色语义门禁已接入常规 CI（必要时继续优化 `track` 一致率）
4. `acceleration control_mode=2` 已完成代码侧一致性固化；下一步是车端协议联调确认

### P1
1. 补齐 localization 验证工具和标准场景
2. 将 FG 主后端迁移验收从 `trackdrive` 扩展到全 mission 矩阵并固化门禁
3. 推进 `HUAT_PathLimitsV2` 实车切主验收，并收敛 `skidpad` 回放稳定性

### P2
1. 持续扩展实车验证
2. 继续清理历史文档漂移，保持“状态文档”与“设计文档”分离
