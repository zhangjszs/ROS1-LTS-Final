# HUAT 自动驾驶系统技术文档中心

> 本目录包含 FSAE/FSAC 大学生方程式赛车自动驾驶系统（湖北汽车工业学院 翼驰车队 2025 赛季）的全部系统级设计规范、模块文档、实车指南与历史报告。

---

## 目录全景

```
docs/
├── system_specifications/    # 01-06 接口契约、TF 时钟一致性与安全规范
├── architecture/             # 核心系统与降级架构设计 (如纯几何鲁棒性)
├── modules/                  # 核心子系统技术文档与设计说明
│   ├── perception/           # 感知算法流程、点云处理分析与回归资产
│   ├── localization/         # 定位因子图、创新点与验证清单
│   └── planning/             # 统一规划管线、各赛事阶段设计与调参
├── competition/              # 赛事规程、赛道规范、锥桶参数与物料清单
├── guides/                   # 开发者指南、代码质量规范、实车参数备忘与启动指令
├── integration/              # VCU 协议映射、门控阈值与系统联调
├── plans/                    # 专项技术演进计划 (含 completed/ 已完成归档)
└── reports/                  # 历史审计报告、验收总结与阶段交接记录
```

---

## 1. 系统级规格文档 (System Specifications)

**目录**: [system_specifications/](system_specifications/)

| 文档 | 说明 |
|------|------|
| [01_interface_contract.md](system_specifications/01_interface_contract.md) | 端到端接口契约：LiDAR→Location→Planning→Control 全链路消息定义 |
| [02_tf_time_consistency.md](system_specifications/02_tf_time_consistency.md) | TF/时间戳/单位一致性检查清单与自检输出定义 |
| [03_failure_mode_library.md](system_specifications/03_failure_mode_library.md) | 端到端失败模式库（FMEA）：20 条典型故障场景与保护策略 |
| [04_evaluation_framework.md](system_specifications/04_evaluation_framework.md) | 评估与回放体系设计：指标定义、场景切片与测试模板 |
| [05_vehicle_control_protocol_contract.md](system_specifications/05_vehicle_control_protocol_contract.md) | 底盘控制协议契约规范 |
| [06_topic_frame_param_contract_v1_draft.md](system_specifications/06_topic_frame_param_contract_v1_draft.md) | 统一命名草案：Topic/Frame/Param 的 canonical 与兼容映射 |

**推荐阅读顺序**: 01 → 02 → 03 → 04 → 05

---

## 2. 架构设计 (Architecture)

**目录**: [architecture/](architecture/)

| 文档 | 说明 |
|------|------|
| [GEOMETRY_ROBUSTNESS.md](architecture/GEOMETRY_ROBUSTNESS.md) | 非视觉模式几何鲁棒性增强配置：堆叠锥去重、缺失锥插补与短路径抑制 |

---

## 3. 模块技术文档 (Modules)

### 3.1 感知模块 (Perception)
**目录**: [modules/perception/](modules/perception/)

| 文档 | 说明 |
|------|------|
| [lidar_pipeline.md](modules/perception/lidar_pipeline.md) | LiDAR 感知处理流程：点云直通滤波、地面分割、聚类与置信度评分 |
| [lidar_pipeline_analysis.md](modules/perception/lidar_pipeline_analysis.md) | LiDAR 全流程算法深度分析与阶段优化总结 |
| [perception_regression_assets.md](modules/perception/perception_regression_assets.md) | 感知回归测试包与颜色语义真值资产管理规范 |

### 3.2 定位模块 (Localization)
**目录**: [modules/localization/](modules/localization/)

| 文档 | 说明 |
|------|------|
| [LOCALIZATION_M1_INTERFACE_FREEZE.md](modules/localization/LOCALIZATION_M1_INTERFACE_FREEZE.md) | 定位输入输出接口冻结规范 |
| [LOCALIZATION_M2_CTF_GRAPH_DESIGN.md](modules/localization/LOCALIZATION_M2_CTF_GRAPH_DESIGN.md) | CTF 因子图（Factor Graph）架构设计与约束推导 |
| [LOCALIZATION_M3_LOOP_CLOSURE_RELOC.md](modules/localization/LOCALIZATION_M3_LOOP_CLOSURE_RELOC.md) | 回环检测与多阶段重定位方案 |
| [LOCALIZATION_M4_VALIDATION_PLAN.md](modules/localization/LOCALIZATION_M4_VALIDATION_PLAN.md) | 多任务回放与实车验证计划 |
| [LOCALIZATION_M5_RISK_CHECKLIST.md](modules/localization/LOCALIZATION_M5_RISK_CHECKLIST.md) | 风险排查清单与容灾阈值 |
| [LOCALIZATION_INNOVATION1_COLOR_TOPOLOGY.md](modules/localization/LOCALIZATION_INNOVATION1_COLOR_TOPOLOGY.md) | 创新点1：颜色-拓扑软关联因子 |
| [LOCALIZATION_INNOVATION2_SKIDPAD_CIRCLE.md](modules/localization/LOCALIZATION_INNOVATION2_SKIDPAD_CIRCLE.md) | 创新点2：八字绕环圆形先验约束 |
| [LOCALIZATION_INNOVATION3_RELOCALIZATION.md](modules/localization/LOCALIZATION_INNOVATION3_RELOCALIZATION.md) | 创新点3：几何指纹重定位机制 |

### 3.3 规划模块 (Planning)
**目录**: [modules/planning/](modules/planning/)

| 文档 | 说明 |
|------|------|
| [PLANNING_M1_INTERFACE_SPEC.md](modules/planning/PLANNING_M1_INTERFACE_SPEC.md) | 统一规划接口规范（V1 兼容 + V2 数组契约） |
| [PLANNING_M2_AUTOCROSS_DESIGN.md](modules/planning/PLANNING_M2_AUTOCROSS_DESIGN.md) | 综合赛道与高速循迹（第一圈保守探索、后续圈极致循迹） |
| [PLANNING_M3_LINEAR_ACCEL_DESIGN.md](modules/planning/PLANNING_M3_LINEAR_ACCEL_DESIGN.md) | 直线加速赛道设计（75m 全力冲刺 + 100m 安全制动） |
| [PLANNING_M4_SKIDPAD_DESIGN.md](modules/planning/PLANNING_M4_SKIDPAD_DESIGN.md) | 八字绕环设计（双圆识别拟合 + 圈次状态机） |
| [PLANNING_M5_VALIDATION_AND_MIGRATION_PLAN.md](modules/planning/PLANNING_M5_VALIDATION_AND_MIGRATION_PLAN.md) | 统一规划流水线验证、切主与回退策略 |
| [SKIDPAD_TUNING_TEMPLATE.md](modules/planning/SKIDPAD_TUNING_TEMPLATE.md) | 八字绕环调参模板 |

---

## 4. 赛事规范与赛道规则 (Competition)

**目录**: [competition/](competition/)

| 文档 | 说明 |
|------|------|
| [competition_guide.md](competition/competition_guide.md) | FSAC 比赛资料总览与环境说明入口 |
| [TRACK_SPECS.md](competition/TRACK_SPECS.md) | 赛道几何规格标准（直道宽、弯道半径、转弯角度） |
| [traffic_cone.md](competition/traffic_cone.md) | 赛会标准锥桶规格（红/黄/蓝/大黄锥规格与反光带定义） |
| [attachments/cones_list.csv](competition/attachments/cones_list.csv) | 锥桶物料采购与备件清单 |

---

## 5. 开发与操作指南 (Guides)

**目录**: [guides/](guides/)

| 文档 | 说明 |
|------|------|
| [code-quality-tools.md](guides/code-quality-tools.md) | 格式化与静态代码检查指南（clang-format, clang-tidy, flake8, black） |
| [code-coverage-guide.md](guides/code-coverage-guide.md) | 单元测试代码覆盖率统计与报告生成指南 |
| [roslaunch_commands.md](guides/roslaunch_commands.md) | 常用仿真启动、回放与实车指令速查 |
| [remember.md](guides/remember.md) | 实车当前阶段参数、EBS 占位状态与上车备忘录 |

---

## 6. 系统集成与部署 (Integration)

**目录**: [integration/](integration/)

| 文档 | 说明 |
|------|------|
| [vcu_mapping.md](integration/vcu_mapping.md) | VCU 控制模式映射、CAN 通信协议与接口定义 |
| [baseline/](integration/baseline/) | 历史基线配置与回退门控配置 |

---

## 7. 历史审计与验收报告 (Reports)

**目录**: [reports/](reports/)

| 文档 | 说明 |
|------|------|
| [remaining_work_audit_2026-03-06.md](reports/remaining_work_audit_2026-03-06.md) | 仓库真实剩余工作与缺失依赖统一审计清单（权威基准） |
| [HANDOFF.md](reports/HANDOFF.md) | 阶段性交接说明与各子系统状态基线 |
| [RESTRUCTURE_PLAN.md](reports/RESTRUCTURE_PLAN.md) | 早期项目规整化重构实施记录（P0-P3 里程碑） |
| [comprehensive_functional_review_report.md](reports/comprehensive_functional_review_report.md) | 全面功能审查与架构合规报告 |
| [audit_fixes_progress.md](reports/audit_fixes_progress.md) | 审计问题修复全量进度跟踪（已完结） |
| [code_fixes_summary_2026-04-06.md](reports/code_fixes_summary_2026-04-06.md) | 关键代码修复与稳定性加固总结 |
| [immediate_tasks_completion_report_2026-04-06.md](reports/immediate_tasks_completion_report_2026-04-06.md) | 阶段性重点任务完成报告 |
| [fusion_fallback_strategy_2026-04-26.md](reports/fusion_fallback_strategy_2026-04-26.md) | 多传感器融合降级策略决策备忘录 |
| [git-cleanup-analysis.md](reports/git-cleanup-analysis.md) | Git 仓库历史体积与资产清理分析 |

---

## 8. 专项演进方案 (Plans)

**目录**: [plans/](plans/)

- 存放各专项任务设计文档与实施计划（如视觉模块演进、延迟优化、主链融合适配等）。
- [completed/](plans/completed/) 归档已完成且经验证的实施方案。
- [未完成计划总结报告.md](plans/未完成计划总结报告.md) 对各项技术方案的当前实施度进行动态分级（已完成 / 部分落地 / 未实现）。
