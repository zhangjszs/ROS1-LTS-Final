# HUAT 自动驾驶系统文档

> 本目录包含 FSAE/FSAC 大学生方程式赛车自动驾驶系统的全部技术文档。

---

## 目录结构

```
docs/
├── system_specifications/    # 系统级规格文档
├── modules/                  # 模块文档
│   ├── perception/           # 感知模块
│   ├── localization/         # 定位模块
│   └── planning/             # 规划模块
├── integration/              # 集成与部署文档
└── plans/                    # 计划、设计和阶段性总结
```

---

## 1. 系统级规格文档

**目录**: [system_specifications/](system_specifications/)

| 文档 | 说明 |
|------|------|
| [01_interface_contract.md](system_specifications/01_interface_contract.md) | 端到端接口契约：LiDAR→Location→Planning→Control 全链路消息定义 |
| [02_tf_time_consistency.md](system_specifications/02_tf_time_consistency.md) | TF/时间/单位一致性检查清单与自检输出定义 |
| [03_failure_mode_library.md](system_specifications/03_failure_mode_library.md) | 端到端失败模式库（FMEA）：20条失败模式与保护策略 |
| [04_evaluation_framework.md](system_specifications/04_evaluation_framework.md) | 评估与回放体系设计：指标定义、场景切片、报告模板 |
| [../RESTRUCTURE_PLAN.md](../RESTRUCTURE_PLAN.md) | 规整化重构规划与实施记录（已完成，含 P0-P3） |
| [06_topic_frame_param_contract_v1_draft.md](system_specifications/06_topic_frame_param_contract_v1_draft.md) | 统一命名草案：Topic/Frame/Param 的 canonical 与兼容映射 |

**推荐阅读顺序**: 01 → 02 → 03 → 04

---

## 2. 模块文档

### 2.1 感知模块

**目录**: [modules/perception/](modules/perception/)

| 文档 | 说明 |
|------|------|
| [lidar_pipeline.md](modules/perception/lidar_pipeline.md) | LiDAR 感知处理流程：点云处理、地面分割、聚类、置信度评分 |

### 2.2 定位模块

**目录**: [modules/localization/](modules/localization/)

| 文档 | 说明 |
|------|------|
| [LOCALIZATION_M1_INTERFACE_FREEZE.md](modules/localization/LOCALIZATION_M1_INTERFACE_FREEZE.md) | 接口冻结规范 |
| [LOCALIZATION_M2_CTF_GRAPH_DESIGN.md](modules/localization/LOCALIZATION_M2_CTF_GRAPH_DESIGN.md) | CTF 因子图设计 |
| [LOCALIZATION_M3_LOOP_CLOSURE_RELOC.md](modules/localization/LOCALIZATION_M3_LOOP_CLOSURE_RELOC.md) | 回环检测与重定位 |
| [LOCALIZATION_M4_VALIDATION_PLAN.md](modules/localization/LOCALIZATION_M4_VALIDATION_PLAN.md) | 验证计划 |
| [LOCALIZATION_M5_RISK_CHECKLIST.md](modules/localization/LOCALIZATION_M5_RISK_CHECKLIST.md) | 风险检查清单 |
| [LOCALIZATION_INNOVATION1_COLOR_TOPOLOGY.md](modules/localization/LOCALIZATION_INNOVATION1_COLOR_TOPOLOGY.md) | 创新点1：颜色-拓扑软关联 |
| [LOCALIZATION_INNOVATION2_SKIDPAD_CIRCLE.md](modules/localization/LOCALIZATION_INNOVATION2_SKIDPAD_CIRCLE.md) | 创新点2：八字绕环圆形约束 |
| [LOCALIZATION_INNOVATION3_RELOCALIZATION.md](modules/localization/LOCALIZATION_INNOVATION3_RELOCALIZATION.md) | 创新点3：多阶段重定位 |

**推荐阅读顺序**: M1 → M2 → M3 → M4 → M5 → Innovation系列

### 2.3 规划模块

**目录**: [modules/planning/](modules/planning/)

| 文档 | 说明 |
|------|------|
| [PLANNING_M1_INTERFACE_SPEC.md](modules/planning/PLANNING_M1_INTERFACE_SPEC.md) | 统一接口规范（V1兼容 + V2字段契约） |
| [PLANNING_M2_AUTOCROSS_DESIGN.md](modules/planning/PLANNING_M2_AUTOCROSS_DESIGN.md) | 高速循迹设计（第一圈稳、后续圈快） |
| [PLANNING_M3_LINEAR_ACCEL_DESIGN.md](modules/planning/PLANNING_M3_LINEAR_ACCEL_DESIGN.md) | 直线加速设计（75m加速 + 100m制动） |
| [PLANNING_M4_SKIDPAD_DESIGN.md](modules/planning/PLANNING_M4_SKIDPAD_DESIGN.md) | 八字绕环设计（双圆拟合 + 圈次状态机） |
| [PLANNING_M5_VALIDATION_AND_MIGRATION_PLAN.md](modules/planning/PLANNING_M5_VALIDATION_AND_MIGRATION_PLAN.md) | 验证、迁移与回退策略 |
| [SKIDPAD_TUNING_TEMPLATE.md](modules/planning/SKIDPAD_TUNING_TEMPLATE.md) | 八字绕环参数整定模板 |

**推荐阅读顺序**: M1 → M2 → M3/M4 → M5

---

## 3. 集成与部署文档

**目录**: [integration/](integration/)

| 文档 | 说明 |
|------|------|
| [vcu_mapping.md](integration/vcu_mapping.md) | VCU 控制模式映射与接口定义 |
| [baseline/](integration/baseline/) | 基线配置与回退门控 |
| [remaining_work_audit_2026-03-06.md](remaining_work_audit_2026-03-06.md) | 当前仓库剩余工作、未实现功能和缺失依赖的统一审计清单 |

---

## 4. 快速导航

### 按任务查找

| 任务 | 推荐文档 |
|------|----------|
| 理解系统架构 | [01_interface_contract.md](system_specifications/01_interface_contract.md) |
| 排查TF/时间问题 | [02_tf_time_consistency.md](system_specifications/02_tf_time_consistency.md) |
| 分析失败原因 | [03_failure_mode_library.md](system_specifications/03_failure_mode_library.md) |
| 设计对比实验 | [04_evaluation_framework.md](system_specifications/04_evaluation_framework.md) |
| 调参感知模块 | [lidar_pipeline.md](modules/perception/lidar_pipeline.md) |
| 调参定位模块 | [LOCALIZATION_M5_RISK_CHECKLIST.md](modules/localization/LOCALIZATION_M5_RISK_CHECKLIST.md) |
| 调参规划模块 | [SKIDPAD_TUNING_TEMPLATE.md](modules/planning/SKIDPAD_TUNING_TEMPLATE.md) |
| 理解创新点 | [LOCALIZATION_INNOVATION*.md](modules/localization/) |
| 查看当前真实剩余事项 | [remaining_work_audit_2026-03-06.md](remaining_work_audit_2026-03-06.md) |

### 按角色查找

| 角色 | 推荐文档 |
|------|----------|
| 新成员 | 系统级规格文档 (01-04) |
| 感知工程师 | [modules/perception/](modules/perception/) |
| 定位工程师 | [modules/localization/](modules/localization/) |
| 规划工程师 | [modules/planning/](modules/planning/) |
| 系统集成 | [integration/](integration/) |
| 论文写作 | 系统级规格文档 + Innovation系列 |

---

## 5. 文档约定

### 命名规范

- `M1/M2/...`: 里程碑文档，按开发顺序编号
- `INNOVATION*`: 创新点文档，用于论文写作
- `*_CHECKLIST`: 检查清单，用于验证和调试

### 状态标识

- ✅ 已完成
- 🚧 进行中
- ⏳ 待开始
- ❌ 已废弃

---

## 6. 更新日志

| 日期 | 更新内容 |
|------|----------|
| 2026-03-06 | 修正文档状态漂移，新增剩余工作统一审计文档 |
| 2026-02-12 | 创建系统级规格文档，重组目录结构 |
| 2026-02-10 | 添加定位模块创新点文档 |
| 2026-02-09 | 添加 LiDAR 感知流程文档 |
| 2026-02-08 | 添加规划模块里程碑文档 |
