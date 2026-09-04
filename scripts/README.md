# 脚本目录全景指南 (Scripts Navigation & Dictionary)

本目录包含 FSAE/FSAC 大学生方程式无人车（HUAT 翼驰车队）自动驾驶系统的全部自动化、验证、基准与维护脚本。

为保证工程结构清晰、职责分离并方便团队协作，所有脚本已按**单一职责原则**分类归档。

---

## 一、目录结构总览

```text
scripts/
├── ci/                # CI / CD 自动化门禁与静态契约验证
├── validation/        # 离线回放验证与各阶段里程碑验收矩阵
├── benchmarks/        # 性能基准测试、SOTA 对比与 A/B 实验
├── tools/             # 仿真辅助发布器、代理工具与数据分析
│   └── tests/         # 脚本工具专用自动化单元测试 (pytest)
├── dev/               # 开发者本地维护、覆盖率与紧急回滚工具
├── vehicle/           # 实车硬件在环自启脚本 (工控机硬件专用)
│   └── autoStartGkj/  # 车端开机自启入口与传感器启动链路
└── *.sh / *.py        # 向后兼容转发包装层 (Forwarding Wrappers)
```

---

## 二、子目录职责与脚本字典

### 1. `scripts/ci/` —— CI 自动化门禁与静态契约验证
此目录脚本供 GitHub Actions CI 与本地提交前（Pre-PR）快速检查调用：

| 脚本 | 语言 | 说明 |
|---|---|---|
| `check_topic_contracts.sh` | Bash | 验证全系统核心话题契约与命名一致性 |
| `check_perf_stats_contracts.sh` | Bash | 验证性能统计输出格式与契约一致性 |
| `check_deprecation_contracts.sh` | Bash | 验证废弃话题与兼容开关默认关闭状态 |
| `check_runtime_smoke.sh` | Bash | 运行时无崩溃冒烟验证 |
| `check_perception_regression.sh` | Bash | 感知回放回归测试与指标比对 |
| `check_perception_regression_mode.sh` | Bash | 感知各赛道模式（track/accel/skidpad）多维度回归 |
| `check_color_semantics_regression_mode.sh` | Bash | 颜色语义回归门禁验证 |
| `check_localization_fg_gate.py` | Python | 定位因子图（Factor Graph）硬指标硬门禁验证 |
| `validate_constraints.py` | Python | 物理动力学与系统约束规则静态校验 |
| `verify_geometry_config.sh` | Bash | 车辆几何与鲁棒性配置三层一致性校验 |

---

### 2. `scripts/validation/` —— 回放验证与里程碑验收矩阵
此目录脚本用于基于实际赛道 Rosbag 数据进行全流程离线回放与切主验证：

| 脚本 | 语言 | 说明 |
|---|---|---|
| `a3_replay_validation.sh` | Bash | A3 里程碑回放验证 |
| `b123_contract_validation.sh` | Bash | B1/B2/B3 契约回放验证 |
| `m5_validation.sh` | Bash | M5 统一规划算法回放验证 |
| `m5_final_validation.sh` | Bash | M5 最终交付级验证全矩阵 |
| `m5_45fold_validation.sh` | Bash | M5 45折交叉验证 |
| `run_adapter_replay_suite.sh` | Bash | 适配器回放测试套件批量执行 |
| `run_planning_replay_window.sh` | Bash | 规划调优时间窗口重放截取 |
| `run_pathlimits_v2_cutover_matrix.sh` | Bash | PathLimits V2 架构切主多任务矩阵回放 |
| `validate_localization_fg_mainline_replay.sh` | Bash | 定位因子图主线回放与状态一致性校验 |
| `validate_pathlimits_v2_cutover_replay.sh` | Bash | PathLimits V2 单场景切主回归验证 |
| `run_fg_validation.sh` | Bash | 因子图单次快速验证 |
| `run_fusion_sanity_test.sh` | Bash | 锥桶融合合理性与时序一致性测试 |

---

### 3. `scripts/benchmarks/` —— 性能基准与 A/B 实验
此目录脚本用于算法性能开销评估、耗时分析与对比实验：

| 脚本 | 语言 | 说明 |
|---|---|---|
| `benchmark_trackdrive.py` | Python | 高速循迹全链路性能基准压测与指标统计 |
| `run_benchmark_sota.sh` | Bash | SOTA 算法横向对比测试 |
| `run_experiment_matrix.py` | Python | 算法参数调优矩阵批量实验执行器 |
| `run_ab_test.sh` | Bash | A/B 方案切换对比运行器 |
| `m5_ab_test.py` | Python | M5 规划算法 A/B 效果对比分析器 |

---

### 4. `scripts/tools/` —— 仿真辅助发布器、代理工具与数据分析
此目录脚本提供开发调试时所需的辅助节点、Mock 工具与日志分析：

| 脚本 | 语言 | 说明 |
|---|---|---|
| `adapter_replay_publisher.py` | Python | 适配器回放发布节点 |
| `fake_vision_publisher.py` | Python | 视觉检测仿真/Mock 数据发布节点 |
| `perception_detection_proxies.py` | Python | 感知检测代理模块 |
| `analyze_adapter_replay.py` | Python | 适配器回放日志深度分析 |
| `analyze_fusion_sanity.py` | Python | 融合合理性数据分析与异常检测 |
| `run_perception_evaluation.py` | Python | 离线感知评测全套指标计算 |
| `run_replay_reports.py` | Python | 自动化生成回放结果报告 |
| `tests/test_run_replay_reports.py` | Python | 工具本身的自动化单元测试 (`pytest`) |

---

### 5. `scripts/dev/` —— 开发者维护与工程工具
此目录脚本供日常开发环境维护与应急操作：

| 脚本 | 语言 | 说明 |
|---|---|---|
| `generate_coverage.sh` | Bash | 本地执行全仓 C++/Python 覆盖率测试并生成 HTML 报告 |
| `freeze_perception_baseline.sh` | Bash | 将当前回放感知指标冻结为官方回归基线 |
| `emergency_rollback.sh` | Bash | 快速紧急回滚与故障排查辅助工具 |

---

### 6. `scripts/vehicle/` —— 实车部署与硬件开机自启
实车工控机环境专用（生产部署）：

- `vehicle/autoStartGkj/start.sh`: 实车自动化主入口（拉起 roscore、底盘通信、传感器驱动与监控）
- `vehicle/autoStartGkj/command`: 状态控制触发文件（写入 `2025` 触发循迹）
- `vehicle/autoStartGkj/sourceBash/`: 各硬件传感器与协议子脚本（`cameraDetect.sh`, `lidarDetect.sh`, `imuDetect.sh`, `interface.sh`, `racingNumCmd.sh`）

---

## 三、向后兼容保障机制

为避免修改目录结构破坏既有 CI/CD 流水线或开发者的习惯命令，根目录下保留了常用脚本的**兼容包装转发器**：
- 例如执行 `./scripts/check_topic_contracts.sh` 会自动转发给 `./scripts/ci/check_topic_contracts.sh`。
- 新增脚本与开发建议统一使用分类子目录。
