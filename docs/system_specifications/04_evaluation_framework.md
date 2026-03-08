# 评估与回放体系设计

> 本文档定义了端到端系统的评估框架，用于回答"我改了算法，到底提升了什么"这一论文核心问题。

---

## 目录

1. [评估框架概述](#1-评估框架概述)
2. [Rosbag回放脚本设计](#2-rosbag回放脚本设计)
3. [核心指标定义](#3-核心指标定义)
4. [场景切片设计](#4-场景切片设计)
5. [报告模板](#5-报告模板)
6. [实验对比规范](#6-实验对比规范)

---

## 1. 评估框架概述

### 1.1 设计目标

| 目标 | 说明 |
|------|------|
| **可复现** | 相同bag、相同参数、相同结果 |
| **可对比** | 算法A vs 算法B 的定量差异 |
| **可追溯** | 每次实验的参数快照可查询 |
| **可切片** | 支持按场景/时间段分析 |

### 1.2 评估流程

```
┌─────────────────────────────────────────────────────────────────┐
│                        评估流程                                  │
├─────────────────────────────────────────────────────────────────┤
│  1. 准备阶段                                                     │
│     ├── 选择rosbag（标注ground truth）                           │
│     ├── 配置参数快照                                             │
│     ├── 设置随机种子                                             │
│     └── **前置检查: /use_sim_time=true**（见 Doc-02 §2.5）      │
├─────────────────────────────────────────────────────────────────┤
│  2. 执行阶段                                                     │
│     ├── rosbag回放                                              │
│     ├── 实时数据采集                                             │
│     └── 日志记录                                                 │
├─────────────────────────────────────────────────────────────────┤
│  3. 分析阶段                                                     │
│     ├── 指标计算                                                 │
│     ├── 场景切片                                                 │
│     └── 统计分析                                                 │
├─────────────────────────────────────────────────────────────────┤
│  4. 报告阶段                                                     │
│     ├── 生成报告                                                 │
│     ├── 可视化图表                                               │
│     └── 参数归档                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3 目录结构建议

```
evaluation/
├── bags/                    # rosbag存储
│   ├── trackdrive/
│   ├── skidpad/
│   └── acceleration/
├── configs/                 # 参数快照
│   ├── baseline/
│   └── experiments/
├── scripts/                 # 评估脚本
│   ├── run_evaluation.py
│   ├── compute_metrics.py
│   └── generate_report.py
├── results/                 # 评估结果
│   ├── exp_001/
│   ├── exp_002/
│   └── ...
└── reports/                 # 生成的报告
    └── comparison/
```

---

## 2. Rosbag回放脚本设计

### 2.1 回放脚本模板

```python
#!/usr/bin/env python3
"""
run_evaluation.py - 统一评估入口脚本
"""

import os
import subprocess
import yaml
import json
import hashlib
from datetime import datetime
from pathlib import Path

class EvaluationRunner:
    def __init__(self, config_path):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.exp_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.result_dir = Path(f"results/exp_{self.exp_id}")
        self.result_dir.mkdir(parents=True, exist_ok=True)
    
    def save_config_snapshot(self):
        """保存参数快照"""
        snapshot = {
            'experiment_id': self.exp_id,
            'timestamp': datetime.now().isoformat(),
            'config': self.config,
            'git_commit': self.get_git_commit(),
            'random_seed': self.config.get('random_seed', 42)
        }
        
        with open(self.result_dir / 'config_snapshot.json', 'w') as f:
            json.dump(snapshot, f, indent=2)
        
        return snapshot
    
    def get_git_commit(self):
        """获取当前git commit"""
        try:
            return subprocess.check_output(
                ['git', 'rev-parse', 'HEAD'],
                cwd='/home/kerwin/2025huat'
            ).decode().strip()
        except:
            return 'unknown'
    
    def set_random_seed(self):
        """设置随机种子"""
        seed = self.config.get('random_seed', 42)
        os.environ['PYTHONHASHSEED'] = str(seed)
        # 其他随机种子设置...
        return seed
    
    def play_rosbag(self):
        """回放rosbag"""
        bag_path = self.config['bag_path']
        start_time = self.config.get('start_time', 0)
        duration = self.config.get('duration', None)

        # MUST: 评估回放前置条件 - 检查 /use_sim_time（Doc-02 §2.5）
        try:
            sim_time = subprocess.check_output(
                ['rosparam', 'get', '/use_sim_time']
            ).decode().strip()
            if sim_time.lower() != 'true':
                raise RuntimeError("/use_sim_time is not true! Set it before evaluation.")
        except subprocess.CalledProcessError:
            raise RuntimeError("/use_sim_time not set! Run: rosparam set /use_sim_time true")

        cmd = [
            'rosbag', 'play',
            '--clock',
            '--rate', str(self.config.get('play_rate', 1.0)),
            '--start', str(start_time),
        ]
        
        if duration:
            cmd.extend(['--duration', str(duration)])
        
        cmd.append(bag_path)
        
        # 启动rosbag play
        self.bag_process = subprocess.Popen(cmd)
    
    def start_recording(self):
        """开始记录评估数据"""
        topics = self.config.get('record_topics', [
            '/perception/lidar_cluster/detections',
            '/localization/car_state',
            '/localization/cone_map',
            '/planning/pathlimits',
            '/control/command',
            '/control/diagnostics',
            '/tf',
            '/rosout_agg',   # A3: 文件侧通道兼容观测（可选）
        ])
        
        cmd = ['rosbag', 'record', '-O', str(self.result_dir / 'recorded.bag')] + topics
        self.record_process = subprocess.Popen(cmd)
    
    def run(self):
        """执行完整评估流程"""
        print(f"[{self.exp_id}] Starting evaluation...")
        
        # 1. 保存配置快照
        self.save_config_snapshot()
        
        # 2. 设置随机种子
        self.set_random_seed()
        
        # 3. 启动记录
        self.start_recording()
        
        # 4. 回放rosbag
        self.play_rosbag()
        
        # 5. 等待完成
        self.bag_process.wait()
        
        # 6. 停止记录
        self.record_process.terminate()
        
        print(f"[{self.exp_id}] Evaluation completed.")
        return self.result_dir

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True, help='Path to config yaml')
    args = parser.parse_args()
    
    runner = EvaluationRunner(args.config)
    runner.run()
```

### 2.2 配置文件模板

```yaml
# evaluation_config.yaml
experiment_name: "baseline_trackdrive_v1"
description: "Baseline evaluation on trackdrive scenario"

# 前置条件检查（MUST）
prerequisites:
  use_sim_time: true  # 回放前 MUST 设置 /use_sim_time=true（Doc-02 §2.5）

# Rosbag配置
bag_path: "/path/to/trackdrive_2024.bag"
start_time: 0
duration: null  # null表示播放完整bag
play_rate: 1.0  # 回放速率

# 随机种子（确保可复现）
random_seed: 42

# 记录的话题
record_topics:
  - "/perception/lidar_cluster/detections"
  - "/localization/car_state"
  - "/localization/cone_map"
  - "/planning/pathlimits"
  - "/control/command"
  - "/control/diagnostics"  # A3: mode_source/file fallback/external_stop_source
  - "/tf"
  - "/rosout_agg"    # A3可选：统计“文件打开失败/回退”日志
  - "/diagnostics"  # 需在 Doc-04 明确定义 schema，否则不可用

# 场景切片（最小可运行方案：仅基于时间 + health 触发）
scene_slices:
  - name: "first_lap"
    start_time: 0
    end_time: 60
  - name: "subsequent_laps"
    start_time: 60
    end_time: null
  - name: "anomaly_slices"
    detection: "auto"  # 由 Doc-02 health 阈值触发（max_diff>=100ms、TF timeout、端到端延迟超阈等）

# Ground Truth文件（如果有）
ground_truth: "/path/to/ground_truth.yaml"

# 参数覆盖（使用 canonical 参数名，见 Doc-01 附录B）
param_overrides:
  perception:
    min_confidence_to_add: 0.2  # canonical 名称（别名: confidence_threshold）
  localization:
    chi2_degrade: 15.0
  planning:
    speed_cap_safe: 11.11
    speed_cap_fast: 13.33
```

---

## 3. 核心指标定义

### 3.1 感知层指标

| 指标名称 | 定义 | 计算方法 | 单位 |
|----------|------|----------|------|
| **检测召回率** | 正确检测数 / 真实锥桶数 | `TP / (TP + FN)` | % |
| **检测精确率** | 正确检测数 / 检测总数 | `TP / (TP + FP)` | % |
| **颜色准确率** | 颜色正确数 / 总检测数 | 正确颜色数 / 总数 | % |
| **位置误差** | 检测位置与真实位置的距离 | `mean(||p_det - p_gt||)` | m |
| **平均置信度** | 所有检测的平均置信度 | `mean(confidence)` | - |
| **检测延迟** | 点云采集到检测输出的时间差 | `t_output - t_lidar` | ms |

### 3.2 定位层指标

| 指标名称 | 定义 | 计算方法 | 单位 |
|----------|------|----------|------|
| **横向误差** | 车辆中心到参考线的垂直距离 | `min_dist(car_pos, ref_line)` | m |
| **航向误差** | 估计航向与真实航向的差异 | `|θ_est - θ_gt|` | deg |
| **位置RMSE** | 位置误差的均方根 | `sqrt(mean((p - p_gt)²))` | m |
| **地图锥桶数** | 地图中累积的锥桶数量 | `len(cone_map)` | 个 |
| **匹配成功率** | 成功匹配的检测比例 | `matched / total_detections` | % |
| **异常状态占比** | 非TRACKING状态的时间比例 | `time_non_tracking / total_time` | % |

### 3.3 规划层指标

| 指标名称 | 定义 | 计算方法 | 单位 |
|----------|------|----------|------|
| **路径长度** | 规划路径的总长度 | `Σ||p_i - p_{i-1}||` | m |
| **平均曲率** | 路径的平均曲率 | `mean(|κ|)` | 1/m |
| **最大曲率** | 路径的最大曲率 | `max(|κ|)` | 1/m |
| **曲率波动** | 曲率的标准差 | `std(κ)` | 1/m |
| **规划失败率** | 无有效路径的帧数比例 | `failed_frames / total_frames` | % |
| **平均目标速度** | 目标速度的平均值 | `mean(target_speeds)` | m/s |
| **闭环检测延迟** | 第一圈结束到闭环检测的时间 | `t_loop_detected - t_lap_end` | s |

### 3.4 控制层指标

| 指标名称 | 定义 | 计算方法 | 单位 |
|----------|------|----------|------|
| **横向跟踪误差** | 实际位置与规划路径的距离 | `min_dist(car_pos, planned_path)` | m |
| **航向跟踪误差** | 实际航向与目标航向的差异 | `|θ_actual - θ_target|` | deg |
| **速度跟踪误差** | 实际速度与目标速度的差异 | `|v_actual - v_target|` | m/s |
| **转向平滑度** | 转向角变化率 | `mean(|d(steering)/dt|)` | 1/s |
| **控制延迟** | 规划输出到控制命令的时间 | `command.header.stamp - pathlimits.header.stamp`（见 Doc-01 §3.1 时间戳语义） | ms |

### 3.5 系统级指标

| 指标名称 | 定义 | 计算方法 | 单位 |
|----------|------|----------|------|
| **出界次数** | 车辆超出赛道边界的次数 | 计数 | 次 |
| **出界时间** | 车辆超出赛道边界的总时间 | 累计时间 | s |
| **完赛时间** | 完成比赛的总时间 | `t_finish - t_start` | s |
| **平均速度** | 全程平均速度 | `total_distance / total_time` | m/s |
| **最大速度** | 全程最高速度 | `max(v)` | m/s |
| **端到端延迟** | 点云采集到控制命令的时间 | `command.header.stamp - detections.header.stamp`（需时间戳传递链完整，见 Doc-02 §4.4；暂用 `bag_time - msg.header.stamp` 作为 age 替代） | ms |

### 3.6 健康/FMEA 覆盖指标

| 指标名称 | 定义 | 计算方法 | 单位 |
|----------|------|----------|------|
| **detections frame_id 合规率** | frame_id 为 "velodyne" 的帧占比 | 合规帧数 / 总帧数 | % |
| **数组长度一致率** | detections 各数组长度一致的帧占比 | 一致帧数 / 总帧数 | % |
| **颜色枚举合法率** | color_types 值在 0-5 范围内的占比 | 合法值数 / 总值数 | % |
| **颜色分布稳定性** | BLUE/YELLOW 比例的滑动窗口标准差 | `std(ratio_blue_yellow, window=50)` | - |
| **TF 成功率** | world→base_link TF 查询成功率 | tf_health.success_rate | % |
| **TF 延迟** | TF 查询平均/最大延迟 | tf_health.avg/max_latency_ms | ms |
| **时间同步差** | detections vs car_state 时间戳差分布 | mean/max/std/out_of_sync_count | ms |
| **pathlimits frame 合规率** | `pathlimits.header.frame_id=world` 的帧占比 | 合规帧数 / 总帧数 | % |
| **pathlimits stamp 非零率** | `pathlimits.stamp` 非零帧占比 | 非零帧数 / 总帧数 | % |
| **pathlimits 时间序有序率** | `pathlimits.stamp >= header.stamp` 的帧占比 | 有序帧数 / 总帧数 | % |
| **地图 id churn** | 单位时间新增锥桶 id 数 | new_ids_per_second | 个/s |
| **地图 confidence_scaled 合规率** | `HUAT_Cone.confidence∈[0,1000]` 的占比 | 合规值数 / 总值数 | % |
| **曲率违规统计** | |curvature| > 0.222 的违规点数 | count(violation_points) | 个 |
| **空路径率** | path[] 为空的帧占比 | empty_frames / total_frames | % |
| **steering/throttle 饱和比例** | 控制量达到边界值的时间占比 | saturated_time / total_time | % |

### 3.7 A3 兼容链路指标（文件侧通道降级验证）

| 指标名称 | 定义 | 计算方法 | 通过判据 |
|----------|------|----------|----------|
| **mode_source_param_ratio** | 控制模式来自参数链路的占比 | 统计 `/diagnostics` 中 `control_entry_health.mode_source=param` 的运行占比 | = 100%（标准赛项/评估回放） |
| **file_fallback_count** | 文件模式回退触发次数 | 统计 `/diagnostics` 中 `control_entry_health.file_mode_fallback_used=true` 次数 | = 0（标准赛项/评估回放） |
| **localization_entry_present_ratio** | 定位入口健康信号覆盖率 | 统计 `/diagnostics` 中 `localization_entry_health` 出现帧占比 | = 100%（标准赛项/评估回放） |
| **planning_entry_present_ratio** | 规划入口健康信号覆盖率 | 统计 `/diagnostics` 中 `planning_entry_health` 出现帧占比 | = 100%（标准赛项/评估回放） |
| **planning_internal_viz_disabled_ratio** | 规划内部可视化侧通道关闭占比 | 统计 `planning_entry_health.internal_viz_side_channel=false` 占比 | = 100%（默认主链路） |
| **file_open_error_rate** | 文件打开失败日志频率 | `/rosout_agg` 中匹配“文件打开失败”计数/分钟 | 可非零但不得引起命令中断 |
| **control_command_continuity** | 控制命令连续性 | `/control/command` 最大间隔/丢包率 | 不劣于基线（同场景对比） |

> 分栏校验：`/diagnostics_agg` **SHOULD** 同时出现 `ControlEntry/control_entry_health`、`LocalizationEntry/localization_entry_health`、`PlanningEntry/planning_entry_health`（用于 diagnostic_aggregator UI 分组验证）。

---

## 4. 场景切片设计

### 4.1 按比赛阶段切片

| 切片名称 | 定义 | 关注指标 |
|----------|------|----------|
| `first_lap` | 第一圈（建图阶段） | 检测召回率、地图构建质量、闭环检测 |
| `subsequent_laps` | 后续圈（高速阶段） | 定位精度、速度跟踪、出界风险 |
| `entry_phase` | 进入赛道阶段 | 初始定位、地图加载 |
| `exit_phase` | 退出赛道阶段 | 安全停车、状态切换 |

### 4.2 按锥桶状态切片

| 切片名称 | 定义 | 关注指标 |
|----------|------|----------|
| `cone_missing` | 有锥桶缺失的场景 | 缺锥补偿效果、定位稳定性 |
| `cone_miscolored` | 有锥桶颜色错误的场景 | 颜色容错、几何降级 |
| `cone_sparse` | 锥桶稀疏区域（间距>预期） | 检测召回率、插值效果 |
| `cone_dense` | 锥桶密集区域 | 检测精确率、误检率 |

### 4.3 按赛道特征切片

| 切片名称 | 定义 | 关注指标 |
|----------|------|----------|
| `straight_section` | 直道段 | 速度跟踪、横向误差 |
| `curve_section` | 弯道段 | 曲率跟踪、转向响应 |
| `sharp_turn` | 急转弯 | 最大曲率、速度降级 |
| `long_straight` | 长直道 | 最高速度、稳定性 |

### 4.4 按距离切片

| 切片名称 | 定义 | 关注指标 |
|----------|------|----------|
| `near_range` | 近距离检测（<10m） | 检测精确率、位置误差 |
| `mid_range` | 中距离检测（10-30m） | 检测召回率、置信度 |
| `far_range` | 远距离检测（>30m） | 检测召回率、漏检率 |

### 4.5 切片配置示例

```yaml
scene_slices:
  - name: "first_lap_straight"
    conditions:
      lap_mode: "MAP_BUILD_SAFE"       # 【未来实现项】：需新增可观测信号，当前无 topic 来源
      track_section: "straight"         # 【未来实现项】：需新增可观测信号，当前无 topic 来源
    time_range:
      start: 0
      end: 30

  - name: "fast_lap_curve"
    conditions:
      lap_mode: "FAST_LAP"             # 【未来实现项】
      track_section: "curve"            # 【未来实现项】
    time_range:
      start: 60
      end: null

  - name: "cone_missing_scenario"
    conditions:
      cone_count_left: "< 3"           # 可从 /localization/cone_map 的 type 字段统计
      cone_count_right: ">= 3"
    detection: "auto"  # 自动检测

  - name: "anomaly_window"
    conditions:
      health_trigger: true              # 由 Doc-02 health 阈值自动触发
    detection: "auto"
```

> **最小可运行方案**: 当前 `lap_mode`/`track_section` 缺少数据源定义（Doc-01 接口契约未包含这些字段）。建议先降级为"仅基于时间 + health 触发"的切片方案（见 §2.2 配置模板中的 `scene_slices`），待新增可观测信号后再启用条件切片。

---

## 5. 报告模板

### 5.1 单次实验报告

```markdown
# 实验报告: {experiment_name}

## 基本信息

| 项目 | 值 |
|------|-----|
| 实验ID | {exp_id} |
| 时间 | {timestamp} |
| Git Commit | {git_commit} |
| 随机种子 | {random_seed} |
| Rosbag | {bag_name} |

## 参数配置

### 感知参数
{perception_params_table}

### 定位参数
{localization_params_table}

### 规划参数
{planning_params_table}

### 控制参数
{control_params_table}

## 核心指标

### 健康状态一览（P0/P1）
| 检查项 | 状态 | 值 |
|--------|------|-----|
| TF 成功率 | {ok/warn/fail} | {value}% |
| 时间同步 max_diff | {ok/warn/fail} | {value}ms |
| 端到端延迟 | {ok/warn/fail} | {value}ms |
| detections frame_id 合规 | {ok/warn/fail} | {value}% |

### FMEA Top8 风险触发状态
| 风险项 | 是否触发 | 详情 |
|--------|----------|------|
| C04 紧急停车失效 | {yes/no} | {detail} |
| S01 TF链断裂 | {yes/no} | {detail} |
| S02 时间不同步 | {yes/no} | {detail} |
| L01 匹配丢失 | {yes/no} | {detail} |
| P03 锥桶颜色错误 | {yes/no} | {detail} |
| P01 锥桶漏检 | {yes/no} | {detail} |
| L03 地图漂移 | {yes/no} | {detail} |
| P02 锥桶错检 | {yes/no} | {detail} |

### 感知层
| 指标 | 值 | 基准 | 差异 |
|------|-----|------|------|
| 检测召回率 | {value}% | {baseline}% | {diff}% |
| 检测精确率 | {value}% | {baseline}% | {diff}% |
| 颜色准确率 | {value}% | {baseline}% | {diff}% |
| 平均位置误差 | {value}m | {baseline}m | {diff}m |

### 定位层
| 指标 | 值 | 基准 | 差异 |
|------|-----|------|------|
| 横向误差均值 | {value}m | {baseline}m | {diff}m |
| 航向误差均值 | {value}° | {baseline}° | {diff}° |
| 位置RMSE | {value}m | {baseline}m | {diff}m |

### 规划层
| 指标 | 值 | 基准 | 差异 |
|------|-----|------|------|
| 规划失败率 | {value}% | {baseline}% | {diff}% |
| 平均曲率 | {value}1/m | {baseline}1/m | {diff}1/m |

### 控制层
| 指标 | 值 | 基准 | 差异 |
|------|-----|------|------|
| 横向跟踪误差 | {value}m | {baseline}m | {diff}m |
| 速度跟踪误差 | {value}m/s | {baseline}m/s | {diff}m/s |

### 系统级
| 指标 | 值 | 基准 | 差异 |
|------|-----|------|------|
| 出界次数 | {value}次 | {baseline}次 | {diff}次 |
| 完赛时间 | {value}s | {baseline}s | {diff}s |
| 平均速度 | {value}m/s | {baseline}m/s | {diff}m/s |

## 场景切片分析

{scene_slice_tables}

## 可视化图表

### 关键图表清单（最小集）
1. 时间同步差分布（直方图 + 时序）
2. 端到端延迟时序图
3. detections 置信度/数量时序图
4. 曲率最大值时序图
5. steering/throttle 饱和比例时序图
6. 航向角（theta）连续性时序图
7. 地图锥桶数趋势图

### 轨迹对比图
{trajectory_comparison_image}

### 误差时序图
{error_timeseries_image}

### 速度曲线图
{speed_profile_image}

## 问题记录

{issues_list}

## 结论与建议

{conclusions}
```

### 5.2 对比实验报告

```markdown
# 对比实验报告: {comparison_name}

## 实验概述

| 项目 | 实验A | 实验B |
|------|-------|-------|
| 实验ID | {exp_a_id} | {exp_b_id} |
| 算法版本 | {version_a} | {version_b} |
| 主要差异 | {diff_description} |

## 指标对比

### 感知层对比
| 指标 | 实验A | 实验B | 改进幅度 | 显著性 |
|------|-------|-------|----------|--------|
| 检测召回率 | {a}% | {b}% | {improve}% | {p_value} |
| 检测精确率 | {a}% | {b}% | {improve}% | {p_value} |
| 颜色准确率 | {a}% | {b}% | {improve}% | {p_value} |

### 定位层对比
| 指标 | 实验A | 实验B | 改进幅度 | 显著性 |
|------|-------|-------|----------|--------|
| 横向误差 | {a}m | {b}m | {improve}% | {p_value} |
| 航向误差 | {a}° | {b}° | {improve}% | {p_value} |

### 系统级对比
| 指标 | 实验A | 实验B | 改进幅度 | 显著性 |
|------|-------|-------|----------|--------|
| 出界次数 | {a}次 | {b}次 | {improve}% | - |
| 完赛时间 | {a}s | {b}s | {improve}% | {p_value} |

## 场景切片对比

{scene_comparison_tables}

## 结论

{conclusions}
```

---

## 6. 实验对比规范

### 6.1 对照组设置

| 实验类型 | 对照组 | 实验组 | 说明 |
|----------|--------|--------|------|
| 算法改进 | 基线算法 | 改进算法 | 仅改变目标算法 |
| 参数调优 | 默认参数 | 调优参数 | 仅改变目标参数 |
| 消融实验 | 完整系统 | 移除某模块 | 验证模块贡献 |

### 6.2 统计显著性检验

```python
from scipy import stats

def compute_significance(baseline_values, experiment_values, alpha=0.05):
    """
    计算两组数据的统计显著性
    
    Returns:
        p_value: p值
        is_significant: 是否显著
        effect_size: 效应量 (Cohen's d)
    """
    # t检验
    t_stat, p_value = stats.ttest_ind(baseline_values, experiment_values)
    
    # 效应量
    pooled_std = np.sqrt(
        (np.std(baseline_values)**2 + np.std(experiment_values)**2) / 2
    )
    effect_size = (np.mean(experiment_values) - np.mean(baseline_values)) / pooled_std
    
    is_significant = p_value < alpha
    
    return {
        'p_value': p_value,
        'is_significant': is_significant,
        'effect_size': effect_size
    }
```

### 6.3 实验记录规范

每次实验必须记录：

1. **参数快照**: 完整的参数配置文件（使用 canonical 参数名，见 Doc-01 附录B）
2. **Git信息**: 当前commit hash
3. **随机种子**: 用于复现
4. **Rosbag信息**: 使用的bag文件名和时间范围
5. **环境信息**: ROS版本、系统版本
6. **结果数据**: 原始指标数据
7. **前置条件**: `/use_sim_time` 状态确认
8. **健康状态**: P0/P1 health 一览（TF/time/e2e）
9. **FMEA 触发**: Top8 风险项是否触发

### 6.4 命名规范

```
实验命名: {algorithm}_{scenario}_{version}_{date}

示例:
- baseline_trackdrive_v1_20240101
- improved_perception_trackdrive_v2_20240115
- ablation_no_color_skidpad_v1_20240201
```

---

## 附录：指标计算脚本

```python
#!/usr/bin/env python3
"""
compute_metrics.py - 指标计算脚本
"""

import rosbag
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Tuple

@dataclass
class DetectionMetrics:
    recall: float
    precision: float
    color_accuracy: float
    position_error_mean: float
    position_error_std: float
    confidence_mean: float

@dataclass
class LocalizationMetrics:
    lateral_error_mean: float
    lateral_error_std: float
    heading_error_mean: float
    heading_error_std: float
    position_rmse: float

@dataclass
class PlanningMetrics:
    path_length: float
    curvature_mean: float
    curvature_max: float
    curvature_std: float
    failure_rate: float

@dataclass
class ControlMetrics:
    lateral_tracking_error_mean: float
    heading_tracking_error_mean: float
    speed_tracking_error_mean: float
    steering_smoothness: float

@dataclass
class SystemMetrics:
    boundary_violation_count: int
    boundary_violation_time: float
    completion_time: float
    average_speed: float
    max_speed: float
    end_to_end_latency: float


def compute_detection_metrics(bag_path: str, ground_truth: Dict) -> DetectionMetrics:
    """计算感知层指标"""
    bag = rosbag.Bag(bag_path)
    
    detections = []
    for topic, msg, t in bag.read_messages(topics=['/perception/lidar_cluster/detections']):
        detections.append(parse_detection(msg))
    
    # 计算召回率、精确率等
    tp, fp, fn = match_detections(detections, ground_truth)
    
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0
    
    # 计算位置误差
    position_errors = compute_position_errors(detections, ground_truth)
    
    return DetectionMetrics(
        recall=recall,
        precision=precision,
        color_accuracy=compute_color_accuracy(detections, ground_truth),
        position_error_mean=np.mean(position_errors),
        position_error_std=np.std(position_errors),
        confidence_mean=np.mean([d.confidence for d in detections])
    )


def compute_localization_metrics(bag_path: str, reference_line: np.ndarray) -> LocalizationMetrics:
    """计算定位层指标"""
    bag = rosbag.Bag(bag_path)
    
    positions = []
    headings = []
    for topic, msg, t in bag.read_messages(topics=['/localization/car_state']):
        positions.append([msg.car_state.x, msg.car_state.y])
        headings.append(msg.car_state.theta)
    
    positions = np.array(positions)
    headings = np.array(headings)
    
    # 计算横向误差
    lateral_errors = compute_lateral_errors(positions, reference_line)
    
    # 计算航向误差
    heading_errors = compute_heading_errors(headings, reference_line)
    
    return LocalizationMetrics(
        lateral_error_mean=np.mean(lateral_errors),
        lateral_error_std=np.std(lateral_errors),
        heading_error_mean=np.rad2deg(np.mean(heading_errors)),
        heading_error_std=np.rad2deg(np.std(heading_errors)),
        position_rmse=np.sqrt(np.mean(lateral_errors**2))
    )


def compute_planning_metrics(bag_path: str) -> PlanningMetrics:
    """计算规划层指标"""
    bag = rosbag.Bag(bag_path)
    
    paths = []
    curvatures = []
    failure_count = 0
    total_count = 0
    
    for topic, msg, t in bag.read_messages(topics=['/planning/pathlimits']):
        total_count += 1
        if len(msg.path) == 0:
            failure_count += 1
            continue
        
        path = np.array([[p.x, p.y] for p in msg.path])
        paths.append(path)
        curvatures.extend(msg.curvatures)
    
    curvatures = np.abs(np.array(curvatures))
    
    return PlanningMetrics(
        path_length=np.sum([compute_path_length(p) for p in paths]),
        curvature_mean=np.mean(curvatures),
        curvature_max=np.max(curvatures),
        curvature_std=np.std(curvatures),
        failure_rate=failure_count / total_count if total_count > 0 else 0
    )


def compute_control_metrics(bag_path: str) -> ControlMetrics:
    """计算控制层指标"""
    bag = rosbag.Bag(bag_path)
    
    # 读取路径和状态
    # 计算跟踪误差
    # ...
    
    return ControlMetrics(
        lateral_tracking_error_mean=0.0,  # 实际计算
        heading_tracking_error_mean=0.0,
        speed_tracking_error_mean=0.0,
        steering_smoothness=0.0
    )


def compute_system_metrics(bag_path: str, track_boundary: np.ndarray) -> SystemMetrics:
    """计算系统级指标"""
    bag = rosbag.Bag(bag_path)
    
    positions = []
    speeds = []
    timestamps = []
    
    for topic, msg, t in bag.read_messages(topics=['/localization/car_state']):
        positions.append([msg.car_state.x, msg.car_state.y])
        speeds.append(msg.V)
        timestamps.append(t.to_sec())
    
    positions = np.array(positions)
    speeds = np.array(speeds)
    timestamps = np.array(timestamps)
    
    # 计算出界次数
    violations = check_boundary_violations(positions, track_boundary)
    
    return SystemMetrics(
        boundary_violation_count=np.sum(violations),
        boundary_violation_time=np.sum(violations) * (timestamps[1] - timestamps[0]),
        completion_time=timestamps[-1] - timestamps[0],
        average_speed=np.mean(speeds),
        max_speed=np.max(speeds),
        end_to_end_latency=0.0  # 待实现：计算公式 = command.header.stamp - detections.header.stamp
        # 需要时间戳传递链完整（Doc-01 §3.1/§5.2, Doc-02 §4.4）
        # 暂用 bag_time - msg.header.stamp 作为 age 替代
    )


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--gt', required=False)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    
    # 计算所有指标
    metrics = {
        'detection': compute_detection_metrics(args.bag, {}),
        'localization': compute_localization_metrics(args.bag, np.array([])),
        'planning': compute_planning_metrics(args.bag),
        'control': compute_control_metrics(args.bag),
        'system': compute_system_metrics(args.bag, np.array([]))
    }
    
    # 保存结果
    import json
    with open(args.output, 'w') as f:
        json.dump({k: v.__dict__ for k, v in metrics.items()}, f, indent=2)
```
