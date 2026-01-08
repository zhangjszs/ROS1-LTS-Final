---
name: pointcloud-perf-profiler
description: 点云管线性能计时（std::chrono）：统一打点、统计每帧耗时/均值/最大值/超时帧，并用 ROS 日志输出，目标 10Hz
---

# pointcloud-perf-profiler

## 目标
让用户快速知道：点云流程每个阶段耗时多少，哪里超时，是否能达到 10Hz。

## 约束（必须遵守）
- 用 `std::chrono`（不要引入复杂 profiler）
- 输出要包含：每帧总耗时、关键阶段耗时、平均/最大、超阈值计数
- 代码风格：尽量少侵入、便于开关（宏/参数）

## 推荐打点位置（示例）
1) 接收点云 callback 起点
2) PassThrough
3) VoxelGrid
4) Ground segmentation
5) Clustering
6) Post-processing（特征/筛选 cone）
7) Publish（markers/msgs）

## 输出模板（建议）
- 每 N 帧打印一次（比如 50 帧）：
  - avg_total_ms, max_total_ms
  - avg_stage_ms[...]
  - over_budget_count（比如 >100ms）

## 必须交付
- 一段可直接粘贴的 C++ 计时代码片段
- 如何开启/关闭（参数/宏）
- 如何验证（运行 N 秒观察日志）
