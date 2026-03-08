# Track Planning (high_speed_tracking) v2 调参设计

**背景 / 问题：**
- 目标场景主要在 **18m 以后**，误检多出现在左右两侧以及前方的“墙/护栏”类连续结构（会让规划输入的锥桶集合出现大量离群/结构性噪声）。
- 目前 v1 已显著改善：短路径占比下降、曲率尖峰下降、路径跳变率下降；但 `path_size` 均值明显变大，且 `max_path_jump` 可能被 partial/full/插值模式切换放大（不一定代表真实抖动）。

**v2 目标：**
- 在不引入大范围策略改动的前提下，让三角剖分/中点过滤对“连续结构误检”更不敏感，从源头减少离群中点进入搜索树。
- 同轮回放补齐观测：录制 `/localization/cone_map`，统计 **dist>=18m**（以及可选 **abs(y)<=1**）的锥桶置信度分布，为 v3 的 `min_cone_confidence` / “远距更高阈值”提供依据。

**v2 方案（配置优先，风险可控）：**
1) 更严格的三角形形状过滤
   - 提高 `min_triangle_angle`：过滤更“瘦”的三角形（常见于离群点/结构性噪声导致的退化三角形）。
2) 更严格的中点—外接圆心一致性过滤
   - 降低 `max_dist_circum_midPoint`：中点必须更贴近外接圆心，否则剔除（更能压制由异常几何关系产生的中点）。
3) 观测补齐（不立即调 `min_cone_confidence`）
   - 回放时同时录制 `/planning/pathlimits` 与 `/localization/cone_map`，离线统计 dist/y 窗口的置信度分布，决定 v3 是否引入阈值以及阈值大小。

**非目标：**
- v2 不调整速度上限/加速度上限等速度模型参数（避免与几何过滤耦合，先把输入质量稳定下来）。
- v2 不改 `the_mode_of_partial_path/the_mode_of_full_path`（避免影响评估指标的可比性）。

**验收 / 观测指标：**
- 规划 proxy（`/planning/pathlimits`）：`empty_path_rate`、`spike_jump_rate`、`max_curvature`、`replan_rate` 不回退；曲率尖峰继续下降则加分。
- cone_map proxy（`/localization/cone_map`）：dist>=18m（可选再分 abs(y)<=1）置信度分布与计数，用于下一轮阈值决策。

**更新（2026-02-20）：**
- cone_map 统计显示 dist>=18m 的置信度整体偏高（p10≈0.82~0.83），具备引入 `min_cone_confidence` 的空间；后续优先用 `min_cone_confidence=0.8` 做一次隔离验证。
- 分段阈值验证：`min_cone_confidence_far_side=0.85` 在本 bag 上过于激进（路径质量明显退化）；收敛到 `min_cone_confidence_far_side=0.83` 后，`replan_rate/spike_jump_rate/curvature_p99` 相比纯 `0.8` 有改善，且路径长度分布基本不变。
