# 开发交接文档

> 交接日期: 2026-02-16 | 涉及模块: Perception LiDAR + 系统级重构

---

## 一、已完成工作总览

本轮开发包含两条并行线：**Perception LiDAR 审计修复** 和 **系统级规整化重构**。

### 1.1 Perception LiDAR 审计 (G1-G16, 共 15 项已落地)

对 `perception_core` + `perception_ros` 做了 18 项 gap 审计，已实现 15 项：

| Gap | 内容 | 改动位置 |
|-----|------|----------|
| G1 | 时间戳三级校验（零值/回退/漂移） | `lidar_cluster_ros.cpp:validateStamp()` |
| G2 | frame_id 校验 + 告警 | `lidar_cluster_ros.cpp:validateStamp()` |
| G3 | ConeDetections 并行数组长度不变量检查 | `lidar_cluster_ros.cpp:publishOutput()` |
| G5+G8 | DiagnosticsHelper 框架 + 三级健康状态机 (NORMAL/LOW/NO_DETECTION) | `lidar_cluster_ros.cpp:updateHealthState()` + `publishDiagnostics()` |
| G6 | Debug marker 可视化 (bbox + 置信度着色) | `lidar_cluster_ros.cpp:publishDebugMarkers()` |
| G7 | PerfStats 滚动窗口 p50/p95/max 写入诊断 | `lidar_cluster_ros.cpp:publishDiagnostics()` 末尾 |
| G9 | 分距离段 (near/mid/far) 置信度统计 | `lidar_cluster_ros.cpp:publishDiagnostics()` |
| G10 | 自车运动补偿 (EgoMotion → tracker predict) | `lidar_cluster_ros.cpp:runOnceLocked()` + `distortion_compensator_v2` |
| G11 | 输出按角度排序，保证帧间稳定 | `lidar_cluster_ros.cpp:publishOutput()` |
| G12 | Tracker ID 透传到 ROS 消息 | `ConeDetection.track_id` → `HUAT_ConeDetections.track_ids` |
| G13 | 模式预设覆盖模块开关 (tracker/topology/obstacle_height) | `lidar_cluster_ros.cpp:applyModePreset()` |
| G14 | YAML 参数单位标注 (m/s/帧/个/[0-1]) | `lidar_base.yaml` 全文 |
| G15 | evaluate_perception_metrics.py (10 项代理指标) | `perf_reports/scripts/evaluate_perception_metrics.py` |
| G16 | GT 评估模式 (precision/recall/F1/RMSE) | 同上，`--gt` 参数 |

### 1.2 系统级规整化重构 (P0-P3, 全部完成)

| 阶段 | Commit | 内容 |
|------|--------|------|
| P0 | `bb05fb0` | Mission launch 去重 (70行×4 → 36行×4)、车辆几何参数统一到 `vehicle_geometry.yaml`、`.yml→.yaml` |
| P1 | `b1dab0d` | 创建 `fsd_common` 包、清理 legacy topic、规范化话题名 (insMsg→sensors/ins)、补齐 planning 配置层级 |
| P2 | `b46a847` | PerfStats 模板基类统一 3 包、ins_bridge 迁移到 vehicle_interface_ros、删除 ros_vehicle_interface、param_snapshot.py |
| P3 | `72cc40a` | geometry_utils/time_utils/contract_utils 迁移到 fsd_common、删除死代码 Time.hpp、修复 IMU 话题 |

**关键产出**:
- `fsd_common` 包: 9 个共享 hpp (topic_contract, diagnostics_helper, param_utils, perf_stats_base, geometry_utils, time_utils, contract_utils, control_mode, perf_stats_skeleton)
- `param_snapshot.py`: 参数快照 + diff 工具
- 4 个 CI 门禁脚本: `check_topic_contracts.sh`, `check_deprecation_contracts.sh`, `check_perf_stats_contracts.sh`, `check_runtime_smoke.sh`

---

## 二、当前代码状态

### 2.1 构建状态

```
catkin build → 19/19 packages succeeded, 0 warnings
```

### 2.2 Perception 模块文件清单

**perception_core** (纯算法, 无 ROS 依赖):
```
include/perception_core/
├── lidar_cluster_core.hpp      # 主流水线 + 配置结构体 (617行)
├── fast_ground_segmentation.hpp # FGS 地面分割
├── patchworkpp.hpp              # Patchwork++ 地面分割
├── confidence_scorer.hpp        # 多维置信度评分
├── cluster_feature_extractor.hpp # 聚类特征提取
├── cone_model_fitter.hpp        # 锥桶几何拟合
├── cone_tracker.hpp             # 卡尔曼跟踪器
├── topology_repair.hpp          # 拓扑修复 (插值/离群剔除)
├── fast_euclidean_clustering.hpp # FEC 快速聚类
├── distortion_adjust_v2.hpp     # 畸变补偿
├── point_cloud_pool.hpp         # 内存池 (实验性)
├── cluster_features.hpp         # 特征结构体
└── imu_data.hpp                 # IMU 数据结构
src/ → 14 个 .cpp 实现文件
```

**perception_ros** (ROS 适配层):
```
src/lidar_cluster_ros.cpp        # 主 ROS 节点 (1512行)
src/distortion_compensator_v2.cpp # IMU 畸变补偿 V2
include/perception_ros/
├── lidar_cluster_ros.hpp        # 节点头文件
└── perf_stats.hpp               # PerfStats (继承 fsd_common 基类)
config/
├── lidar_base.yaml              # 基础配置 (641行, 含完整单位标注)
├── lidar_track.yaml             # track 模式覆盖
├── lidar_accel.yaml             # accel 模式覆盖
└── lidar_skidpad.yaml           # skidpad 模式覆盖
test/
├── test_perf_stats.cpp          # PerfStats 单元测试
└── confidence.cpp               # 置信度测试
```

### 2.3 处理流水线 (lidar_cluster_core::Process)

```
输入点云 → ① ROI裁剪 → ② 地面分割(FGS/RANSAC/Patchwork++)
         → ③ 降采样+SOR → ④ 多帧累积 → ⑤ 聚类(Euclidean/DBSCAN/FEC)
         → ⑤b 近距去重 → ⑥ 卡尔曼跟踪 → ⑦ 拓扑修复 → 输出
```

### 2.4 配置加载层级

```
1. lidar_base.yaml          ← 全局默认值 + mode_presets
2. lidar_<mode>.yaml        ← track/accel/skidpad 覆盖
3. mission overlay          ← fsd_launch/config/missions/
4. vehicle overlay          ← fsd_launch/config/vehicles/A13/perception.yaml
5. local overlay            ← *.local.yaml (gitignore)
```

`mode_presets` 机制: `lidar_base.yaml` 中按 `roi.mode` 自动覆盖 confidence ramp、adaptive_y、cluster 分段、tracker/topology/obstacle_height 开关。

---

## 三、未完成项 (Perception 审计 G4/G17/G18)

原始审计在对话中产生，未保存为文件。以下 3 项未实现：

### G4: 输入边界防御 (建议优先级: 中)

**问题**: `pointCallback` 对空点云、全 NaN 点云、点数异常暴增等边界情况缺少显式防御。

**建议实现**:
- 在 `pointCallback` 入口检查 `cloud->empty()`
- 检查点云尺寸是否超过合理上限 (如 >500k 点发出告警)
- 对 NaN/Inf 点做快速扫描或依赖 PCL `removeNaNFromPointCloud`
- 改动范围: `lidar_cluster_ros.cpp:pointCallback()`, 约 10-15 行

### G17: perception_core 单元测试补全 (建议优先级: 中)

**问题**: 14 个 core 算法文件仅有 2 个测试 (`test_ground_segmentation_perf.cpp`, `test_point_cloud_pool.cpp`)。关键模块缺少测试:

**建议补全顺序**:
1. `confidence_scorer` — 输入已知特征，验证评分范围和单调性
2. `cone_tracker` — 模拟多帧检测序列，验证关联/确认/删除逻辑
3. `topology_repair` — 构造缺口场景，验证插值和离群剔除
4. `fast_ground_segmentation` — 构造平面+障碍物点云，验证分割正确性
5. `cluster_feature_extractor` — 已知几何体验证特征提取

### G18: 端到端 bag 回放回归测试 (建议优先级: 低)

**问题**: 缺少自动化的 bag 回放 → 指标对比 → pass/fail 判定流程。

**建议实现**:
- 基于 `evaluate_perception_metrics.py` 构建
- 选取 1-2 个标准 bag 作为 baseline
- 脚本: 回放 bag → 录制输出 → 计算指标 → 与 baseline 对比 → 超阈值则 fail
- 可集成到 `scripts/check_perception_regression.sh`
- 依赖: 需要一个可用的标准测试 bag 文件

---

## 四、关键架构决策记录

| 决策 | 理由 |
|------|------|
| 健康状态机用连续帧计数而非滑动窗口 | 简单、零分配、延迟可控；3 帧恢复防止抖动 |
| GT 匹配用贪心最近邻而非 Hungarian | 锥桶数 <500，O(n*m) 足够；避免引入额外依赖 |
| PerfStats 用模板基类而非继承虚函数 | 零运行时开销；各包 PerfSample 字段不同，模板更灵活 |
| mode_presets 在 YAML 中定义而非 C++ 硬编码 | 可热调参、不需重编译；launch 层可覆盖 |
| fsd_common 只含 header-only 库 | 避免链接依赖；所有 *_ros 包只需 `find_package` |
| ins 包保留为 msg-only | 旧 bag 文件中录制了 `ins/ASENSING` 类型，删除会导致无法回放 |
| Debug markers 默认关闭 | 避免生产环境性能开销；`debug/publish_markers:=true` 按需开启 |

---

## 五、验证方法

### 5.1 编译验证
```bash
catkin clean -y && catkin build
# 期望: 19/19 succeeded, 0 warnings
```

### 5.2 单元测试
```bash
catkin run_tests perception_ros
catkin run_tests perception_core
catkin run_tests localization_ros
```

### 5.3 CI 门禁脚本
```bash
scripts/check_topic_contracts.sh        # topic 契约一致性
scripts/check_deprecation_contracts.sh  # 废弃路径默认关闭
scripts/check_perf_stats_contracts.sh   # PerfStats 回归
```

### 5.4 Bag 回放验证
```bash
# 基本功能验证
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag

# 检查诊断输出
rostopic echo /perception/diagnostics

# 检查检测输出
rostopic hz /perception/lidar_cluster/detections
rostopic echo /perception/lidar_cluster/detections --noarr | head -20

# 参数快照
rosrun fsd_launch param_snapshot.py
```

### 5.5 Perception 离线评估
```bash
# 代理指标 (无需 GT)
python3 perf_reports/scripts/evaluate_perception_metrics.py <bag_file>

# 带 GT 评估 (需要 GT CSV)
python3 perf_reports/scripts/evaluate_perception_metrics.py <bag_file> \
    --gt perf_reports/gt_example.csv --gt-threshold 1.0
```

---

## 六、后续开发建议

### 短期 (赛前调参)
1. 用实车 bag 跑 `evaluate_perception_metrics.py`，建立各赛道模式的 baseline 指标
2. 根据 diagnostics 输出的 `conf_near/conf_mid/conf_far` 调整置信度 ramp 参数
3. 用 `debug/publish_markers:=true` 在 RViz 中可视化检测质量

### 中期 (稳定性)
4. 实现 G4 输入边界防御 (约 1 小时工作量)
5. 补全 G17 核心模块单元测试 (confidence_scorer 和 cone_tracker 优先)
6. 制作 1-2 个标准测试 bag 的 GT CSV，启用精度评估

### 长期 (能力提升)
7. 实现 G18 自动化回归测试流水线
8. 考虑相机融合路径 (color_types 字段已预留 BLUE/YELLOW/RED)
9. 多帧累积 (`cluster/multi_frame`) 目前默认关闭，可在远距离检测不足时启用评估

