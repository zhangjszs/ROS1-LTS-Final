# 性能报告系统

本系统用于自动收集、分析和报告ROS节点的性能数据，并与Git版本关联以便长期追踪。

## 目录结构

```
perf_reports/
├── data/              # 性能数据存储（JSON格式）
├── reports/           # 生成的Markdown报告和图表
└── scripts/           # 自动化脚本
    ├── collect_perf_data.py      # 收集性能数据
    ├── generate_report.py        # 生成Markdown报告
    ├── generate_charts.py       # 生成性能图表
    └── run_perf_analysis.sh     # 一键运行所有步骤
```

## 功能特性

### 1. 性能数据收集
- 自动从ROS日志中提取性能统计信息
- 关联Git commit ID和分支信息
- 收集系统环境信息（OS、CPU、Python版本等）
- 支持多个节点的性能数据收集

### 2. Markdown报告生成
- **单版本报告**：详细展示单个代码版本的性能指标
- **对比报告**：比较不同版本间的性能变化
- 包含测试时间、环境配置、关键指标
- 自动计算性能变化百分比
- 标记显著变化（>10%为⚠️，提升为✅）

### 3. 性能图表生成
- 处理时间分解图
- 时间分布饼图
- 数据量统计图
- 版本对比柱状图
- 性能趋势线图
- P95/P99对比图

## 快速开始

### 安装依赖

```bash
pip3 install -r requirements.txt
```

### 统一CLI工具（推荐）

```bash
# 完整工作流
python3 scripts/perf_tool.py full --note "变更说明" --scenario "测试场景"

# 或使用快捷脚本
./quick_report.sh --note "变更说明"
```

### 单独操作

```bash
# 收集数据
python3 scripts/perf_tool.py collect --note "变更说明"

# 生成报告
python3 scripts/perf_tool.py report

# 生成图表
python3 scripts/perf_tool.py chart

# 性能分析
python3 scripts/perf_tool.py analyze
```

### 传统方式

```bash
# 从默认日志文件收集
python3 perf_reports/scripts/collect_perf_data.py

# 指定日志文件
python3 perf_reports/scripts/collect_perf_data.py --log-file ~/.ros/log/latest/rosout.log

# 指定输出目录
python3 perf_reports/scripts/collect_perf_data.py --output-dir /path/to/output

# 添加变更说明/场景/标签
python3 perf_reports/scripts/collect_perf_data.py --note "优化地面分割缓存" --scenario "high_speed_tracking rosbag" --tags ground,cache
```

**输出**：`perf_reports/data/perf_data_YYYYMMDD_HHMMSS.json`

#### 步骤2：生成Markdown报告

```bash
# 生成最新版本的单版本报告
python3 perf_reports/scripts/generate_report.py

# 生成版本对比报告
python3 perf_reports/scripts/generate_report.py --compare

# 对比特定commit
python3 perf_reports/scripts/generate_report.py --compare --commits 92f879f,d626384

# 生成时间线报告
python3 perf_reports/scripts/generate_report.py --timeline
```

**输出**：`perf_reports/reports/perf_report_*.md` 或 `perf_reports/reports/perf_comparison_*.md` 或 `perf_reports/reports/perf_timeline_*.md`

#### 步骤3：生成性能图表

```bash
# 生成最新版本的性能图表
python3 perf_reports/scripts/generate_charts.py

# 生成版本对比图表
python3 perf_reports/scripts/generate_charts.py --compare
```

**输出**：`perf_reports/reports/*.png`

## 性能指标说明

### 时间指标（毫秒）
- `t_pass_ms`: 点云过滤时间
- `t_ground_ms`: 地面分割时间
- `t_cluster_ms`: 聚类时间
- `t_delaunay_ms`: Delaunay三角剖分时间
- `t_way_ms`: 路径计算时间
- `t_total_ms`: 总处理时间

### 数据量指标
- `N`: 输入点云/锥桶数量
- `K`: 聚类数量
- `T`: 三角形数量
- `E`: 边数量
- `D`: 检测数量
- `bytes`: 发布消息字节数

### 统计指标
- `mean`: 平均值
- `p50`: 中位数（50百分位）
- `p95`: 95百分位（95%的样本低于此值）
- `p99`: 99百分位（99%的样本低于此值）
- `max`: 最大值

## 使用场景

### 场景1：日常性能监控

每次运行rosbag测试后：

```bash
# 1. 运行测试
roslaunch high_speed_tracking play_high_speed_bag.launch

# 2. 收集数据并生成报告
python3 perf_reports/scripts/run_perf_analysis.sh

# 3. 查看报告
cat perf_reports/reports/perf_report_*.md
```

### 场景2：代码优化前后对比

```bash
# 1. 优化前收集数据
python3 perf_reports/scripts/collect_perf_data.py --scenario "high_speed_tracking rosbag" --note "优化前基线" --tags baseline

# 2. 进行代码优化
git commit -m "Optimize ground segmentation"

# 3. 优化后收集数据
python3 perf_reports/scripts/collect_perf_data.py --scenario "high_speed_tracking rosbag" --note "优化地面分割缓存" --tags ground,cache

# 4. 生成对比报告
python3 perf_reports/scripts/generate_report.py --compare

# 5. 生成对比图表
python3 perf_reports/scripts/generate_charts.py --compare
```

### 场景3：长期性能追踪

```bash
# 定期收集数据（例如每天）
python3 perf_reports/scripts/collect_perf_data.py

# 查看历史趋势
python3 perf_reports/scripts/generate_report.py --compare

# 生成时间线报告
python3 perf_reports/scripts/generate_report.py --timeline

# 生成所有版本的对比图表
python3 perf_reports/scripts/generate_charts.py --compare
```

## 报告示例

### 单版本报告结构

```markdown
# 性能测试报告

**生成时间**: 2026-01-16 15:30:00

## 1. 测试环境

### Git 信息
- **Commit**: `92f879fc2e71bdeff0a4e621afcf88e53f1b6963`
- **分支**: `main`
- **提交信息**: refactor package naming and launch configs

### 系统信息
- **主机名**: kerwin
- **操作系统**: Linux 5.15.0
- **Python版本**: 3.8.10
- **CPU**: Intel(R) Core(TM) i7-10700K

- **数据收集时间**: 2026-01-16T15:00:00

### 测试场景
- **场景**: high_speed_tracking rosbag
- **变更说明**: 优化地面分割缓存
- **标签**: ground, cache

## 2. 性能指标

### lidar_cluster

**统计窗口**: 300 个样本

#### 处理时间 (ms)
| 指标 | 平均值 | 中位数 | P95 | P99 | 最大值 |
|------|--------|--------|-----|-----|--------|
| t_pass_ms | 2.375 | 2.359 | 2.955 | 3.315 | 4.019 |
| t_ground_ms | 27.711 | 27.369 | 30.326 | 31.094 | 33.032 |
| t_cluster_ms | 4.691 | 4.599 | 5.328 | 5.790 | 5.918 |
| t_total_ms | 34.906 | 34.580 | 38.014 | 39.076 | 40.306 |

#### 数据量统计
| 指标 | 平均值 | 中位数 | P95 | P99 | 最大值 |
|------|--------|--------|-----|-----|--------|
| N | 32102.123 | 33482.000 | 33596.300 | 33666.000 | 33705.000 |
| K | 14.000 | 14.000 | 15.000 | 15.000 | 15.000 |
| bytes | 251127.627 | 251343.000 | 255631.600 | 256592.260 | 257102.000 |

## 3. 性能分析

### lidar_cluster

- **平均总处理时间**: 34.906 ms
- **最大处理时间**: 40.306 ms
- **P95处理时间**: 38.014 ms

#### 时间占比分析
- 点云过滤: 6.80%
- 地面分割: 79.38%
- 聚类处理: 13.44%

- **平均输入点数**: 32102
- **平均聚类数**: 14

## 4. 建议

基于当前性能数据，建议关注以下方面：

1. 监控P95和P99值，确保系统在高负载下的稳定性
2. 识别处理时间占比最大的模块，优先优化
3. 关注内存使用和消息发布量（bytes指标）
4. 定期进行性能回归测试，确保代码变更不会导致性能下降
```

### 对比报告结构

```markdown
# 性能对比报告

**生成时间**: 2026-01-16 15:30:00

## 1. 版本信息

| 版本 | Commit | 分支 | 提交信息 | 测试时间 |
|------|--------|------|----------|----------|
| v1 | `92f879f` | `main` | refactor package naming... | 2026-01-16T15:00:00 |
| v2 | `d626384` | `main` | Update project files | 2026-01-16T16:00:00 |

### 变更说明与测试场景

| 版本 | 场景 | 变更说明 | 标签 |
|------|------|----------|------|
| v1 | high_speed_tracking rosbag | 优化地面分割缓存 | ground, cache |
| v2 | high_speed_tracking rosbag | 优化聚类筛选 | cluster |

## 2. 性能对比

### lidar_cluster

#### 总处理时间对比 (ms)
| 版本 | 平均值 | 中位数 | P95 | P99 | 最大值 |
|------|--------|--------|-----|-----|--------|
| v1 | 34.906 | 34.580 | 38.014 | 39.076 | 40.306 |
| v2 | 32.123 | 31.890 | 35.234 | 36.543 | 37.890 |

#### 关键指标对比
| 指标 | v1 | v2 | 变化 |
|------|----|----|------|
| t_total_ms | 34.906 | 32.123 | -7.98% ✅ |
| N | 32102.123 | 32500.000 | +1.24% |
| K | 14.000 | 14.000 | 0.00% |

## 3. 分析结论

### 性能变化趋势
- 查看各版本间性能指标的变化趋势
- 关注超过10%的性能变化，标记为⚠️
- 性能提升标记为✅

### 优化建议
1. 识别性能下降的模块，分析原因
2. 保持性能提升的优化
3. 建立性能基准，防止回归
```

### 时间线报告结构

```markdown
# 性能时间线报告

**生成时间**: 2026-01-16 18:00:00

## 1. 时间线

### lidar_cluster

| 时间 | Commit | 场景 | 变更说明 | 标签 | t_total_mean(ms) | t_total_p95(ms) | delta_mean | delta_p95 |
|------|--------|------|----------|------|------------------|------------------|------------|-----------|
| 2026-01-15T10:00:00 | `92f879f` | high_speed_tracking rosbag | 优化地面分割缓存 | ground, cache | 34.906 | 38.014 | N/A | N/A |
| 2026-01-16T16:00:00 | `d626384` | high_speed_tracking rosbag | 优化聚类筛选 | cluster | 32.123 | 35.234 | -2.783 | -2.780 |
```

## 依赖项

```bash
# Python依赖
pip3 install py-cpuinfo matplotlib numpy

# 或使用requirements.txt
pip3 install -r perf_reports/requirements.txt
```

## 故障排除

### 问题1：没有找到日志文件

**错误**：`Warning: Log file not found: ~/.ros/log/latest/rosout.log`

**解决**：
```bash
# 检查ROS日志目录
ls ~/.ros/log/latest/

# 指定正确的日志文件
python3 perf_reports/scripts/collect_perf_data.py --log-file /path/to/your/rosout.log
```

### 问题2：没有性能数据

**错误**：`No data available. Run collect_perf_data.py first.`

**解决**：
```bash
# 确保节点正在运行并输出性能统计
rostopic echo /rosout | grep "\[perf\]"

# 运行测试后等待足够时间收集数据
```

### 问题3：图表生成失败

**错误**：`ModuleNotFoundError: No module named 'matplotlib'`

**解决**：
```bash
# 安装依赖
pip3 install matplotlib numpy py-cpuinfo
```

## 最佳实践

1. **定期收集**：每次代码变更后都收集性能数据
2. **版本控制**：将性能数据提交到Git仓库（可选）
3. **自动化**：集成到CI/CD流程中
4. **基准测试**：建立性能基准，防止回归
5. **文档记录**：记录优化措施和对应的性能变化

## 扩展功能

可以扩展此系统以支持：
- 更多性能指标
- 自定义图表样式
- 导出为其他格式（PDF、HTML）
- 集成到CI/CD
- 性能告警机制
- 历史趋势分析

## 许可证

本系统遵循项目主许可证。
