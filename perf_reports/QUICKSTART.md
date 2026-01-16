# 性能报告系统 - 快速开始指南

## 系统概述

本系统自动收集ROS节点的性能数据，与Git版本关联，并生成Markdown格式的报告和图表，便于长期追踪性能变化。

## 目录结构

```
perf_reports/
├── data/                      # 性能数据存储（JSON格式）
│   ├── perf_data_20260116_162125.json
│   └── perf_data_20260116_162149.json
├── reports/                   # 生成的Markdown报告和图表
│   ├── perf_report_92f879fc_20260116_162202.md
│   ├── lidar_cluster_performance_20260116_162206.png
│   └── high_speed_tracking_performance_20260116_162206.png
├── scripts/                   # 自动化脚本
│   ├── collect_perf_data.py     # 收集性能数据
│   ├── generate_report.py       # 生成Markdown报告
│   ├── generate_charts.py      # 生成性能图表
│   └── run_perf_analysis.sh   # 一键运行所有步骤
├── quick_report.sh            # 快速生成报告（推荐使用）
├── requirements.txt           # Python依赖
└── README.md                # 详细文档
```

## 快速开始

### 方法1：使用快速脚本（推荐）

```bash
# 1. 运行rosbag测试
roslaunch high_speed_tracking play_high_speed_bag.launch

# 2. 测试完成后，运行快速报告脚本
cd /home/kerwin/2025huat
./perf_reports/quick_report.sh
```

### 方法2：使用Python脚本

```bash
# 1. 收集性能数据
python3 perf_reports/scripts/collect_perf_data.py

# 添加变更说明/场景/标签
python3 perf_reports/scripts/collect_perf_data.py --note "优化地面分割缓存" --scenario "high_speed_tracking rosbag" --tags ground,cache

# 2. 生成Markdown报告
python3 perf_reports/scripts/generate_report.py

# 3. 生成性能图表
python3 perf_reports/scripts/generate_charts.py
```

### 方法3：使用一键脚本

```bash
python3 perf_reports/scripts/run_perf_analysis.sh
```

## 查看生成的报告

### 查看Markdown报告

```bash
# 查看最新报告
cat perf_reports/reports/perf_report_*.md

# 查看时间线报告
cat perf_reports/reports/perf_timeline_*.md

# 使用Markdown查看器
less perf_reports/reports/perf_report_*.md

# 在浏览器中打开
firefox perf_reports/reports/perf_report_*.md
```

### 查看性能图表

```bash
# 使用图像查看器
eog perf_reports/reports/*.png

# 或使用其他查看器
feh perf_reports/reports/*.png
```

## 版本对比

### 对比两个版本的性能

```bash
# 1. 收集第一个版本的数据
python3 perf_reports/scripts/collect_perf_data.py

# 2. 进行代码修改
git commit -m "Optimize ground segmentation"

# 3. 收集第二个版本的数据
python3 perf_reports/scripts/collect_perf_data.py

# 4. 生成对比报告
python3 perf_reports/scripts/generate_report.py --compare

# 5. 生成对比图表
python3 perf_reports/scripts/generate_charts.py --compare
```

### 对比特定commit

```bash
# 对比两个特定的commit
python3 perf_reports/scripts/generate_report.py --compare --commits 92f879f,d626384

# 生成时间线报告
python3 perf_reports/scripts/generate_report.py --timeline
```

## 性能指标说明

### 时间指标（毫秒）
- **t_pass_ms**: 点云过滤时间
- **t_ground_ms**: 地面分割时间
- **t_cluster_ms**: 聚类时间
- **t_delaunay_ms**: Delaunay三角剖分时间
- **t_way_ms**: 路径计算时间
- **t_total_ms**: 总处理时间

### 数据量指标
- **N**: 输入点云/锥桶数量
- **K**: 聚类数量
- **T**: 三角形数量
- **E**: 边数量
- **D**: 检测数量
- **bytes**: 发布消息字节数

### 统计指标
- **mean**: 平均值
- **p50**: 中位数（50百分位）
- **p95**: 95百分位（95%的样本低于此值）
- **p99**: 99百分位（99%的样本低于此值）
- **max**: 最大值

## 实际使用示例

### 示例1：日常性能监控

```bash
# 每次运行rosbag测试后
roslaunch high_speed_tracking play_high_speed_bag.launch

# 等待测试完成，然后生成报告
./perf_reports/quick_report.sh

# 查看报告
cat perf_reports/reports/perf_report_*.md
```

### 示例2：优化前后对比

```bash
# 1. 优化前收集数据
roslaunch high_speed_tracking play_high_speed_bag.launch &
sleep 60
python3 perf_reports/scripts/collect_perf_data.py --scenario "high_speed_tracking rosbag" --note "优化前基线" --tags baseline

# 2. 进行代码优化
# 编辑代码...
git commit -m "Optimize Delaunay triangulation"

# 3. 优化后收集数据
roslaunch high_speed_tracking play_high_speed_bag.launch &
sleep 60
python3 perf_reports/scripts/collect_perf_data.py --scenario "high_speed_tracking rosbag" --note "优化Delaunay三角剖分" --tags delaunay

# 4. 生成对比报告
python3 perf_reports/scripts/generate_report.py --compare
python3 perf_reports/scripts/generate_charts.py --compare

# 5. 查看对比结果
cat perf_reports/reports/perf_comparison_*.md
```

### 示例3：长期性能追踪

```bash
# 定期收集数据（例如每天）
# 第1天
./perf_reports/quick_report.sh

# 第2天
./perf_reports/quick_report.sh

# 第3天
./perf_reports/quick_report.sh

# 查看历史趋势
python3 perf_reports/scripts/generate_report.py --compare
python3 perf_reports/scripts/generate_charts.py --compare

# 生成时间线报告
python3 perf_reports/scripts/generate_report.py --timeline
```

## 报告内容解读

### 单版本报告

报告包含以下部分：

1. **测试环境**
   - Git信息（commit、分支、提交信息）
   - 系统信息（主机名、OS、Python版本、CPU）
   - 数据收集时间
   - 测试场景（场景、变更说明、标签）

2. **性能指标**
   - 处理时间统计表
   - 数据量统计表
   - 按节点分类显示

3. **性能分析**
   - 总处理时间统计
   - 时间占比分析
   - 关键指标解读

4. **建议**
   - 基于数据的优化建议
   - 监控要点

### 对比报告

对比报告包含：

1. **版本信息表**
   - 各版本的commit信息
   - 测试时间
   - 变更说明与测试场景

2. **性能对比**
   - 总处理时间对比图
   - P95/P99对比
   - 关键指标变化百分比
   - 显著变化标记（⚠️表示超过10%，✅表示提升）

3. **分析结论**
   - 性能变化趋势
   - 优化建议

### 时间线报告

时间线报告包含：

1. **按节点时间线**
   - 按 collection_time 排序的性能记录
   - 变更说明/场景/标签
   - t_total_mean / t_total_p95
   - 相邻差值（delta_mean/delta_p95）

## 故障排除

### 问题1：没有找到日志文件

**错误信息**：
```
Warning: Log file not found: ~/.ros/log/latest/rosout.log
```

**解决方案**：
```bash
# 检查ROS日志目录
ls ~/.ros/log/latest/

# 指定正确的日志文件
python3 perf_reports/scripts/collect_perf_data.py --log-file /path/to/your/rosout.log
```

### 问题2：没有性能数据

**错误信息**：
```
No data available. Run collect_perf_data.py first.
```

**解决方案**：
```bash
# 确保节点正在运行并输出性能统计
rostopic echo /rosout | grep "\[perf\]"

# 运行测试后等待足够时间收集数据
# 至少需要收集300个样本才能输出统计信息
```

### 问题3：图表生成失败

**错误信息**：
```
ModuleNotFoundError: No module named 'matplotlib'
```

**解决方案**：
```bash
# 安装依赖
pip3 install matplotlib numpy

# 或使用requirements.txt
pip3 install -r perf_reports/requirements.txt
```

### 问题4：Git信息获取失败

**错误信息**：
```
commit_hash: "unknown"
```

**解决方案**：
```bash
# 确保在Git仓库中
cd /home/kerwin/2025huat
git status

# 如果不在仓库中，初始化Git
git init
git add .
git commit -m "Initial commit"
```

## 最佳实践

1. **定期收集**：每次代码变更后都收集性能数据
2. **版本控制**：将性能数据提交到Git仓库（可选）
3. **自动化**：集成到CI/CD流程中
4. **基准测试**：建立性能基准，防止回归
5. **文档记录**：记录优化措施和对应的性能变化

## 集成到工作流程

### 选项1：手动集成

```bash
# 在每次代码变更后
1. 运行rosbag测试
2. 执行 ./perf_reports/quick_report.sh
3. 查看报告，确认性能变化
4. 提交代码和报告
```

### 选项2：自动化集成

创建一个脚本自动执行所有步骤：

```bash
#!/bin/bash
# auto_test.sh

# 1. 运行测试
timeout 120 roslaunch high_speed_tracking play_high_speed_bag.launch

# 2. 生成报告
./perf_reports/quick_report.sh

# 3. 检查性能回归
# 添加你的检查逻辑...

# 4. 如果通过，提交代码
git add .
git commit -m "Performance test passed"
```

## 扩展功能

可以扩展此系统以支持：

1. **更多性能指标**：添加CPU使用率、内存使用等
2. **自定义图表**：修改generate_charts.py添加新的图表类型
3. **导出格式**：支持PDF、HTML等格式
4. **CI/CD集成**：自动运行性能测试
5. **性能告警**：当性能下降超过阈值时发送通知
6. **历史趋势**：添加时间序列分析

## 技术支持

如有问题，请查看：
- 详细文档：[perf_reports/README.md](file:///home/kerwin/2025huat/perf_reports/README.md)
- 脚本源码：[perf_reports/scripts/](file:///home/kerwin/2025huat/perf_reports/scripts/)
- 示例报告：[perf_reports/reports/](file:///home/kerwin/2025huat/perf_reports/reports/)

## 许可证

本系统遵循项目主许可证。
