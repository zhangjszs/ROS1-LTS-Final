# 定位验证工具集

本目录包含定位模块的验证和测试工具，用于支持定位模块的开发、测试和验证工作。

## 工具列表

### 1. localization_health_check.sh
**功能**：实时检查定位系统的运行状态

**用法**：
```bash
./localization_health_check.sh
```

**检查内容**：
- ROS核心运行状态
- 定位相关话题状态和发布频率
- 定位工作状态（TRACKING/DEGRADED/INS_ONLY）
- 端到端延迟指标
- 重定位成功率

---

### 2. localization_log_check.sh
**功能**：分析rosout日志，查找定位模块的错误和异常

**用法**：
```bash
./localization_log_check.sh <rosout.log路径> [搜索关键词]
```

**示例**：
```bash
# 分析最新日志
./localization_log_check.sh ~/.ros/log/latest/rosout.log

# 只分析包含localization关键词的日志
./localization_log_check.sh ~/.ros/log/latest/rosout.log localization
```

**分析内容**：
- 错误和警告统计
- 重定位成功/失败统计
- 定位状态变化记录
- 异常时间跳变检测

---

### 3. inject_scenario.py
**功能**：向定位系统注入测试场景，验证系统鲁棒性

**用法**：
```bash
./inject_scenario.py --scenario <场景类型> [选项]
```

**支持的场景**：
| 场景类型 | 功能 | 选项 |
|---------|------|------|
| `gps_loss` | 模拟GPS/定位信号丢失 | `--duration`: 丢失持续时间(秒) |
| `cone_noise` | 向锥桶检测注入位置噪声 | `--noise-std`: 噪声标准差(米)，`--duration`: 持续时间 |
| `wrong_color` | 注入锥桶颜色错误 | `--swap-prob`: 颜色交换概率，`--duration`: 持续时间 |

**示例**：
```bash
# 注入5秒GPS丢失
./inject_scenario.py --scenario gps_loss --duration 5

# 注入0.5m的锥桶位置噪声，持续10秒
./inject_scenario.py --scenario cone_noise --noise-std 0.5 --duration 10

# 注入30%概率的颜色错误
./inject_scenario.py --scenario wrong_color --swap-prob 0.3 --duration 15
```

---

### 4. compare_backends.py
**功能**：对比mapper和Factor Graph两个定位后端的输出，验证FG后端迁移的正确性

**用法**：
```bash
./compare_backends.py --duration 60 --output fg_validation_results.csv
```

**输出**：
- 实时统计位置、航向、速度误差
- 自动保存对比数据到`backend_comparison.csv`
- 测试结束后生成对比图表`backend_comparison.png`
- 打印误差统计信息（平均值、最大值、95分位值等）

**要求**：
- 需要同时启动两个后端，分别发布到`/localization/mapper/car_state`和`/localization/fg/car_state`话题

---

### 5. cone_gt_annotator.py
**功能**：从回放 bag 中提取锥桶检测点并聚类，生成 GT CSV 初稿

**用法**：
```bash
./cone_gt_annotator.py --bag ~/rosbag/track.bag --output ~/rosbag/track_gt.csv
```

**常用参数**：
- `--topic`：检测话题，默认`/perception/lidar_cluster/detections`
- `--max-messages`：最多读取帧数，默认300
- `--merge-radius`：点聚类半径（米），默认0.6
- `--min-observations`：最小观测次数过滤，默认3

---

## 典型使用流程

### 1. 常规健康检查
```bash
# 运行系统时，定期执行健康检查
./localization_health_check.sh
```

### 2. 问题定位
```bash
# 出现异常时，分析日志
./localization_log_check.sh ~/.ros/log/latest/rosout.log
```

### 3. 鲁棒性测试
```bash
# 注入各种场景测试定位系统的稳定性
./inject_scenario.py --scenario gps_loss --duration 5
./inject_scenario.py --scenario cone_noise --noise-std 0.3
```

### 4. FG后端迁移验证
```bash
# 同时启动新旧两个后端，运行对比工具
./compare_backends.py
# 收集足够数据后，分析误差是否在可接受范围内
```
