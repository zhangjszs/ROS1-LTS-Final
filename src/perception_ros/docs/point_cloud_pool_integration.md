# 点云内存池集成说明

## 概述

点云内存池已集成到`perception_ros`，可通过配置文件启用。

## 配置方法

编辑 `perception_ros/config/lidar_cluster_example.yaml`：

```yaml
# ==========================
# Performance Optimization (性能优化)
# ==========================
# 点云内存池：减少频繁的内存分配/释放，提升性能
# 注意：在高频率、大规模点云场景下效果更明显
use_point_cloud_pool: true   # 设置为true启用内存池
```

## 启动验证

启动节点后，查看日志：

```bash
roslaunch fsd_launch missions/trackdrive.launch simulation:=true bag:=/path/to/bag.bag
```

如果启用成功，会看到：
```
[ INFO] Point cloud pool enabled for memory optimization
```

## 性能监控

### 方法1：通过ROS参数服务器

```bash
# 查看当前配置
rosparam get /lidar_cluster_node/use_point_cloud_pool
```

### 方法2：代码中添加统计输出

在`lidar_cluster_ros.cpp`的`pointCallback`中添加：

```cpp
if (use_point_cloud_pool_) {
    auto stats = perception::GlobalPointCloudPool::instance().getStats();
    if (stats.total_acquisitions % 100 == 0) {  // 每100帧输出一次
        ROS_INFO("Pool stats - Small: %zu/%zu, Medium: %zu/%zu, Fallback: %zu",
                 stats.small_available, stats.small_total,
                 stats.medium_available, stats.medium_total,
                 stats.fallback_allocations);
    }
}
```

## 性能对比测试

### 测试场景

使用rosbag回放测试：

```bash
# 禁用内存池
rosparam set /lidar_cluster_node/use_point_cloud_pool false
roslaunch fsd_launch missions/trackdrive.launch simulation:=true bag:=test.bag

# 启用内存池
rosparam set /lidar_cluster_node/use_point_cloud_pool true
roslaunch fsd_launch missions/trackdrive.launch simulation:=true bag:=test.bag
```

### 预期效果

| 场景 | 无内存池 | 有内存池 | 提升 |
|------|----------|----------|------|
| 低频率（<10Hz） | 25ms | 25ms | 0% |
| 中频率（10-20Hz） | 25ms | 23ms | 8% |
| 高频率（>20Hz） | 30ms | 22ms | 27% |

**注意**：
- 内存池在**高频率、大规模点云**场景下效果更明显
- 小规模测试可能看不到明显提升
- 配合零拷贝优化效果更佳

## 调优建议

### 1. 调整池大小

如果发现`fallback_allocations`过高（>10%），说明池不够用，可以增加池大小。

修改代码中的默认配置（未来可以移到yaml）：

```cpp
perception::PointCloudPoolConfig config;
config.small_pool_size = 20;   // 默认10
config.medium_pool_size = 15;  // 默认8
config.large_pool_size = 8;    // 默认4
```

### 2. 线程安全

如果使用多线程处理点云，启用线程安全：

```cpp
config.thread_safe = true;
```

## 故障排查

### 问题1：内存使用增加

**原因**：内存池预分配了点云对象

**解决**：
- 小池：10个 × 5k点 × 16字节 ≈ 0.8MB
- 中池：8个 × 30k点 × 16字节 ≈ 3.8MB
- 大池：4个 × 100k点 × 16字节 ≈ 6.4MB
- **总计**：约11MB（可接受）

### 问题2：性能没有提升

**可能原因**：
1. 点云规模太小（<5k点）
2. 处理频率太低（<5Hz）
3. 瓶颈在其他地方（地面分割算法、聚类等）

**建议**：
- 使用`perf_stats_`分析各模块耗时
- 确认内存分配是否是瓶颈

### 问题3：Fallback过多

**现象**：`fallback_allocations`占比>10%

**解决**：增加对应池的大小

## 下一步优化

1. **零拷贝优化**：使用`swap()`代替数据拷贝
2. **池大小可配置**：将池配置移到yaml文件
3. **自适应池大小**：根据运行时统计动态调整

## 参考文档

- 详细使用指南：`perception_core/docs/point_cloud_pool_usage.md`
- 头文件：`perception_core/include/perception_core/point_cloud_pool.hpp`
- 测试代码：`perception_core/test/test_point_cloud_pool.cpp`
