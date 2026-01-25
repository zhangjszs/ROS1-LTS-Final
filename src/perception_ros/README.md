# perception_ros 激光雷达点云聚类（ROS 包装层）

检测接收到的的点云并进行滤波与聚类，算法核心已拆到 `perception_core`（无 ROS 依赖），本包仅负责 ROS 订阅/发布/参数/可视化。  
参考 `config/lidar_cluster_example.yaml` 中的注释来修改参数。  
复制一份 example yaml 文件，更名为 “lidar_cluster.yaml”。

## 使用方法

构建后使用 `roslaunch perception_ros lidar_cluster.launch` 启动即可。

## 实现原理

### 程序

  1. `lidar_cluster_ros.cpp` 启动，进行初始化。nodelet 控制独立线程的 `RunOnce()` 也开始运行。

  2. `lidar_cluster_ros.cpp` 中的回调函数接受到原始点云，存入 core。使用 `pcl::fromROSMsg(*original_cloud_ptr, *current_pc_ptr)` 将点云数据存入新的 `current_pc_ptr` 变量中。

  3. 在 `RunOnce()` 中，首先使用直通滤波 `PassThrough()` 进行处理并发布。  
    此后 `ground_segmentation()` 进行地面分割，同样发布数据到 `points/no_ground`。

  4. 然后进行聚类 `clusterMethod32()`,在非地面点云中查找锥桶。

  5. 把最终识别结果发布至 `detections`

### 直线

  1. 订阅 `points/raw` 原始点云数据（默认由 launch 中的 `topics/input` 指定）

  2. 进行直通滤波 (Passthrough) 处理
    在 `perception_core` 中被调用

  3. 进行聚类处理

  3. 使用 `cluster_velodyne32()` 进行聚类后处理  
      - 提取聚类点云
      - 计算置信度
      - 计算 bbox 和聚类中心

  4. publish 相关结果

## Todo

- [x] 理解 `/config/lidar_cluster.yaml` 中参数含义及用途

- [x] 尝试添加各点云簇置信度

- [x] 重写置信度计算 

- [x] 根据bbox的z轴高度去除误判对象

- [x] 针对 8 字，赛道环境实现特定的滤波&匹配模式

- [x] 修复由 `marker_array_all` 引起的 euc 错误问题

- [ ] 代码风格统一化


## Rviz Config 解析 （使用 lidar_cluster(_c).rviz）

- lidar_cluster 提供 raw、passthrough、cones 三个点云输出，和 markerArray（识别为锥桶的点云聚类）;

- markerArray(all) 提供所有已识别的聚类的 bbox

## Topic

### raw（/perception/lidar_cluster/points/raw）

原始激光雷达数据

### passthrough（/perception/lidar_cluster/points/passthrough）

使用直通滤波后的结果

### no_ground（/perception/lidar_cluster/points/no_ground）

滤除了地面，效果尚可

### cones（/perception/lidar_cluster/points/cones）

识别为锥桶的点云

### detections (/perception/lidar_cluster/detections)

以数组形式给出当前push中所有的锥桶信息：
  
  - （相对车辆的）坐标
  
  - 置信度

  - bbox 最大点及最小点

  - 相对距离

  - 颜色

> 参考 `autodrive_msgs/msg/HUAT_ConeDetections.msg`

## 疑问

- `imu_subscriber` 中使用的惯导数据借口去与INS570D的接口并不相同，使用的是 “p2”
