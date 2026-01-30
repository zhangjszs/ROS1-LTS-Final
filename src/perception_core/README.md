# perception_core

ROS-independent perception algorithms for FSD autonomous driving.

## Purpose

Core perception algorithms without ROS dependencies, enabling:
- Unit testing without ROS infrastructure
- Portability to non-ROS systems
- Clear separation of algorithm logic from middleware

## Features

- **LiDAR Clustering**: Euclidean clustering with adaptive distance thresholds
- **Ground Segmentation**: RANSAC and Patchwork++ algorithms
- **Cone Detection**: Feature extraction and confidence scoring
- **Cone Model Fitting**: Geometric model validation
- **Distortion Adjustment**: Motion compensation for point clouds

## Dependencies

- PCL (Point Cloud Library)
- Eigen3

## Usage

```cpp
#include <perception_core/lidar_cluster_core.hpp>

LidarClusterConfig config;
lidar_cluster cluster;
cluster.Configure(config);
cluster.SetInputCloud(cloud, seq);
LidarClusterOutput output;
cluster.Process(&output);
```

## Integration

Used by `perception_ros` package for ROS integration.

## Testing

```bash
catkin run_tests perception_core
```
