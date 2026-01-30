# localization_core

ROS-independent localization algorithms for FSD autonomous driving.

## Purpose

Core localization and state estimation algorithms without ROS dependencies.

## Features

- **IMU State Estimator**: Inertial measurement unit data processing
- **Location Mapping**: Position tracking and coordinate transformation
- **Sensor Fusion**: Multi-sensor data integration

## Dependencies

- Eigen3

## Usage

```cpp
#include <localization_core/imu_state_estimator.hpp>

IMUStateEstimator estimator;
// Configure and use estimator
```

## Integration

Used by `localization_ros` package for ROS integration.

## Testing

```bash
catkin run_tests localization_core
```
