# planning_core

ROS-independent planning algorithms for FSD autonomous driving.

## Purpose

Core path planning algorithms without ROS dependencies.

## Features

- **Line Detection**: Hough transform-based track boundary detection
- **Skidpad Detection**: Figure-8 pattern recognition and path generation
- **Path Generation**: Center line calculation and trajectory planning

## Dependencies

- PCL (Point Cloud Library)
- Eigen3

## Algorithms

### Line Detection
- Hough transform for boundary line detection
- Center line calculation from left/right boundaries
- Finish line detection for lap completion

### Skidpad Detection
- Circle fitting for figure-8 patterns
- Path generation for skidpad maneuvers

## Usage

```cpp
#include <planning_core/line_detection_core.hpp>

LineDetectionParams params;
LineDetectionCore core(params);
core.UpdateCones(cones);
core.UpdateVehicleState(state);
core.RunAlgorithm();
auto path = core.GetPlannedPath();
```

## Integration

Used by `planning_ros` package for ROS integration.

## Testing

```bash
catkin run_tests planning_core
```
