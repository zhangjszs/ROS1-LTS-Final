# control_core

ROS-independent control algorithms for FSD autonomous driving.

## Purpose

Core vehicle control algorithms without ROS dependencies.

## Features

- **Controller Base**: Abstract base class for all controllers
- **Line Controller**: Path tracking for straight-line missions
- **High-Speed Controller**: Optimized for high-speed tracking
- **Skidpad Controller**: Specialized for figure-8 maneuvers
- **Test Controller**: Development and testing controller

## Control Methods

- Pure pursuit path tracking
- PID steering control
- Velocity control with feedforward
- Brake force calculation

## Usage

```cpp
#include <control_core/line_controller.hpp>

LineController controller;
ControlParams params;
controller.SetParams(params);
controller.UpdateCarState(state);
controller.UpdatePath(path);
ControlOutput output = controller.ComputeOutput();
```

## Parameters

- `angle_kp`, `angle_ki`, `angle_kd`: PID gains for steering
- `angle_kv`: Velocity-dependent steering gain
- `angle_kl`: Lookahead distance gain
- `steering_delta_max`: Maximum steering rate
- `car_length`: Vehicle wheelbase

## Integration

Used by `control_ros` package for ROS integration.

## Testing

```bash
catkin run_tests control_core
```
