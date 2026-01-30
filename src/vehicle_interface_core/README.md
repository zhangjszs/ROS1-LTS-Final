# vehicle_interface_core

ROS-independent vehicle communication interface.

## Purpose

Core UDP socket utilities for vehicle communication without ROS dependencies.

## Features

- UDP socket communication
- Vehicle command serialization
- Vehicle status deserialization

## Dependencies

- Standard C++ libraries

## Usage

```cpp
#include <vehicle_interface_core/udp_socket.hpp>

// Use UDP socket for vehicle communication
```

## Integration

Used by `vehicle_interface_ros` package for ROS integration.

## Testing

```bash
catkin run_tests vehicle_interface_core
```
