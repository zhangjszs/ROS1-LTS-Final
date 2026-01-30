# vehicle_racing_num_core

ROS-independent race number management.

## Purpose

Core race number writing and management without ROS dependencies.

## Features

- Race number file I/O
- Configuration management

## Dependencies

- Standard C++ libraries

## Usage

```cpp
#include <vehicle_racing_num_core/racing_num_writer.hpp>

// Manage race numbers
```

## Integration

Used by `vehicle_racing_num_ros` package for ROS integration.

## Testing

```bash
catkin run_tests vehicle_racing_num_core
```
