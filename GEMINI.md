# GEMINI.md - 2025HUAT FSD Autonomous System

This file provides context and instructions for AI agents working on the 2025HUAT Formula Student Driverless (FSD) project.

## Project Overview

**2025HUAT** is an autonomous racing software stack developed for Formula Student Driverless competitions. It is built on **ROS Noetic** and follows a modular **Core + ROS Wrapper** architecture to decouple algorithm logic from the ROS middleware.

### Core Architecture Principles
- **Separation of Concerns:** Algorithms are implemented in `*_core` packages (pure C++17, no ROS dependency). ROS integration happens in `*_ros` packages.
- **Interface Stability:** All custom messages are defined in `autodrive_msgs`.
- **Shared Utilities:** Common headers (topic contracts, geometry utils, time utils) are centralized in `fsd_common`.
- **Config Management:** Hierarchical YAML configuration (Base -> Mode -> Mission -> Vehicle -> Local).

## Key Modules

| Package Category | Description |
|------------------|-------------|
| `*_core` | Pure C++ algorithm libraries (Perception, Planning, Control, Localization). |
| `*_ros` | ROS node/nodelet adapters for the corresponding core libraries. |
| `autodrive_msgs` | Unified ROS message definitions (`HUAT_*`). |
| `fsd_launch` | Mission entry points and configuration files. |
| `fsd_common` | Header-only shared utilities and system-wide contracts. |
| `fsd_visualization` | RViz markers and visualization nodes. |
| `vehicle_interface` | Bridge between ROS and vehicle hardware (UDP/CAN). |

## Development Workflows

### Build System
The project uses `catkin_tools`.
```bash
# Build all packages
catkin build

# Build specific package
catkin build <package_name>

# Clean and rebuild
catkin clean -y && catkin build
```

### Testing
- **Unit Tests:** Located in `*_core/test` and `*_ros/test`.
- **CI Scripts:** Run `scripts/check_topic_contracts.sh`, `scripts/check_perf_stats_contracts.sh`, etc., to verify system integrity.
```bash
# Run all tests
catkin run_tests
```

### Running the System (Simulation)
```bash
# High-speed tracking mission with rosbag
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag

# Options: simulation:=true, bag:=<path>, loop:=true, rate:=<float>, rviz_mode:=dual
```

## Coding Conventions

### Language Standards
- **C++:** C++17. Use `clang-format` and `clang-tidy`.
- **Python:** Python 3.8+. Follow PEP8 (managed by `ruff` and `black`).

### Naming Conventions
- **Topics:** `<subsystem>/<component>/<data_type>` (e.g., `perception/lidar_cluster/detections`).
- **Frames:** `world`, `base_link`, `velodyne`, `imu`.
- **Parameters:** `vehicle/wheelbase`, `control/pure_pursuit/kp`.
- **Messages:** `HUAT_PascalCase.msg`.

### Dependency Rules
1. `*_core` packages **must not** depend on ROS.
2. `*_ros` packages depend on their corresponding `*_core` and `fsd_common`.
3. `fsd_common` is a header-only utility package.
4. `autodrive_msgs` should only contain message definitions (C++ headers moved to `fsd_common`).

## Documentation & Reference
- `README.md`: General project info.
- `HANDOFF.md`: Recent changes and current state (Perception/System focus).
- `RESTRUCTURE_PLAN.md`: Historical context on the modular refactoring.
- `docs/remaining_work_audit_2026-03-06.md`: Current TODOs and known gaps.
