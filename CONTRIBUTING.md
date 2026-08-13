# Contributing to 2025HUAT

Thank you for your interest in contributing to the 2025HUAT Formula Student Driverless autonomous racing system! This document provides guidelines for contributing to the project.

## Table of Contents

- [Development Setup](#development-setup)
- [Architecture Overview](#architecture-overview)
- [Coding Standards](#coding-standards)
- [Testing](#testing)
- [Commit Guidelines](#commit-guidelines)
- [Pull Request Process](#pull-request-process)
- [Documentation](#documentation)

---

## Development Setup

### Prerequisites

- **OS:** Ubuntu 20.04 (recommended)
- **ROS:** ROS Noetic
- **C++:** C++17 compatible compiler
- **Python:** 3.8+

### Building the Project

```bash
# Source ROS environment
source /opt/ros/noetic/setup.bash

# Build all packages
catkin build --no-status --summarize

# Source the workspace
source devel/setup.bash
```

### Pre-commit Hooks

We use pre-commit hooks to enforce code quality. Install them once:

```bash
pre-commit install
pre-commit run --all-files
```

---

## Architecture Overview

The project follows a **Core + ROS Wrapper** architecture:

- **`*_core/`** packages: Pure C++ algorithms with no ROS dependencies (PCL, Eigen only)
- **`*_ros/`** packages: ROS wrappers that handle topics, services, and parameters
- **`autodrive_msgs/`**: Custom message definitions
- **`fsd_launch/`**: Mission/subsystem launch composition and runtime config
- **`fsd_common/`**: Header-only shared utilities and system-wide contracts

### Dependency Rules

1. `*_core` packages **must not** depend on ROS
2. `*_ros` packages depend on their corresponding `*_core` and `fsd_common`
3. `fsd_common` is a header-only utility package
4. `autodrive_msgs` should only contain message definitions

### Configuration Loading

Configuration follows a hierarchical loading order (later layers override earlier ones):

```
1. base config     → *_ros/config/<subsystem>_base.yaml
2. mode config     → *_ros/config/<subsystem>_<mode>.yaml
3. mission config  → fsd_launch/config/missions/<mission>/<subsystem>.yaml
4. vehicle config  → fsd_launch/config/vehicles/<vehicle>/<subsystem>.yaml
5. local config    → *.local.yaml (gitignored)
```

---

## Coding Standards

### Language Standards

- **C++**: Target C++17
- **Python**: 3.8+

### Naming Conventions

| Element | Convention | Example |
|---------|------------|---------|
| Files (C++) | lower_snake_case | `lidar_cluster_core.cpp` |
| Files (Python) | lower_snake_case | `cone_detector.py` |
| Classes | UpperCamelCase | `ConeTracker` |
| Functions | lower_snake_case | `extract_features()` |
| Variables | lower_snake_case | `detection_score` |
| Constants | UPPER_SNAKE_CASE | `MAX_ITERATIONS` |
| ROS Topics | lower_snake_case | `/perception/cones` |
| Namespaces | lower_snake_case | `perception::clustering` |

### Topic Naming

```
Format: <subsystem>/<component>/<data_type>
Example: perception/lidar_cluster/detections
```

- All lowercase snake_case
- No camelCase (legacy topics exempted during migration)

### Frame Naming

- `world` — Global ENU coordinate frame
- `base_link` — Vehicle center
- `velodyne` — LiDAR sensor
- `imu` — IMU sensor

### C++ Formatting

We use `clang-format` with the project's `.clang-format` configuration:

```bash
clang-format -i <file>
```

Key rules:
- 2 spaces indentation, no tabs
- 100 character column limit
- Egyptian style braces (attached)
- Left-aligned pointers/references (`int* ptr`, `int& ref`)

### Include Order (C++)

```cpp
// 1. ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// 2. Standard library
#include <vector>
#include <string>

// 3. Third party (Eigen, PCL, OpenCV)
#include <pcl/point_cloud.h>
#include <Eigen/Dense>

// 4. Project headers
#include "perception_core/clustering.h"
```

### Python Formatting

```bash
black <file_or_dir>
isort <file_or_dir>
```

### Linting

```bash
# C++ linting
clang-tidy <file> -- -std=c++17

# Python linting
flake8 <file_or_dir>
bandit -ll -r src/
```

### Error Handling

- **C++**: Use exceptions sparingly; prefer error codes or `std::optional<T>` for recoverable errors
- **Python**: Raise specific exceptions; avoid bare `except:`
- **ROS**: Use `ROS_ERROR()`, `ROS_WARN()` for logging; return false on failure

### Type Conventions

- Use `const` by default for references and pointers when not modifying
- Prefer `int64_t` for timestamps (microseconds)
- Use `double` for floating-point calculations
- Avoid raw loops; use STL algorithms or Eigen operations

---

## Testing

### Running Tests

```bash
# Run all tests
catkin run_tests --no-status --summarize
catkin_test_results build

# Run tests for specific package
catkin run_tests <package_name>
catkin_test_results build/<package_name>

# Run individual test binary
./build/<package>/test/test_<name>

# Run with GTest filter
./build/perception_core/test/test_cone_tracker --gtest_filter="*ConeTracker*"
```

### Test Organization

- Unit tests in `*_core/test/` for algorithmic changes
- Integration tests in `*_ros/test/` for ROS interfaces
- Performance benchmarks in `<package>/benchmark/`

### GTest Patterns

```cpp
TEST(ConeTrackerTest, UpdatesPositions) {
  ConeTracker tracker;
  EXPECT_TRUE(tracker.update(cloud));
  EXPECT_EQ(tracker.size(), 3u);
}
```

---

## Commit Guidelines

We follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <description>

[optional body]
```

### Types

| Type | Description |
|------|-------------|
| `feat` | New feature |
| `fix` | Bug fix |
| `docs` | Documentation only |
| `style` | Code style (formatting, no logic change) |
| `refactor` | Code refactoring |
| `test` | Adding or updating tests |
| `chore` | Build, CI, or tooling changes |
| `perf` | Performance improvement |

### Examples

```
feat(planning): add skidpad detection
fix(localization): correct IMU time offset
chore(config): update cone trust parameters
test(perception): add cone tracker unit tests
docs(readme): update installation instructions
```

### Scope

Use the package or module name as scope: `perception`, `planning`, `control`, `localization`, `vehicle`, `launch`, `config`, `ci`, `docs`

---

## Pull Request Process

1. **Create a feature branch** from `main`:
   ```bash
   git checkout -b feat/your-feature-name
   ```

2. **Make your changes** following the coding standards above

3. **Run tests** and ensure they pass:
   ```bash
   catkin build
   catkin run_tests
   ```

4. **Run linters** on changed files:
   ```bash
   clang-format -i <changed_files>
   pre-commit run --all-files
   ```

5. **Update documentation** if your change affects:
   - Architecture → update `docs/`
   - APIs → update relevant `*_ros/README.md`
   - Configuration → update config files and docs

6. **Submit a PR** with:
   - Problem/solution summary
   - Impacted packages and launch/config files
   - Validation commands run (with results)
   - Linked issue/task
   - Screenshots for visualization changes

7. **Address review feedback** and ensure CI passes

---

## Documentation

### Project Documentation

- `README.md` — Project overview and quick start
- `docs/` — Design and process documentation
- `docs/modules/` — Per-module design documents
- `docs/plans/` — Implementation plans
- `docs/system_specifications/` — System-level specifications
- `src/*/README.md` — Package-specific documentation

### Adding Documentation

When adding new features:
1. Update or create the relevant `docs/modules/<module>/` document
2. Add a design document to `docs/plans/` for significant changes
3. Update `CHANGELOG.md` with the change entry

---

## License

By contributing to this project, you agree that your contributions will be licensed under the BSD 3-Clause License. See [LICENSE](./LICENSE) for details.
