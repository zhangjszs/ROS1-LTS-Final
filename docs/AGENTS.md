# Repository Guidelines

## Project Overview
This is a ROS Noetic `catkin` workspace for autonomous driving (FSD HUAT). Main code lives in `src/`.

### Module Organization
- `*_core/`: Algorithmic modules without ROS dependencies
- `*_ros/`: ROS wrappers (nodes, launch integration, topic/parameter plumbing)
- `src/autodrive_msgs/`: Shared message definitions
- `src/fsd_launch/`: Mission/subsystem launch composition and runtime config
- `docs/`: Design and process documentation
- `scripts/` and `perf_reports/`: Validation and performance tooling

**Do not edit generated workspace artifacts in `build/`, `devel/`, or `logs/`.**

---

## Build Commands

### Full Build
```bash
source /opt/ros/noetic/setup.bash
catkin build --no-status --summarize
source devel/setup.bash
```

### Build Single Package
```bash
catkin build <package_name>
```

### Build with Tests
```bash
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

---

## Test Commands

### Run All Tests
```bash
catkin run_tests --no-status --summarize
catkin_test_results build
```

### Run Single Package Tests
```bash
catkin run_tests <package_name>
catkin_test_results build/<package_name>
```

### Run Individual Test Binary (after building)
```bash
# Find the test binary in build/<package>/test/
./build/<package>/test/test_<name>
```

### Run with GTest Filter
```bash
./build/perception_core/test/test_cone_tracker --gtest_filter="*ConeTracker*"
```

### ROS Launch Testing
```bash
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to.bag
```

---

## Linting & Code Formatting

### C++ Formatting (Required before commit)
```bash
clang-format -i <file>
clang-format -i src/*/src/*.cpp src/*/include/*.hpp
```

### C++ Linting
```bash
clang-tidy <file> -- -std=c++17
```

### Python Formatting
```bash
black <file_or_dir>
isort <file_or_dir>
```

### Python Linting
```bash
flake8 <file_or_dir>
bandit -ll -r src/
```

### Pre-commit Hooks (Install once)
```bash
pre-commit install
pre-commit run --all-files
```

---

## Code Style Guidelines

### Language Standard
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
| ROS Topics/Services | lower_snake_case | `/perception/cones` |
| Namespaces | lower_snake_case | `perception::clustering` |

### C++ Formatting Rules (from `.clang-format`)
- **Indentation**: 2 spaces, no tabs
- **Column limit**: 100 characters
- **Braces**: Attached (Egyptian style)
- **Pointers/References**: Left-aligned (`int* ptr`, `int& ref`)
- **Includes**: Regrouped (ROS → STL → Third-party → Project)
- **Access modifier offset**: -1 (private/protected indented)

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

### Error Handling
- **C++**: Use exceptions sparingly; prefer error codes or `std::optional<T>` for recoverable errors
- **Python**: Raise specific exceptions; avoid bare `except:`
- **ROS**: Use `ROS_ERROR()`, `ROS_WARN()` for logging; return false on failure

### Type Conventions
- Use `const` by default for references and pointers when not modifying
- Prefer `int64_t` for timestamps (microseconds)
- Use `double` for floating-point calculations
- Avoid raw loops; use STL algorithms or Eigen operations

### Class Structure
```cpp
class ConeTracker {
 public:
  ConeTracker() = default;
  explicit ConeTracker(const Config& config);
  bool update(const PointCloud& cloud);

 private:
  struct Impl;  // PImpl idiom for heavy implementations
  std::unique_ptr<Impl> impl_;
};
```

### ROS Node Structure
- Use a class wrapper around the node handle
- Initialize in constructor or `init()` method
- Use callbacks as class methods bound via `ros::Timer` or subscriptions

---

## Testing Guidelines

### Test File Naming
- C++: `test_<feature>.cpp` in `<package>/test/`
- Python: `test_<feature>.py`

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

### Commit Message Format (Conventional Commits)
```
<type>(<scope>): <description>

[optional body]
```

Types: `feat`, `fix`, `chore`, `docs`, `refactor`, `test`

Examples:
```
feat(planning): add skidpad detection
fix(localization): correct IMU time offset
chore(config): update cone trust parameters
```

### PR Description Should Include
- Problem/solution summary
- Impacted packages and launch/config files
- Validation commands run (with results)
- Linked issue/task
- Screenshots for visualization changes
