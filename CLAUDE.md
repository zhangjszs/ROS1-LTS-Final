# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

2025HUAT is a Formula Student Driverless (FSD) autonomous racing system built on ROS1 Noetic. The project uses a **Core + ROS Wrapper** architecture to decouple ROS middleware from algorithm logic.

## Build System

**Build tool:** `catkin_tools` (NOT `catkin_make`)

```bash
# Build all packages
cd ~/2025huat
catkin build

# Build specific package
catkin build <package_name>

# Clean build
catkin clean -y
catkin build

# Source workspace
source devel/setup.bash
```

## Testing

```bash
# Run all tests
catkin run_tests

# Run tests for specific package
catkin run_tests <package_name>

# Alternative test command
catkin test
```

## Architecture

### Core + ROS Wrapper Pattern

The codebase separates algorithm logic (core) from ROS integration (ros):

- **`*_core/`** packages: Pure C++ algorithms with no ROS dependencies (PCL, Eigen only)
- **`*_ros/`** packages: ROS wrappers that handle topics, services, and parameters

Core packages:
- `perception_core/` - LiDAR clustering, ground segmentation, cone detection
- `planning_core/` - Line detection (Hough transform), skidpad detection, path generation
- `control_core/` - Pure pursuit, PID controllers (line, high-speed, skidpad, test)
- `localization_core/` - State estimation and localization algorithms
- `vehicle_interface_core/` - Vehicle communication protocol logic
- `vehicle_racing_num_core/` - Race number management logic

ROS packages:
- `perception_ros/` - Wraps perception_core with ROS topics
- `planning_ros/` - Wraps planning_core with ROS topics
- `control_ros/` - Wraps control_core with ROS topics
- `localization_ros/` - Wraps localization_core with ROS topics
- `vehicle_interface_ros/` - UDP bridge to vehicle CAN bus
- `vehicle_racing_num_ros/` - Race number ROS interface

Infrastructure:
- `autodrive_msgs/` - Custom message definitions (`HUAT_ConeDetections`, `HUAT_CarState`, `HUAT_PathLimits`, `HUAT_VehicleCmd`)
- `fsd_launch/` - Unified launch file organization
- `fsd_visualization/` - Visualization nodes and RViz configs
- `ins/` - INS message compatibility bridge

### Launch File Organization

Launch files are organized hierarchically in `fsd_launch/`:

```
fsd_launch/launch/
├── trackdrive.launch   # Mission-level (user entry point)
├── skidpad.launch
├── acceleration.launch
├── autocross.launch
├── ebs_test.launch
├── simulation/
├── subsystems/        # Subsystem-level (internal)
│   ├── perception.launch
│   ├── localization.launch
│   ├── planning.launch
│   ├── control.launch
│   └── vehicle.launch
└── tools/             # Utility tools
    ├── rosbag_play.launch
    ├── rviz.launch
    ├── topic_bridge.launch
    └── debug.launch
```

## Running the System

### Simulation Mode (rosbag playback)

```bash
# Basic simulation
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag

# With loop playback
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag loop:=true

# Different RViz modes
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag rviz_mode:=dual
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag rviz_mode:=pointcloud
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag rviz_mode:=global

# Custom playback rate
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=/path/to/bag.bag rate:=0.5
```

### Real Vehicle Mode

```bash
bash autoStartGkj/start.sh
```

The startup script:
1. Launches roscore
2. Starts vehicle racing number node
3. Starts vehicle interface (UDP bridge)
4. Launches sensor drivers (LiDAR, camera, INS)
5. Waits for command file to trigger main system launch

State control: Write '2025' to `autoStartGkj/command` to trigger trackdrive launch.

### Mission Modes

| Mission | Launch Command | Description |
|---------|---------------|-------------|
| TrackDrive | `roslaunch fsd_launch trackdrive.launch` | High-speed lap tracking |
| Skidpad | `roslaunch fsd_launch skidpad.launch` | Figure-8 maneuver |
| Acceleration | `roslaunch fsd_launch acceleration.launch` | Straight-line acceleration |
| Autocross | `roslaunch fsd_launch autocross.launch` | Complex track navigation |

## Key Topics and Messages

### Custom Messages (autodrive_msgs)

- `/perception/lidar_cluster/detections` - `autodrive_msgs/HUAT_ConeDetections` - Cone detections
- `/localization/car_state` - `autodrive_msgs/HUAT_CarState` - Vehicle state
- `/planning/pathlimits` - `autodrive_msgs/HUAT_PathLimits` - Planning output path limits
- `/vehicle/cmd` - `autodrive_msgs/HUAT_VehicleCmd` - Control commands

### Standard Messages

- `/velodyne_points` - `sensor_msgs/PointCloud2` - Raw LiDAR point cloud
- `/fsd/viz/*` - `visualization_msgs/Marker` - Visualization markers for RViz

## Coordinate Frames

- **Global frame:** `velodyne` or `global_frame` (acts as map frame)
- **Ego frame:** `base_link` (vehicle center)
- **Sensor frame:** `velodyne` (LiDAR sensor)

## Coding Patterns

### ROS Node Structure

Most ROS wrapper nodes follow this pattern:

```cpp
class NodeWrapper {
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_("~");  // For private parameters

  // Core algorithm instance
  CoreAlgorithm core_;

  void runAlgorithm() {
    ros::Rate rate(loop_rate_);
    while (ros::ok()) {
      // Process with core algorithm
      core_.Process();
      ros::spinOnce();
      rate.sleep();
    }
  }
};
```

### Parameter Loading

Use private node handle for parameters:

```cpp
ros::NodeHandle private_nh("~");
private_nh.param<double>("param_name", param_var, default_value);
```

Configuration files are in each package's `config/` directory.

## Planning Algorithm Details

### High-Speed Tracking (planning_core)

Uses Delaunay triangulation (Bowyer-Watson algorithm) for color-blind path planning:
1. Triangulate cone positions
2. Find center path through triangulation
3. Generate smooth trajectory

### Line Detection (planning_core)

Hough transform-based approach:
1. Detect left/right track boundaries
2. Calculate center line
3. Detect finish line for lap completion

### Skidpad Detection (planning_core)

Circle fitting for figure-8 patterns:
1. Fit circles to cone clusters
2. Generate figure-8 path
3. Optimize for smooth transitions

## Control System

Controllers in `control_core/`:

- **LineController** - Basic path tracking with pure pursuit
- **HighSpeedController** - Optimized for high-speed stability
- **SkidpadController** - Specialized for tight figure-8 maneuvers
- **TestController** - Development and testing

PID parameters for steering control:
- `angle_kp`, `angle_ki`, `angle_kd` - PID gains
- `angle_kv` - Velocity-dependent gain
- `angle_kl` - Lookahead distance gain
- `steering_delta_max` - Maximum steering rate

## Vehicle Interface

`vehicle_interface_ros/` communicates with vehicle via UDP:
- Sends control commands (steering, throttle, brake)
- Receives vehicle state (velocity, position, IMU)
- Protocol implementation in `App.cpp`

## Development Workflow

### Adding New Features

1. Implement algorithm in appropriate `*_core/` package (no ROS dependencies)
2. Add unit tests in `*_core/test/`
3. Create ROS wrapper in corresponding `*_ros/` package
4. Add launch file to `fsd_launch/` if needed
5. Update visualization in `fsd_visualization/` if needed

### Modifying Existing Code

1. Read the core package README to understand algorithm
2. Check existing tests in `test/` directory
3. Make changes to core logic first
4. Update ROS wrapper if interface changes
5. Run tests: `catkin run_tests <package_name>`
6. Test with rosbag: `roslaunch fsd_launch trackdrive.launch simulation:=true bag:=...`

### Debugging

```bash
# Launch with debug tools
roslaunch fsd_launch tools/debug.launch mission:=trackdrive bag:=/path/to/bag.bag

# Check topics
rostopic list
rostopic echo /perception/lidar_cluster/detections
rostopic hz /velodyne_points

# Check transforms
rosrun tf tf_echo velodyne base_link

# Visualize in RViz
roslaunch fsd_launch tools/rviz.launch rviz_mode:=dual
```

## Important Notes

### Legacy Topic Compatibility

Canonical command topic is `/vehicle/cmd`.  
Legacy topics (`/vehcileCMDMsg`, `/Carstate`) are optional compatibility paths and disabled by default.

### Startup Script Dependencies

`autoStartGkj/start.sh` expects:
- `$HOME/Driver/pbox_node_dirve-V3.0.5-20240412/` - INS driver workspace
- `$HOME/camera/` - Camera driver workspace
- These paths may need adjustment for your environment

### Configuration Files

Each package maintains its own config files:
- `perception_ros/config/lidar_base.yaml`
- `perception_ros/config/lidar_track.yaml`
- `perception_ros/config/lidar_accel.yaml`
- `perception_ros/config/lidar_skidpad.yaml`
- `planning_ros/config/line_detection.yaml`
- `planning_ros/config/skidpad_detection.yaml`
- `planning_ros/config/high_speed_tracking.yml`
- `control_ros/config/param.yaml`
- `localization_ros/config/location.yaml`
- `localization_ros/config/state_estimator.yaml`

### Deprecated Code

The `_deprecated/` directory contains old code kept for reference. Do not use or modify.

## Project Standards

- **C++ Standard:** C++17
- **ROS Version:** Noetic
- **License:** BSD-3-Clause
- **Package Version:** 1.0.0 (all packages unified)
- **Code Formatting:** `.clang-format` in root directory

## CI/CD and Quality Assurance

### Continuous Integration

The project uses GitHub Actions for automated testing:

- **ROS CI**: Builds all packages and runs tests on every push/PR
- **Code Coverage**: Generates coverage reports using gcov/lcov
- **Static Analysis**: Runs cppcheck and clang-tidy for code quality

### Running Static Analysis Locally

```bash
# Install tools
sudo apt-get install cppcheck clang-tidy

# Run cppcheck
cppcheck --enable=warning,style,performance,portability \
  --suppressions-list=.cppcheck-suppressions.txt \
  src/

# Run clang-tidy (after building with compile commands)
catkin build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
find src/perception_core -name "*.cpp" | xargs clang-tidy -p build
```

### Code Coverage

```bash
# Build with coverage flags
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="--coverage" -DCMAKE_C_FLAGS="--coverage"

# Run tests
catkin run_tests

# Generate coverage report
lcov --directory build --capture --output-file coverage.info
lcov --remove coverage.info '/usr/*' '/opt/*' '*/test/*' --output-file coverage.info
genhtml coverage.info --output-directory coverage_html
```

## Common Issues

### Build Failures

```bash
# Clean and rebuild
catkin clean -y
catkin build

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Runtime Issues

- Ensure all sensor drivers are running before launching main system
- Check that rosbag contains required topics: `/velodyne_points`, `/INS/ASENSING_INS`
- Verify coordinate frame transforms are published correctly
- Check parameter files are loaded (use `rosparam list`)

### ROS Topic Name Pitfall

**Never pass tilde (`~`) names to `NodeHandle` methods directly.** `~/foo` is only valid as a constructor argument, not as a topic string passed to `advertise`/`subscribe`/`publish`. Use absolute paths (`/node_name/foo`) or relative paths (`foo`) instead.

```cpp
// WRONG — throws ros::InvalidNameException at runtime
nh.advertise<Msg>("~/diagnostics", 1);

// CORRECT
nh.advertise<Msg>("my_node/diagnostics", 1);  // relative
nh.advertise<Msg>("/my_node/diagnostics", 1); // absolute
```

### Available Test Rosbags

| File | Size | Duration | Topics |
|------|------|----------|--------|
| `~/rosbag/track.bag` | 5.1G | 121s | `/velodyne_points`, `/INS/ASENSING_INS`, `/resize_img_out` |
| `~/rosbag/skidpad.bag` | 6.2G | — | Full sensor suite |
| `~/rosbag/accel.bag` | 995M | — | Full sensor suite |
| `~/rosbag/2024-10-17-01-19-05.bag` | 681M | 89s | LiDAR only (no INS) |

`track.bag` is the recommended bag for full-stack trackdrive validation.

## Observability Patterns

### DiagnosticsHelper

All subsystems publish health via `autodrive_msgs::DiagnosticsHelper`. Read live diagnostics:

```bash
# All diagnostics (raw)
rostopic echo /diagnostics

# Aggregated by subsystem (requires diagnostic_aggregator running)
rostopic echo /diagnostics_agg

# Per-subsystem topics
rostopic echo /localization/diagnostics
rostopic echo /control/diagnostics
rostopic echo /perception/lidar_cluster/perception/diagnostics
```

Key diagnostic fields to check after a rosbag run:
- `mapper_state` — TRACKING / DEGRADED / INS_ONLY
- `e2e_latency_mean_ms` / `e2e_latency_max_ms` — end-to-end pipeline latency
- `stop_reason` — why control stopped (RUNNING / MISSION_COMPLETE / INPUT_TIMEOUT)
- `reloc_attempt_count` / `reloc_success_count` — factor graph relocalization stats
- `n_detections`, `t_total_ms` — perception throughput

### PathLimits Timestamp Convention

`HUAT_PathLimits` carries two timestamps (defined in `planning_ros/include/planning_ros/contract_utils.hpp`):
- `header.stamp` — original sensor/LiDAR timestamp (for e2e latency measurement)
- `stamp` — wall-clock time when planning published the message

Always use `FinalizePathLimitsMessage(msg, input_stamp, frame_id)` when constructing a PathLimits message.

## Audit Comment Convention

Code changes from the system audit use `// B<N>:` tags for traceability:
- B1–B6: P0 safety fixes (array bounds, NaN guards, message sync, timeouts)
- B7–B22: P1 improvements (observability, documentation, latency tracking)

When adding new audit-style fixes, continue the numbering sequence.
