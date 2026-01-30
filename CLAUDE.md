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
- `autodrive_msgs/` - Custom message definitions (HUAT_map, HUAT_Carstate, vehicle_cmd)
- `fsd_launch/` - Unified launch file organization
- `fsd_visualization/` - Visualization nodes and RViz configs
- `ins/` - INS message compatibility bridge

### Launch File Organization

Launch files are organized hierarchically in `fsd_launch/`:

```
fsd_launch/launch/
├── missions/          # Mission-level (user entry points)
│   ├── trackdrive.launch
│   ├── skidpad.launch
│   ├── acceleration.launch
│   └── autocross.launch
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
roslaunch fsd_launch missions/trackdrive.launch simulation:=true bag:=/path/to/bag.bag

# With loop playback
roslaunch fsd_launch missions/trackdrive.launch simulation:=true bag:=/path/to/bag.bag loop:=true

# Different RViz modes
roslaunch fsd_launch missions/trackdrive.launch simulation:=true bag:=/path/to/bag.bag rviz_mode:=dual
roslaunch fsd_launch missions/trackdrive.launch simulation:=true bag:=/path/to/bag.bag rviz_mode:=pointcloud
roslaunch fsd_launch missions/trackdrive.launch simulation:=true bag:=/path/to/bag.bag rviz_mode:=global

# Custom playback rate
roslaunch fsd_launch missions/trackdrive.launch simulation:=true bag:=/path/to/bag.bag rate:=0.5
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
| TrackDrive | `roslaunch fsd_launch missions/trackdrive.launch` | High-speed lap tracking |
| Skidpad | `roslaunch fsd_launch missions/skidpad.launch` | Figure-8 maneuver |
| Acceleration | `roslaunch fsd_launch missions/acceleration.launch` | Straight-line acceleration |
| Autocross | `roslaunch fsd_launch missions/autocross.launch` | Complex track navigation |

## Key Topics and Messages

### Custom Messages (autodrive_msgs)

- `/coneMap` - `autodrive_msgs/HUAT_map` - Detected cone positions
- `/Carstate` - `autodrive_msgs/HUAT_Carstate` - Vehicle state (position, velocity, heading)
- `/vehcileCMDMsg` - `autodrive_msgs/vehicle_cmd` - Control commands (note: typo in topic name is intentional)

### Standard Messages

- `/velodyne_points` - `sensor_msgs/PointCloud2` - Raw LiDAR point cloud
- `/path_global` - `nav_msgs/Path` - Global planned path
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
6. Test with rosbag: `roslaunch fsd_launch missions/trackdrive.launch simulation:=true bag:=...`

### Debugging

```bash
# Launch with debug tools
roslaunch fsd_launch tools/debug.launch mission:=trackdrive bag:=/path/to/bag.bag

# Check topics
rostopic list
rostopic echo /coneMap
rostopic hz /velodyne_points

# Check transforms
rosrun tf tf_echo velodyne base_link

# Visualize in RViz
roslaunch fsd_launch tools/rviz.launch rviz_mode:=dual
```

## Important Notes

### Topic Name Typo

The control command topic is `/vehcileCMDMsg` (note "vehcile" typo). This is intentional for compatibility with existing vehicle interface.

### Startup Script Dependencies

`autoStartGkj/start.sh` expects:
- `$HOME/Driver/pbox_node_dirve-V3.0.5-20240412/` - INS driver workspace
- `$HOME/camera/` - Camera driver workspace
- These paths may need adjustment for your environment

### Configuration Files

Each package maintains its own config files:
- `perception_ros/config/lidar_cluster.yaml`
- `planning_ros/config/line_detection.yaml`
- `planning_ros/config/skidpad_detection.yaml`
- `planning_ros/config/high_speed_tracking.yaml`
- `control_ros/config/controllers.yaml`
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
- Check that rosbag contains required topics: `/velodyne_points`, `/Carstate`
- Verify coordinate frame transforms are published correctly
- Check parameter files are loaded (use `rosparam list`)
