# Copilot Instructions for 2025huat (ROS1 Project)

## Project Architecture
- **Domain:** Formula Student Driverless (FSD) Autonomous Racing.
- **Framework:** ROS1 Noetic on Linux.
- **Build System:** `catkin tools` (evidenced by `build/catkin_tools_prebuild`).
- **Core Components:**
  - **Perception:** `lidar_cluster` (Cone detection), `image_pointcloud_fusion`, `skidpad_detection`.
  - **Planning:** `high_speed_tracking` (Delaunay triangulation based path planner).
  - **Control:** `control` (Path tracking & command generation).
  - **Interface:** `ros_vehicle_interface` (UDP bridge to vehicle CAN), `common_msgs`.
  - **Startup:** `autoStartGkj/` contains shell scripts controlling the node lifecycle.

## Critical Developer Workflows
- **Build:** Use `catkin build` (not `catkin_make`).
  ```bash
  catkin build [package_name]
  source devel/setup.bash
  ```
- **Startup:** The system uses `gnome-terminal` based startup scripts in `autoStartGkj/`.
  - Entry point: `bash autoStartGkj/start.sh`
  - Logic: Writes state '0' or '2025' to `autoStartGkj/command` to manage state transitions.
- **Visualization:** `race_rviz_viz` package provides pre-configured RViz setups (see `viz.launch`).

## Project-Specific Conventions
### 1. Topics & Messages (`common_msgs`)
- **Map/Cones:** `/coneMap` uses `common_msgs/HUAT_map`.
- **Vehicle State:** `/Carstate` uses `common_msgs/HUAT_Carstate`.
- **Control Cmd:** `/vehcileCMDMsg` (Note "vehcile" Typo) uses `common_msgs/vehicle_cmd`.
- **Global Path:** `/path_global` uses `nav_msgs/Path`.

### 2. Coordinate Frames
- **Global:** `velodyne` (often acts as map frame) or `global_frame`.
- **Ego:** `base_link`.
- **Sensor:** `velodyne`.

### 3. File Structure
- `src/` contains all ROS packages.
- `logs/` is actively used for storing run logs (outside standard `~/.ros`).

## Integration Points
- **Vehicle Interface:** `ros_vehicle_interface` communicates via UDP. The "App" class (`src/ros_vehicle_interface/src/App.cpp`) manages the network socket.
- **Path Planner:** `high_speed_tracking` implements color-blind path finding using `Bowyer-Watson` algorithm for triangulation.

## Coding Patterns
- **Loops:** Preference for `ros::Rate` loops inside `runAlgorithm` methods (e.g., `skidpad_detection`, `lidar_cluster`).
- **Private Params:** Use `ros::NodeHandle nh("~")` for loading parameters.
