# RViz Configuration Files

## Overview

This directory contains RViz configuration files for the 2025HUAT Formula Student Driverless system. The configurations are organized by viewing perspective to support different debugging and visualization needs.

## Configuration Files

### 1. ego_perception.rviz (Ego-centric View)

**Purpose:** Perception debugging from vehicle's perspective

**Fixed Frame:** `velodyne` (or `base_link`)

**Visual Effect:** Cones appear to move toward the stationary vehicle

**Display Elements:**
- Raw point cloud (gray, optional)
- PassThrough filtered points (gray)
- Ground-removed points (blue)
- Cone points (orange spheres)
- Bounding boxes (white wireframes)
- Distance labels (red text)
- Confidence information

**Use Cases:**
- Debug perception algorithms
- Tune ground segmentation
- Validate clustering results
- Inspect cone detection accuracy

**Launch:**
```bash
# Standalone perception debugging
roslaunch perception_ros lidar_cluster.launch \
  bag:=/path/to/bag.bag

# Debug mode with ego view
roslaunch fsd_launch debug_perception_location.launch \
  simulation:=true \
  bag:=/path/to/bag.bag \
  rviz_mode:=ego
```

---

### 2. world_planning.rviz (World-centric View)

**Purpose:** Global planning and localization visualization

**Fixed Frame:** `world`

**Visual Effect:** Vehicle moves through the map

**Display Elements:**
- Vehicle model + trajectory trail
- Global cone map (color-coded: blue/yellow/orange)
- Planned path
- Track boundaries
- Current detections (optional, for comparison)

**Use Cases:**
- Debug localization and mapping
- Validate path planning
- Monitor vehicle trajectory
- Inspect global cone map quality

**Launch:**
```bash
# Debug mode with world view (default)
roslaunch fsd_launch debug_perception_location.launch \
  simulation:=true \
  bag:=/path/to/bag.bag

# Or explicitly specify world mode
roslaunch fsd_launch debug_perception_location.launch \
  simulation:=true \
  bag:=/path/to/bag.bag \
  rviz_mode:=world
```

---

### 3. minimal.rviz (Minimal Template)

**Purpose:** Minimal configuration template for quick testing

**Fixed Frame:** `velodyne`

**Display Elements:** Basic axes and grid only

**Use Cases:**
- Quick RViz startup
- Template for new configurations
- Lightweight visualization

**Launch:**
```bash
rviz -d $(find fsd_visualization)/rviz/minimal.rviz
```

---

## Usage Modes

### Single View Mode

Launch with a specific perspective:

```bash
# Ego-centric (perception debugging)
roslaunch fsd_launch debug_perception_location.launch \
  simulation:=true \
  bag:=/path/to/bag.bag \
  rviz_mode:=ego

# World-centric (planning/localization)
roslaunch fsd_launch debug_perception_location.launch \
  simulation:=true \
  bag:=/path/to/bag.bag \
  rviz_mode:=world
```

### Dual View Mode (Recommended)

Launch both perspectives simultaneously for comprehensive debugging:

```bash
roslaunch fsd_launch debug_perception_location.launch \
  simulation:=true \
  bag:=/path/to/bag.bag \
  rviz_mode:=dual
```

This opens two RViz windows:
- **Left window:** Ego-centric perception view
- **Right window:** World-centric planning view

### No RViz Mode

Disable RViz entirely (for headless operation):

```bash
roslaunch fsd_launch debug_perception_location.launch \
  simulation:=true \
  bag:=/path/to/bag.bag \
  rviz_mode:=none
```

---

## Coordinate Frames

### Ego-centric Configurations
- **Fixed Frame:** `velodyne` or `base_link`
- **TF Chain:** `velodyne` → `base_link`
- **Behavior:** Vehicle stays centered, world moves around it

### World-centric Configurations
- **Fixed Frame:** `world`
- **TF Chain:** `world` → `base_link` → `velodyne`
- **Behavior:** Vehicle moves through stationary world

**Important:** Ensure TF tree is complete for proper visualization. Check with:
```bash
rosrun tf view_frames
```

---

## Visualization Topics

### Perception Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Raw LiDAR data |
| `/perception/lidar_cluster/points/passthrough` | `sensor_msgs/PointCloud2` | Filtered points |
| `/perception/lidar_cluster/points/no_ground` | `sensor_msgs/PointCloud2` | Ground-removed points |
| `/perception/lidar_cluster/points/cones` | `sensor_msgs/PointCloud2` | Detected cone points |
| `/fsd/viz/cones` | `visualization_msgs/MarkerArray` | Cone markers |

### Planning Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/fsd/viz/path` | `visualization_msgs/MarkerArray` | Planned path |
| `/fsd/viz/boundaries` | `visualization_msgs/MarkerArray` | Track boundaries |
| `/fsd/viz/vehicle` | `visualization_msgs/MarkerArray` | Vehicle marker |
| `/coneMap` | `autodrive_msgs/HUAT_map` | Global cone map |

---

## Creating Custom Configurations

### Method 1: Start from Template

```bash
# Copy minimal config
cp $(find fsd_visualization)/rviz/minimal.rviz \
   $(find fsd_visualization)/rviz/my_config.rviz

# Edit in RViz
rviz -d $(find fsd_visualization)/rviz/my_config.rviz

# Save: File → Save Config
```

### Method 2: Modify Existing Config

```bash
# Load existing config
rviz -d $(find fsd_visualization)/rviz/ego_perception.rviz

# Make changes in RViz GUI

# Save: File → Save Config As → my_config.rviz
```

---

## Configuration Standards

### Naming Convention
- Use descriptive names: `<perspective>_<purpose>.rviz`
- Examples: `ego_perception.rviz`, `world_planning.rviz`

### Required Elements
All configs should include:
- **Fixed Frame:** Appropriate coordinate frame
- **Grid:** Ground plane reference
- **Axes:** Coordinate system visualization
- **Background:** Dark gray (48, 48, 48) for better contrast

### Color Scheme
| Element | Color (RGB) | Purpose |
|---------|-------------|---------|
| Cones | 255, 165, 0 | Orange for visibility |
| Path | 0, 255, 0 | Green for planned trajectory |
| Boundaries | 0, 0, 255 | Blue for track limits |
| Vehicle | 255, 255, 0 | Yellow for ego vehicle |
| Ground-removed points | 0, 150, 255 | Light blue |

---

## Troubleshooting

### RViz shows blank screen
- Check Fixed Frame matches published TF frames
- Verify topics are publishing: `rostopic list`
- Check TF tree: `rosrun tf view_frames`

### Points not visible in world view
- Ensure `base_link → velodyne` TF is published
- Check if `use_sim_time` is set correctly
- Verify point cloud frame_id matches TF tree

### Dual mode performance issues
- Running two RViz instances is CPU/GPU intensive
- Reduce point cloud density in display settings
- Disable unused visualization elements
- Consider using single view mode on lower-end hardware

### Config changes not taking effect
- Ensure you saved the config file
- Check launch file references correct config path
- Restart RViz completely

---

## Performance Optimization

### For Low-End Systems
- Use single view mode instead of dual
- Reduce point cloud size: `Size (m)` in PointCloud2 display
- Decrease history length for trails
- Lower frame rate in Global Options
- Disable alpha/transparency effects

### For High-End Systems
- Enable dual view mode for comprehensive debugging
- Increase point cloud quality
- Enable anti-aliasing
- Add more visualization elements

---

## Migration Notes

### Removed Configurations
The following configs were consolidated or deprecated:
- `default.rviz` → merged into `minimal.rviz`
- `fsd_default.rviz` → merged into `minimal.rviz`
- `lidar_cluster.rviz` → replaced by `ego_perception.rviz`
- `lidar_cluster_enhanced.rviz` → renamed to `ego_perception.rviz`
- `fsd_main.rviz` → renamed to `world_planning.rviz`
- `global_viz.rviz` → merged into `world_planning.rviz`
- `pointcloud_only.rviz` → functionality in `ego_perception.rviz`
- `display_high.rviz` → deprecated (was in `_deprecated/`)
- `display_skidpad.rviz` → deprecated (was in `_deprecated/`)

### Updated Launch Files
All launch files have been updated to reference new config names:
- `perception_ros/launch/lidar_cluster.launch`
- `fsd_launch/launch/tools/debug_perception_location.launch`
- `fsd_launch/launch/tools/rviz.launch`

---

## Changelog

### 2026-02-01
- Reorganized configs into 3 core files (ego/world/minimal)
- Added dual-view mode support
- Renamed configs for clarity:
  - `lidar_cluster_enhanced.rviz` → `ego_perception.rviz`
  - `fsd_main.rviz` → `world_planning.rviz`
  - `fsd_default.rviz` → `minimal.rviz`
- Removed 6 deprecated/duplicate configs
- Updated all launch file references
- Added `rviz_mode` parameter to debug launch files

### 2025-01-25
- Initial unified RViz configuration management
- Created baseline configs for perception and planning

---

## Contact

For questions or issues with RViz configurations, please contact the FSD visualization team or open an issue in the project repository.
