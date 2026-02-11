# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0] - 2026-02-11 - M5 Unified Planner Release

### Added
- **Unified Planning Pipeline** - New `planning_pipeline_node` that consolidates line, skidpad, and high-speed tracking backends
- `speed_profile.hpp` - Core library for velocity profile computation with curvature-based speed limiting
- `planning_pipeline.launch` - Unified entry point for all planning missions
- A/B testing framework for comparing unified vs legacy planners

### Changed
- **Default planner switched to `unified`** across all mission launch files:
  - `trackdrive.launch`: `high_speed` → `unified`
  - `acceleration.launch`: `line` → `unified`
  - `autocross.launch`: `high_speed` → `unified`
  - `skidpad.launch`: `skidpad` → `unified`
  - `planning.launch`: `high_speed` → `unified`
- Performance tuning for unified pipeline acceptance:
  - `max_treeSearch_time`: 0.12s → 0.10s
  - `search_radius`: 18.0m → 15.0m
  - `max_distance` (line): 175.0m → 150.0m
  - `perf_stats_enable`: true

### Fixed
- Topic forwarding for line/skidpad backends in unified pipeline - now correctly publish to `planning/pathlimits`

### Deprecated
- Legacy standalone launch files remain available via explicit `planner:=<legacy>` parameter:
  - `high_speed` - Original high-speed tracking node
  - `line` - Original line detection node
  - `skidpad` - Original skidpad detection node

### Migration Guide

#### For Users
```bash
# New default behavior - uses unified planner
roslaunch fsd_launch trackdrive.launch simulation:=true bag:=track.bag

# Explicit legacy fallback (if needed)
roslaunch fsd_launch trackdrive.launch planner:=high_speed
roslaunch fsd_launch acceleration.launch planner:=line
roslaunch fsd_launch skidpad.launch planner:=skidpad
```

#### For Developers
See `docs/PLANNING_M5_VALIDATION_AND_MIGRATION_PLAN.md` for:
- Complete validation procedures
- Automated acceptance testing
- Rollback procedures
- Legacy code cleanup roadmap

### Metrics
**M5 Acceptance Criteria (All Passed):**
| Mission | PathLimits Freq | PathLimits Max Interval | Cmd Freq | Cmd Max Interval |
|---------|-----------------|-------------------------|----------|------------------|
| Trackdrive | 9.97 Hz | 0.153s | 10.0 Hz | 0.106s |
| Acceleration | 19.99 Hz | 0.053s | 9.997 Hz | 0.104s |
| Skidpad | 20.01 Hz | 0.056s | 10.0 Hz | 0.107s |

**Targets:** PathLimits ≥8Hz, ≤0.25s | Cmd ≥10Hz, ≤0.20s

---

## [1.0.0] - 2026-02-08

### Added
- Initial release with separate planning nodes:
  - `high_speed_tracking_node` - Trackdrive/autocross missions
  - `line_detection_node` - Acceleration mission
  - `skidpad_detection_node` - Skidpad mission
- Core planning algorithms in `planning_core`
- ROS wrappers in `planning_ros`
- Mission-specific launch files

[1.1.0]: https://github.com/your-org/2025huat/compare/v1.0.0...m5-baseline
[1.0.0]: https://github.com/your-org/2025huat/releases/tag/v1.0.0
