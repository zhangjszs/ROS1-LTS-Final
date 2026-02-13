#pragma once

// DEPRECATED: This header previously contained perception-specific PerfSample
// and PerfStats types.  Those have been moved to their owning packages:
//   - perception_ros/include/perception_ros/perf_stats.hpp
//   - planning_ros/include/high_speed_tracking/utils/PerfStats.hpp
//   - localization_ros/include/localization_ros/localization_perf_stats.hpp
//
// Use the generic RollingStatsWindow<T> template directly:
//   #include <autodrive_msgs/perf_stats_skeleton.hpp>

#include <autodrive_msgs/perf_stats_skeleton.hpp>
