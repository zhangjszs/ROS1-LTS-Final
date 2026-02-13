#pragma once

#include <iomanip>
#include <sstream>

#include <ros/ros.h>

#include <autodrive_msgs/perf_stats_skeleton.hpp>

namespace planning_ros {

struct PerfSample {
  double t_delaunay_ms = 0.0;
  double t_way_ms = 0.0;
  double t_total_ms = 0.0;
  double n_points = 0.0;
  double n_triangles = 0.0;
  double n_edges = 0.0;
  double bytes_pub = 0.0;
};

class PerfStats {
 public:
  using MetricStats = autodrive_msgs::perf::MetricStats;

  PerfStats() = default;

  void Configure(const std::string &name, bool enabled, size_t window_size, size_t log_every) {
    window_.Configure(name, enabled, window_size, log_every);
  }

  void Add(const PerfSample &sample) {
    if (window_.PushSample(sample)) {
      LogStats();
    }
  }

 private:
  static void AppendMetric(std::ostringstream &os, const char *name, const MetricStats &stats) {
    os << name << "{mean=" << stats.mean << ",p50=" << stats.p50 << ",p95=" << stats.p95
       << ",p99=" << stats.p99 << ",max=" << stats.max << "} ";
  }

  void LogStats() const {
    std::ostringstream os;
    os.setf(std::ios::fixed, std::ios::floatfield);
    os << std::setprecision(3);
    os << "[perf] node=" << window_.Name() << " window=" << window_.SampleCount() << " ";

    AppendMetric(os, "t_delaunay_ms",
                 window_.ComputeStatsFor([](const PerfSample &s) { return s.t_delaunay_ms; }));
    AppendMetric(os, "t_way_ms", window_.ComputeStatsFor([](const PerfSample &s) { return s.t_way_ms; }));
    AppendMetric(os, "t_total_ms", window_.ComputeStatsFor([](const PerfSample &s) { return s.t_total_ms; }));
    AppendMetric(os, "N", window_.ComputeStatsFor([](const PerfSample &s) { return s.n_points; }));
    AppendMetric(os, "T", window_.ComputeStatsFor([](const PerfSample &s) { return s.n_triangles; }));
    AppendMetric(os, "E", window_.ComputeStatsFor([](const PerfSample &s) { return s.n_edges; }));
    AppendMetric(os, "bytes", window_.ComputeStatsFor([](const PerfSample &s) { return s.bytes_pub; }));

    ROS_INFO_STREAM(os.str());
  }

  autodrive_msgs::perf::RollingStatsWindow<PerfSample> window_;
};

}  // namespace planning_ros

// Backward-compatible global aliases.
using PerfSample = planning_ros::PerfSample;
using PerfStats = planning_ros::PerfStats;
