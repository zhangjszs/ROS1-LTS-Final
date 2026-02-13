#pragma once

#include <iomanip>
#include <sstream>

#include <ros/ros.h>

#include <autodrive_msgs/perf_stats_skeleton.hpp>

namespace perception_ros {

struct PerfSample {
  double t_pass_ms = 0.0;
  double t_ground_ms = 0.0;
  double t_cluster_ms = 0.0;
  double t_total_ms = 0.0;
  double n_points = 0.0;
  double n_clusters = 0.0;
  double n_detections = 0.0;
  double bytes_pub = 0.0;
};

class PerfStats {
 public:
  using MetricStats = autodrive_msgs::perf::MetricStats;

  struct Snapshot {
    MetricStats t_pass_ms;
    MetricStats t_ground_ms;
    MetricStats t_cluster_ms;
    MetricStats t_total_ms;
    MetricStats n_points;
    MetricStats n_clusters;
    MetricStats n_detections;
    MetricStats bytes_pub;
  };

  PerfStats() = default;

  void Configure(const std::string &name, bool enabled, size_t window_size, size_t log_every) {
    window_.Configure(name, enabled, window_size, log_every);
  }

  void Add(const PerfSample &sample) {
    if (window_.PushSample(sample)) {
      LogStats();
    }
  }

  Snapshot SnapshotStats() const {
    Snapshot snap;
    snap.t_pass_ms = window_.ComputeStatsFor([](const PerfSample &s) { return s.t_pass_ms; });
    snap.t_ground_ms = window_.ComputeStatsFor([](const PerfSample &s) { return s.t_ground_ms; });
    snap.t_cluster_ms = window_.ComputeStatsFor([](const PerfSample &s) { return s.t_cluster_ms; });
    snap.t_total_ms = window_.ComputeStatsFor([](const PerfSample &s) { return s.t_total_ms; });
    snap.n_points = window_.ComputeStatsFor([](const PerfSample &s) { return s.n_points; });
    snap.n_clusters = window_.ComputeStatsFor([](const PerfSample &s) { return s.n_clusters; });
    snap.n_detections = window_.ComputeStatsFor([](const PerfSample &s) { return s.n_detections; });
    snap.bytes_pub = window_.ComputeStatsFor([](const PerfSample &s) { return s.bytes_pub; });
    return snap;
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

    AppendMetric(os, "t_pass_ms", window_.ComputeStatsFor([](const PerfSample &s) { return s.t_pass_ms; }));
    AppendMetric(os, "t_ground_ms", window_.ComputeStatsFor([](const PerfSample &s) { return s.t_ground_ms; }));
    AppendMetric(os, "t_cluster_ms", window_.ComputeStatsFor([](const PerfSample &s) { return s.t_cluster_ms; }));
    AppendMetric(os, "t_total_ms", window_.ComputeStatsFor([](const PerfSample &s) { return s.t_total_ms; }));
    AppendMetric(os, "N", window_.ComputeStatsFor([](const PerfSample &s) { return s.n_points; }));
    AppendMetric(os, "K", window_.ComputeStatsFor([](const PerfSample &s) { return s.n_clusters; }));
    AppendMetric(os, "D", window_.ComputeStatsFor([](const PerfSample &s) { return s.n_detections; }));
    AppendMetric(os, "bytes", window_.ComputeStatsFor([](const PerfSample &s) { return s.bytes_pub; }));

    ROS_INFO_STREAM(os.str());
  }

  autodrive_msgs::perf::RollingStatsWindow<PerfSample> window_;
};

}  // namespace perception_ros

// Backward-compatible global aliases.
using PerfSample = perception_ros::PerfSample;
using PerfStats = perception_ros::PerfStats;
