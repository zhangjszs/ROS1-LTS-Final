#pragma once

#include <iomanip>
#include <sstream>
#include <string>

#include <ros/ros.h>

#include <autodrive_msgs/perf_stats_skeleton.hpp>

namespace localization_ros {

struct LocPerfSample {
  double t_opt_ms = 0.0;        // iSAM2 optimization time
  double t_preint_ms = 0.0;     // IMU preintegration accumulation time
  double t_assoc_ms = 0.0;      // data association time
  double t_prune_ms = 0.0;      // landmark pruning time
  double t_total_ms = 0.0;      // total keyframe processing time
  double landmark_count = 0.0;
  double chi2_normalized = 0.0;
  double cone_match_ratio = 0.0;
  double gnss_availability = 0.0;  // 1.0 if GNSS used this frame, 0.0 otherwise
};

class LocPerfStats {
 public:
  using MetricStats = autodrive_msgs::perf::MetricStats;

  struct Snapshot {
    MetricStats t_opt_ms;
    MetricStats t_preint_ms;
    MetricStats t_assoc_ms;
    MetricStats t_prune_ms;
    MetricStats t_total_ms;
    MetricStats landmark_count;
    MetricStats chi2_normalized;
    MetricStats cone_match_ratio;
    MetricStats gnss_availability;
  };

  LocPerfStats() = default;

  void Configure(const std::string &name, bool enabled, size_t window_size, size_t log_every) {
    window_.Configure(name, enabled, window_size, log_every);
  }

  void Add(const LocPerfSample &sample) {
    if (window_.PushSample(sample)) {
      LogStats();
    }
  }

  Snapshot SnapshotStats() const {
    Snapshot snap;
    snap.t_opt_ms = window_.ComputeStatsFor([](const LocPerfSample &s) { return s.t_opt_ms; });
    snap.t_preint_ms = window_.ComputeStatsFor([](const LocPerfSample &s) { return s.t_preint_ms; });
    snap.t_assoc_ms = window_.ComputeStatsFor([](const LocPerfSample &s) { return s.t_assoc_ms; });
    snap.t_prune_ms = window_.ComputeStatsFor([](const LocPerfSample &s) { return s.t_prune_ms; });
    snap.t_total_ms = window_.ComputeStatsFor([](const LocPerfSample &s) { return s.t_total_ms; });
    snap.landmark_count = window_.ComputeStatsFor([](const LocPerfSample &s) { return s.landmark_count; });
    snap.chi2_normalized = window_.ComputeStatsFor([](const LocPerfSample &s) { return s.chi2_normalized; });
    snap.cone_match_ratio = window_.ComputeStatsFor([](const LocPerfSample &s) { return s.cone_match_ratio; });
    snap.gnss_availability =
        window_.ComputeStatsFor([](const LocPerfSample &s) { return s.gnss_availability; });
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

    AppendMetric(os, "t_opt_ms", window_.ComputeStatsFor([](const LocPerfSample &s) { return s.t_opt_ms; }));
    AppendMetric(os, "t_preint_ms",
                 window_.ComputeStatsFor([](const LocPerfSample &s) { return s.t_preint_ms; }));
    AppendMetric(os, "t_assoc_ms",
                 window_.ComputeStatsFor([](const LocPerfSample &s) { return s.t_assoc_ms; }));
    AppendMetric(os, "t_prune_ms",
                 window_.ComputeStatsFor([](const LocPerfSample &s) { return s.t_prune_ms; }));
    AppendMetric(os, "t_total_ms",
                 window_.ComputeStatsFor([](const LocPerfSample &s) { return s.t_total_ms; }));
    AppendMetric(os, "LM", window_.ComputeStatsFor([](const LocPerfSample &s) { return s.landmark_count; }));
    AppendMetric(os, "chi2",
                 window_.ComputeStatsFor([](const LocPerfSample &s) { return s.chi2_normalized; }));
    AppendMetric(os, "match",
                 window_.ComputeStatsFor([](const LocPerfSample &s) { return s.cone_match_ratio; }));
    AppendMetric(os, "gnss",
                 window_.ComputeStatsFor([](const LocPerfSample &s) { return s.gnss_availability; }));

    ROS_INFO_STREAM(os.str());
  }

  autodrive_msgs::perf::RollingStatsWindow<LocPerfSample> window_;
};

}  // namespace localization_ros
