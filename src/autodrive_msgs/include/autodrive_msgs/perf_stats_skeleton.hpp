#pragma once
// Redirect: migrated to fsd_common. This header will be removed in a future release.
#include <fsd_common/perf_stats_skeleton.hpp>
// Re-export into old namespace for backward compatibility
namespace autodrive_msgs {
  namespace perf {
    using fsd_common::perf::MetricStats;
    using fsd_common::perf::RollingStatsWindow;
  }
}
