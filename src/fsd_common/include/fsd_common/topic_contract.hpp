#pragma once

namespace fsd_common {
namespace topic_contract {

// ── Core data flow ──────────────────────────────────────────────
inline constexpr const char *kVehicleCmd = "vehicle/cmd";
inline constexpr const char *kCarState = "localization/car_state";
inline constexpr const char *kPathLimits = "planning/pathlimits";
inline constexpr const char *kApproachingGoal = "planning/skidpad/approaching_goal";

// ── Perception / Localization ───────────────────────────────────
inline constexpr const char *kConeDetections = "perception/lidar_cluster/detections";
inline constexpr const char *kConeMap = "localization/cone_map";

// ── Vehicle interface ───────────────────────────────────────────
inline constexpr const char *kInsMsg = "sensors/ins";
inline constexpr const char *kVehicleStatus = "vehicle/status";

// ── Diagnostics ─────────────────────────────────────────────────
inline constexpr const char *kDiagnosticsGlobal = "/diagnostics";
inline constexpr const char *kPerceptionDiagnostics = "perception/diagnostics";
inline constexpr const char *kControlDiagnostics = "control/diagnostics";
inline constexpr const char *kLocalizationDiagnostics = "localization/diagnostics";
inline constexpr const char *kPlanningDiagnostics = "planning/diagnostics";
inline constexpr const char *kLocalizationFgStatus = "localization/fg_status";

} // namespace topic_contract

namespace frame_contract {

inline constexpr const char *kWorld = "world";
inline constexpr const char *kBaseLink = "base_link";
inline constexpr const char *kVelodyne = "velodyne";
inline constexpr const char *kImu = "imu";

} // namespace frame_contract

namespace stamp_contract {

/// Maximum acceptable drift between sensor stamp and receive time [s].
inline constexpr double kMaxStampDriftSec = 1.0;

} // namespace stamp_contract
} // namespace fsd_common
