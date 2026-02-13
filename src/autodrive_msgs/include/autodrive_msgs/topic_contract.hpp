#pragma once

namespace autodrive_msgs {
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
inline constexpr const char *kInsMsg = "insMsg";
inline constexpr const char *kVehicleStatus = "vehicleStatusMsg";

// ── Diagnostics ─────────────────────────────────────────────────
inline constexpr const char *kDiagnosticsGlobal = "/diagnostics";
inline constexpr const char *kControlDiagnostics = "control/diagnostics";
inline constexpr const char *kLocalizationDiagnostics = "localization/diagnostics";
inline constexpr const char *kPlanningDiagnostics = "planning/diagnostics";
inline constexpr const char *kLocalizationFgStatus = "localization/fg_status";

// ── Legacy (deprecated – remove after 2025 season) ──────────────
inline constexpr const char *kVehicleCmdLegacy = "vehcileCMDMsg";
inline constexpr const char *kCarStateLegacy = "/Carstate";
inline constexpr const char *kApproachingGoalLegacy = "/skidpad_detection_node/approaching_goal";

} // namespace topic_contract

namespace frame_contract {

inline constexpr const char *kWorld = "world";
inline constexpr const char *kBaseLink = "base_link";
inline constexpr const char *kVelodyne = "velodyne";
inline constexpr const char *kImu = "imu";

} // namespace frame_contract
} // namespace autodrive_msgs
