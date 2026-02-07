#pragma once
// ============================================================
// track_constraints.hpp
// 赛道约束字典 - Header-only C++ 接口
// ============================================================
// 本文件将 docs/constraints/*.yaml 中的约束参数编码为 C++ 结构体，
// 供 planning_core / localization_core 在算法中直接引用。
//
// 设计原则：
//   - 无外部依赖（不依赖 yaml-cpp / ROS），保持 core 包纯净
//   - 参数值与 YAML 文件一一对应，修改时需同步更新
//   - 提供 mission-specific 预设，通过 getMissionConstraints() 获取
// ============================================================

#include <cmath>
#include <string>

namespace planning_core {

// ---- 通用约束 (COM_*) ----
namespace cone_type {
  constexpr int BLUE         = 0;
  constexpr int YELLOW       = 1;
  constexpr int ORANGE_SMALL = 2;
  constexpr int ORANGE_BIG   = 3;
  constexpr int NONE         = 4;
}  // namespace cone_type

namespace cone_semantic {
  constexpr int LEFT_BOUNDARY  = cone_type::BLUE;
  constexpr int RIGHT_BOUNDARY = cone_type::YELLOW;
}  // namespace cone_semantic

// ---- 加速赛约束 (ACC_*) ----
struct AccelerationConstraints {
  double track_width         = 3.0;    // ACC_01 [m]
  double accel_zone_length   = 75.0;   // ACC_02 [m]
  double brake_zone_length   = 100.0;  // ACC_03 [m]
  double cone_spacing        = 5.0;    // ACC_04 [m]
  double start_timing_gap    = 0.3;    // ACC_05 [m]
};

// ---- 八字赛约束 (SKP_*) ----
struct SkidpadConstraints {
  double cone_circle_radius  = 15.25;  // SKP_01 [m] 锥桶圆半径
  double driving_radius      = 9.125;  // SKP_02 [m] 车辆行驶半径
  double center_distance     = 18.25;  // SKP_03 [m] 圆心间距
  double track_width         = 3.0;    // SKP_04 [m]
  int    cones_inner         = 16;     // SKP_05 每圆内侧锥桶数
  int    cones_outer         = 16;     // SKP_05 每圆外侧锥桶数
  int    total_laps          = 4;      // SKP_06
  int    timed_laps[2]       = {2, 4}; // SKP_06 计时圈
};

// ---- 高速避障约束 (AUT_*) ----
struct AutocrossConstraints {
  double min_track_width     = 3.5;    // AUT_01 [m]
  double ref_lap_length      = 805.0;  // AUT_02 [m]
  double ref_speed_min       = 11.11;  // AUT_03 [m/s] (40 km/h)
  double ref_speed_max       = 13.33;  // AUT_03 [m/s] (48 km/h)
  double hairpin_outer_r_min = 9.0;    // AUT_04 [m]
  double hairpin_diam_min    = 23.0;   // AUT_04 [m]
  double hairpin_diam_max    = 45.0;   // AUT_04 [m]
  double slalom_spacing_min  = 7.62;   // AUT_05 [m]
  double slalom_spacing_max  = 12.19;  // AUT_05 [m]
  double straight_after_hairpin = 60.0; // AUT_06 [m]
  double straight_after_sweeper = 45.0; // AUT_06 [m]

  // 导出约束
  double min_turn_radius() const {
    return hairpin_outer_r_min / 2.0;  // 约 4.5m
  }
  double max_curvature() const {
    return 1.0 / min_turn_radius();
  }
};

// ---- 聚合结构 ----
struct TrackConstraints {
  AccelerationConstraints acceleration;
  SkidpadConstraints      skidpad;
  AutocrossConstraints    autocross;
};

// ---- 工厂函数 ----
inline TrackConstraints getDefaultConstraints() {
  return TrackConstraints{};
}

// 按 mission 名称获取安全边界宽度
inline double getSafetyWidth(const std::string& mission) {
  if (mission == "acceleration") return 3.0;
  if (mission == "skidpad")      return 3.0;
  return 3.5;  // autocross / trackdrive
}

}  // namespace planning_core
