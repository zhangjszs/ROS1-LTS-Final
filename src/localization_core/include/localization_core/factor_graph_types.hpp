#pragma once

#include <cstdint>
#include <vector>

namespace localization_core {

/// Key symbol prefixes for GTSAM
namespace fg_symbols {
  constexpr unsigned char kPose = 'x';
  constexpr unsigned char kVelocity = 'v';
  constexpr unsigned char kBias = 'b';
  constexpr unsigned char kLandmark = 'l';
}

/// Keyframe trigger thresholds
struct KeyframeCriteria {
  double dist_threshold = 1.0;    // m
  double yaw_threshold = 0.1;     // rad
  double dt_threshold = 0.5;      // s
};

/// GNSS quality tier for dynamic covariance
enum class GnssQuality : uint8_t {
  GOOD,     // nsv >= 12, age < 5  -> sigma = 0.5m
  MEDIUM,   // nsv >= 8,  age < 15 -> sigma = 2.0m
  POOR,     // otherwise           -> sigma = 10.0m
  INVALID   // skip GNSS factor
};

/// Landmark (cone) in the factor graph
struct FgLandmark {
  uint32_t id = 0;
  double x = 0.0;
  double y = 0.0;
  uint8_t color_type = 4;  // 0=BLUE,1=YELLOW,2=ORANGE_SMALL,3=ORANGE_BIG,4=NONE
  uint32_t obs_count = 0;
};

/// Cone observation in range-bearing form
struct ConeObservation {
  double range = 0.0;
  double bearing = 0.0;
  uint8_t color_type = 4;
  double confidence = 0.0;
  int landmark_id = -1;  // -1 = new landmark
};

/// Factor graph optimizer configuration
struct FactorGraphConfig {
  // iSAM2
  double relinearize_threshold = 0.1;
  int relinearize_skip = 1;
  bool use_dogleg = true;
  bool run_extra_update = false;  // optional 2nd isam update

  // Keyframe
  KeyframeCriteria keyframe;

  // IMU noise
  double sigma_imu_xy = 0.05;     // m per step
  double sigma_imu_theta = 0.01;  // rad per step
  double sigma_vel = 0.3;         // m/s

  // GNSS noise (base, scaled by quality)
  double sigma_gnss_good = 0.5;
  double sigma_gnss_medium = 2.0;
  double sigma_gnss_poor = 10.0;

  // Vehicle speed/yaw rate
  double sigma_speed = 0.2;       // m/s
  double sigma_yaw_rate = 0.05;   // rad/s

  // Cone observation
  double sigma_range_base = 0.1;  // m
  double sigma_range_scale = 0.02;// m per m range
  double sigma_bearing_base = 0.02; // rad
  double sigma_bearing_scale = 0.01;// rad per m range
  double min_cone_confidence = 0.0; // confidence gate
  int max_cone_factors_per_keyframe = 10; // <=0 means unlimited

  // Color consistency
  double color_mismatch_penalty = 2.0;
  double color_weight = 1.0;

  // Robust kernels
  double huber_cone = 2.0;
  double huber_speed = 1.345;
  double cauchy_gnss = 5.0;

  // Landmark management
  double new_landmark_range_max = 30.0;  // m
  double merge_distance = 2.0;           // m
  int max_landmarks = 500;
  double landmark_init_sigma = 5.0;      // m
};

}  // namespace localization_core
