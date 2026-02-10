#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace localization_core {

// ─── Anomaly / Relocalization enums & types ─────────────────────

enum class AnomalyState : uint8_t {
  TRACKING = 0,
  DEGRADED = 1,
  LOST     = 2,
  RELOC_A  = 3,   // descriptor-based relocalization
  RELOC_B  = 4    // particle-filter relocalization
};

enum class TopoRelation : uint8_t {
  SAME_SIDE     = 0,
  OPPOSITE_SIDE = 1,
  UNKNOWN       = 2
};

/// 6×6 color confusion matrix  (rows = observed, cols = true)
/// Indices: 0=BLUE, 1=YELLOW, 2=ORANGE_SMALL, 3=ORANGE_BIG, 4=NONE, 5=RED
constexpr std::size_t kColorTypeCount = 6;
using ColorConfusionMatrix = std::array<std::array<double, kColorTypeCount>, kColorTypeCount>;

inline ColorConfusionMatrix DefaultColorConfusion() {
  // Diagonal-dominant: P(obs==true) = 0.80, uniform off-diagonal.
  constexpr double kDiag = 0.80;
  constexpr double kOff = (1.0 - kDiag) / static_cast<double>(kColorTypeCount - 1);
  ColorConfusionMatrix m{};
  for (std::size_t i = 0; i < kColorTypeCount; ++i)
    for (std::size_t j = 0; j < kColorTypeCount; ++j)
      m[i][j] = (i == j) ? kDiag : kOff;
  return m;
}

/// Sub-map descriptor for relocalization database
struct SubMapDescriptor {
  uint64_t keyframe_id = 0;
  double px = 0.0;
  double py = 0.0;
  double ptheta = 0.0;
  std::vector<float> histogram;           // polar histogram descriptor
  std::vector<uint32_t> local_landmark_ids;
  std::vector<std::pair<double, double>> local_landmark_positions;  // global (x,y)
};

/// Result of a relocalization attempt
struct RelocResult {
  bool success = false;
  double dx = 0.0;
  double dy = 0.0;
  double dtheta = 0.0;
  int inlier_count = 0;
  double mean_residual = 1e9;
};

/// Particle for particle-filter relocalization
struct Particle {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double weight = 0.0;
  double log_weight = 0.0;
};

/// Anomaly state machine configuration
struct AnomalyConfig {
  double chi2_degrade = 15.0;       // chi² threshold to enter DEGRADED
  double chi2_recover = 5.0;        // chi² threshold to recover to TRACKING
  int    chi2_window  = 10;         // sliding window size for chi²
  double match_ratio_lost = 0.3;    // match ratio below which we start lost timer
  double match_lost_duration = 2.0; // seconds of low match ratio before LOST
  int    no_cone_frames_max = 30;   // consecutive no-cone frames before DEGRADED
  double reloc_a_timeout = 3.0;     // seconds before escalating RELOC_A → RELOC_B
  double reloc_b_timeout = 5.0;     // seconds before RELOC_B → LOST (retry)
};

/// Relocalization configuration
struct RelocConfig {
  // Descriptor parameters
  double submap_radius = 15.0;
  int    n_sectors = 12;
  int    n_rings = 5;
  int    n_channels = 3;            // color channels in histogram
  int    max_descriptors = 200;

  // Retrieval & RANSAC
  int    top_k = 5;
  int    ransac_max_iter = 100;
  int    min_inliers = 4;
  double ransac_inlier_thresh = 1.5;

  // Particle filter
  int    n_particles = 200;
  double particle_sigma_xy = 3.0;
  double particle_sigma_theta = 0.3;
  double converge_sigma_xy = 0.5;
  double converge_sigma_theta = 0.1;
  double timeout = 5.0;            // seconds before giving up
};

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
  uint8_t color_type = 4;  // 0=BLUE,1=YELLOW,2=ORANGE_SMALL,3=ORANGE_BIG,4=NONE,5=RED
  uint32_t obs_count = 0;
};

/// Cone observation in range-bearing form
struct ConeObservation {
  double range = 0.0;
  double bearing = 0.0;
  uint8_t color_type = 4;  // 0=BLUE,1=YELLOW,2=ORANGE_SMALL,3=ORANGE_BIG,4=NONE,5=RED
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

  // ─── Color-topology soft association ──────────────────────────
  double w_maha  = 1.0;           // Mahalanobis distance weight in multi-dim cost
  double w_color = 0.3;           // color confusion cost weight
  double w_topo  = 0.2;           // topology cost weight
  double topo_penalty = 2.0;      // penalty for OPPOSITE_SIDE relation
  double neighbor_radius = 8.0;   // radius for neighbor search in topo classification
  double gate_threshold = 15.0;   // Mahalanobis gate for data association

  // Color confusion matrix
  ColorConfusionMatrix color_confusion = DefaultColorConfusion();

  // ─── Map mode & geometry priors ───────────────────────────────
  std::string map_mode = "track";         // "accel" | "skidpad" | "track"

  // Skidpad circle parameters (used when map_mode == "skidpad")
  double circle_radius = 15.25;           // m
  double circle_center1_x = 0.0;
  double circle_center1_y = -18.25;
  double circle_center2_x = 0.0;
  double circle_center2_y = 18.25;
  double circle_sigma = 1.0;             // noise sigma for circle factor

  // ─── Anomaly & relocalization ─────────────────────────────────
  AnomalyConfig anomaly;
  RelocConfig reloc;
};

}  // namespace localization_core
