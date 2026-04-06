#pragma once

#include "planning_core/track_constraints.hpp"

#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <vector>

namespace planning_core {

/**
 * @brief Boundary Graph for Autocross (M2)
 *
 * Builds a graph representation of track boundaries using color consistency
 * and geometric priors. Designed for "first lap safe, subsequent laps fast"
 * strategy.
 *
 * Architecture:
 *   1. Boundary Graph Layer: Color consistency + geometric priors
 *   2. Corridor Layer: Drivable corridor with safety margin
 *   3. Centerline Optimizer: Smooth, low-curvature, boundary-clear trajectory
 *
 * Reference: PLANNING_M2_AUTOCROSS_DESIGN.md
 */

/**
 * @brief Node in the boundary graph representing a cone/waypoint
 */
struct BoundaryNode {
  uint32_t id = 0;
  double x = 0.0;
  double y = 0.0;
  int color = cone_type::NONE;  // Using cone_type constants
  double confidence = 1.0;

  // Graph connectivity
  std::vector<uint32_t> neighbors;

  // For boundary classification
  bool is_left_boundary = false;
  bool is_right_boundary = false;
  double lateral_position = 0.0;  // Distance from centerline estimate
};

/**
 * @brief Edge connecting boundary nodes
 */
struct BoundaryEdge {
  uint32_t from = 0;
  uint32_t to = 0;
  double weight = 0.0;
  double length = 0.0;

  // Edge type
  enum class Type { UNKNOWN, LEFT_BOUNDARY, RIGHT_BOUNDARY, CROSS_TRACK };
  Type type = Type::UNKNOWN;
};

/**
 * @brief Configuration for boundary graph construction
 */
struct BoundaryGraphConfig {
  // Color consistency rules
  bool use_color_rules = true;
  int left_color = cone_semantic::LEFT_BOUNDARY;    // RED
  int right_color = cone_semantic::RIGHT_BOUNDARY;  // BLUE

  // Geometric constraints
  double min_track_width = 3.0;  // Minimum track width [m]
  double max_track_width = 5.0;  // Maximum track width [m]
  double max_edge_length = 8.0;  // Maximum edge length [m]

  // Confidence thresholds
  double min_confidence = 0.3;   // Minimum cone confidence
  double color_weight = 1.0;     // Weight for color consistency
  double distance_weight = 2.0;  // Weight for geometric distance

  // Corridor parameters
  double safety_margin = 0.3;  // Safety margin from boundary [m]
};

/**
 * @brief Drivable corridor derived from boundary graph
 */
struct DrivableCorridor {
  // Left and right boundary sequences
  std::vector<BoundaryNode> left_boundary;
  std::vector<BoundaryNode> right_boundary;

  // Corridor centerline (initial estimate)
  std::vector<std::pair<double, double>> centerline;

  // Width at each centerline point
  std::vector<double> width;

  bool is_valid = false;
  std::string error_msg;
};

/**
 * @brief Boundary Graph builder for autocross planning
 *
 * M2 Architecture implementation:
 *   - Builds boundary graph from cone detections
 *   - Uses color consistency and geometric priors
 *   - Generates drivable corridor
 */
class BoundaryGraph {
 public:
  BoundaryGraph() = default;
  explicit BoundaryGraph(const BoundaryGraphConfig& config) : config_(config) {}

  /**
   * @brief Set configuration
   */
  void SetConfig(const BoundaryGraphConfig& config) { config_ = config; }

  /**
   * @brief Build boundary graph from cone positions
   * @param cone_positions Vector of (x, y, color, confidence) tuples
   * @return true if graph built successfully
   */
  bool BuildGraph(const std::vector<std::tuple<double, double, int, double>>& cone_positions);

  /**
   * @brief Extract drivable corridor from boundary graph
   * @return Corridor with left/right boundaries and centerline
   */
  DrivableCorridor ExtractCorridor() const;

  /**
   * @brief Get all nodes in the graph
   */
  const std::map<uint32_t, BoundaryNode>& GetNodes() const { return nodes_; }

  /**
   * @brief Get all edges in the graph
   */
  const std::vector<BoundaryEdge>& GetEdges() const { return edges_; }

  /**
   * @brief Clear the graph
   */
  void Clear() {
    nodes_.clear();
    edges_.clear();
    left_boundary_ids_.clear();
    right_boundary_ids_.clear();
  }

  /**
   * @brief Check if graph is valid
   */
  bool IsValid() const {
    return !nodes_.empty() && !left_boundary_ids_.empty() && !right_boundary_ids_.empty();
  }

 private:
  BoundaryGraphConfig config_;

  // Graph storage
  std::map<uint32_t, BoundaryNode> nodes_;
  std::vector<BoundaryEdge> edges_;

  // Boundary classification
  std::vector<uint32_t> left_boundary_ids_;
  std::vector<uint32_t> right_boundary_ids_;

  // Internal methods
  void ClassifyBoundaries();
  void BuildEdges();
  double ComputeEdgeWeight(const BoundaryNode& a, const BoundaryNode& b) const;
  bool IsColorConsistent(const BoundaryNode& a, const BoundaryNode& b) const;
};

/**
 * @brief Centerline optimizer for corridor-based trajectory generation
 *
 * Generates smooth centerline within drivable corridor.
 */
class CenterlineOptimizer {
 public:
  struct Config {
    double smoothing_weight = 0.5;    // Smoothness objective weight
    double boundary_clearance = 0.2;  // Minimum clearance from boundary [m]
    double max_curvature = 0.2;       // Maximum curvature [1/m]
  };

  CenterlineOptimizer() = default;
  explicit CenterlineOptimizer(const Config& config) : config_(config) {}

  /**
   * @brief Optimize centerline within drivable corridor
   * @param corridor Input drivable corridor
   * @return Optimized centerline as sequence of (x, y) points
   */
  std::vector<std::pair<double, double>> Optimize(const DrivableCorridor& corridor);

  void SetConfig(const Config& config) { config_ = config; }

 private:
  Config config_;
};

/**
 * @brief Autocross mode state machine (M2)
 *
 * MAP_BUILD_SAFE: First lap with larger safety margin
 * FAST_LAP: Subsequent laps with optimized trajectory
 */
class AutocrossModeStateMachine {
 public:
  enum class Mode {
    MAP_BUILD_SAFE,  // First lap: conservative
    FAST_LAP         // Subsequent laps: optimized
  };

  AutocrossModeStateMachine() = default;

  /**
   * @brief Update state machine based on lap progress
   * @param loop_closure_stable Number of consecutive frames with stable loop closure
   */
  void Update(bool loop_closure_stable);

  Mode GetMode() const { return mode_; }

  /**
   * @brief Get speed limit multiplier for current mode
   */
  double GetSpeedMultiplier() const { return (mode_ == Mode::MAP_BUILD_SAFE) ? 0.7 : 1.0; }

  /**
   * @brief Get safety margin for current mode
   */
  double GetSafetyMargin() const { return (mode_ == Mode::MAP_BUILD_SAFE) ? 0.5 : 0.3; }

  void Reset() {
    mode_ = Mode::MAP_BUILD_SAFE;
    stable_frame_count_ = 0;
  }

 private:
  Mode mode_ = Mode::MAP_BUILD_SAFE;
  int stable_frame_count_ = 0;

  // Hysteresis thresholds
  static constexpr int kEnterFastLapThreshold = 30;  // Frames to enter FAST_LAP
  static constexpr int kExitFastLapThreshold = 10;   // Frames to exit FAST_LAP
};

}  // namespace planning_core
