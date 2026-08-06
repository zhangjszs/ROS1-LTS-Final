#include "planning_core/boundary_graph.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace planning_core {

bool BoundaryGraph::BuildGraph(
    const std::vector<std::tuple<double, double, int, double>>& cone_positions) {
  Clear();

  if (cone_positions.empty()) {
    return false;
  }

  // Create nodes from cones
  uint32_t id = 0;
  for (const auto& cone : cone_positions) {
    double x = std::get<0>(cone);
    double y = std::get<1>(cone);
    int color = std::get<2>(cone);
    double confidence = std::get<3>(cone);

    if (confidence < config_.min_confidence) {
      continue;
    }

    BoundaryNode node;
    node.id = id++;
    node.x = x;
    node.y = y;
    node.color = color;
    node.confidence = confidence;

    nodes_[node.id] = node;
  }

  if (nodes_.size() < 2) {
    return false;
  }

  // Classify boundaries based on color rules
  ClassifyBoundaries();

  // Build edges between nodes
  BuildEdges();

  return IsValid();
}

void BoundaryGraph::ClassifyBoundaries() {
  left_boundary_ids_.clear();
  right_boundary_ids_.clear();

  // Simple classification based on color and lateral position
  // In a full implementation, this would use RANSAC or clustering
  double center_x = 0.0, center_y = 0.0;
  for (const auto& [id, node] : nodes_) {
    center_x += node.x;
    center_y += node.y;
  }
  center_x /= nodes_.size();
  center_y /= nodes_.size();

  for (auto& [id, node] : nodes_) {
    // Use color as primary classifier
    if (config_.use_color_rules) {
      if (node.color == config_.left_color) {
        node.is_left_boundary = true;
        left_boundary_ids_.push_back(id);
      } else if (node.color == config_.right_color) {
        node.is_right_boundary = true;
        right_boundary_ids_.push_back(id);
      } else {
        // Unknown color: use geometric position
        // Assume driving direction is roughly +X, left is +Y
        if (node.y > center_y) {
          node.is_left_boundary = true;
          left_boundary_ids_.push_back(id);
        } else {
          node.is_right_boundary = true;
          right_boundary_ids_.push_back(id);
        }
      }
    } else {
      // Geometric only
      if (node.y > center_y) {
        node.is_left_boundary = true;
        left_boundary_ids_.push_back(id);
      } else {
        node.is_right_boundary = true;
        right_boundary_ids_.push_back(id);
      }
    }

    // Compute lateral position relative to center
    node.lateral_position = node.y - center_y;
  }
}

namespace {
// Spatial hash grid for O(n) neighbor queries (replaces O(n²) brute force).
// Cell size = max_edge_length so neighbors are in same or adjacent cells.
class SpatialHash2D {
 public:
  explicit SpatialHash2D(double cell_size) : cell_size_(cell_size) {
    inv_cell_size_ = (cell_size > 0.0) ? 1.0 / cell_size : 1.0;
  }

  void Insert(uint32_t id, double x, double y) {
    auto [cx, cy] = Cell(x, y);
    grid_[Key(cx, cy)].push_back(id);
  }

  // Collect candidate neighbor IDs within one cell distance
  std::vector<uint32_t> Candidates(double x, double y) const {
    std::vector<uint32_t> result;
    auto [cx, cy] = Cell(x, y);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        auto it = grid_.find(Key(cx + dx, cy + dy));
        if (it != grid_.end()) {
          result.insert(result.end(), it->second.begin(), it->second.end());
        }
      }
    }
    return result;
  }

 private:
  static int64_t Key(int32_t cx, int32_t cy) {
    // Pack two 32-bit cell coords into one 64-bit key
    return (static_cast<int64_t>(cx) << 32) | static_cast<uint32_t>(cy);
  }

  std::pair<int32_t, int32_t> Cell(double x, double y) const {
    return {static_cast<int32_t>(std::floor(x * inv_cell_size_)),
            static_cast<int32_t>(std::floor(y * inv_cell_size_))};
  }

  double cell_size_;
  double inv_cell_size_;
  std::unordered_map<int64_t, std::vector<uint32_t>> grid_;
};
}  // namespace

void BoundaryGraph::BuildEdges() {
  edges_.clear();

  // Build edges between nearby nodes on same boundary using spatial hash
  auto connect_boundary = [this](const std::vector<uint32_t>& boundary_ids) {
    if (boundary_ids.size() < 2)
      return;

    SpatialHash2D grid(config_.max_edge_length);
    for (uint32_t id : boundary_ids) {
      const auto& node = nodes_[id];
      grid.Insert(id, node.x, node.y);
    }

    // Track which pairs are already connected to avoid duplicates
    std::set<std::pair<uint32_t, uint32_t>> seen;

    for (uint32_t id_i : boundary_ids) {
      const auto& node_i = nodes_[id_i];
      auto candidates = grid.Candidates(node_i.x, node_i.y);
      for (uint32_t id_j : candidates) {
        if (id_j <= id_i)
          continue;  // Avoid duplicate pairs and self-loops
        auto pair = std::make_pair(id_i, id_j);
        if (seen.insert(pair).second == false)
          continue;

        const auto& node_j = nodes_[id_j];
        double dx = node_i.x - node_j.x;
        double dy = node_i.y - node_j.y;
        double dist = std::hypot(dx, dy);

        if (dist < config_.max_edge_length) {
          BoundaryEdge edge;
          edge.from = id_i;
          edge.to = id_j;
          edge.length = dist;
          edge.weight = ComputeEdgeWeight(node_i, node_j);
          edge.type = node_i.is_left_boundary ? BoundaryEdge::Type::LEFT_BOUNDARY
                                              : BoundaryEdge::Type::RIGHT_BOUNDARY;

          edges_.push_back(edge);

          // Add to node neighbor lists
          nodes_[edge.from].neighbors.push_back(edge.to);
          nodes_[edge.to].neighbors.push_back(edge.from);
        }
      }
    }
  };

  connect_boundary(left_boundary_ids_);
  connect_boundary(right_boundary_ids_);

  // Build cross-track edges (between left and right boundaries)
  // Use spatial hash on right boundary for efficient neighbor lookup
  if (!left_boundary_ids_.empty() && !right_boundary_ids_.empty()) {
    const double cross_max = config_.max_track_width;
    SpatialHash2D grid(cross_max);
    for (uint32_t rid : right_boundary_ids_) {
      const auto& node = nodes_[rid];
      grid.Insert(rid, node.x, node.y);
    }

    for (uint32_t left_id : left_boundary_ids_) {
      const auto& left_node = nodes_[left_id];
      auto candidates = grid.Candidates(left_node.x, left_node.y);
      for (uint32_t right_id : candidates) {
        const auto& right_node = nodes_[right_id];
        double dx = left_node.x - right_node.x;
        double dy = left_node.y - right_node.y;
        double dist = std::hypot(dx, dy);

        // Check if within track width bounds
        if (dist >= config_.min_track_width && dist <= cross_max) {
          BoundaryEdge edge;
          edge.from = left_id;
          edge.to = right_id;
          edge.length = dist;
          edge.weight = dist;  // Prefer narrower cross-sections
          edge.type = BoundaryEdge::Type::CROSS_TRACK;

          edges_.push_back(edge);
        }
      }
    }
  }
}

double BoundaryGraph::ComputeEdgeWeight(const BoundaryNode& a, const BoundaryNode& b) const {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dist = std::hypot(dx, dy);

  double weight = config_.distance_weight * dist;

  // Color consistency bonus
  if (IsColorConsistent(a, b)) {
    weight -= config_.color_weight * 0.5;
  }

  return weight;
}

bool BoundaryGraph::IsColorConsistent(const BoundaryNode& a, const BoundaryNode& b) const {
  // Same color on same boundary is consistent
  if (a.color == b.color && a.color != cone_type::NONE) {
    return true;
  }
  // Different colors on same boundary might indicate misclassification
  return false;
}

DrivableCorridor BoundaryGraph::ExtractCorridor() const {
  DrivableCorridor corridor;

  if (!IsValid()) {
    corridor.error_msg = "Invalid boundary graph";
    return corridor;
  }

  // Sort boundaries by longitudinal position (x)
  auto sort_by_x = [this](std::vector<BoundaryNode>& boundary) {
    std::sort(boundary.begin(), boundary.end(),
              [](const BoundaryNode& a, const BoundaryNode& b) { return a.x < b.x; });
  };

  // Extract left boundary
  for (uint32_t id : left_boundary_ids_) {
    corridor.left_boundary.push_back(nodes_.at(id));
  }
  sort_by_x(corridor.left_boundary);

  // Extract right boundary
  for (uint32_t id : right_boundary_ids_) {
    corridor.right_boundary.push_back(nodes_.at(id));
  }
  sort_by_x(corridor.right_boundary);

  // Compute centerline as midpoint of boundaries
  size_t num_points = std::min(corridor.left_boundary.size(), corridor.right_boundary.size());
  for (size_t i = 0; i < num_points; ++i) {
    double cx = (corridor.left_boundary[i].x + corridor.right_boundary[i].x) / 2.0;
    double cy = (corridor.left_boundary[i].y + corridor.right_boundary[i].y) / 2.0;
    corridor.centerline.emplace_back(cx, cy);

    double width = std::hypot(corridor.left_boundary[i].x - corridor.right_boundary[i].x,
                              corridor.left_boundary[i].y - corridor.right_boundary[i].y);
    corridor.width.push_back(width);
  }

  corridor.is_valid = !corridor.centerline.empty();
  return corridor;
}

// Autocross Mode State Machine
void AutocrossModeStateMachine::Update(bool loop_closure_stable) {
  if (loop_closure_stable) {
    stable_frame_count_++;
  } else {
    stable_frame_count_--;
  }

  // Clamp to reasonable range. Must allow negative values so the exit
  // threshold (-kExitFastLapThreshold) is reachable from FAST_LAP.
  stable_frame_count_ =
      std::clamp(stable_frame_count_, -static_cast<int>(kExitFastLapThreshold),
                 static_cast<int>(kEnterFastLapThreshold));

  // State transitions with hysteresis
  switch (mode_) {
    case Mode::MAP_BUILD_SAFE:
      if (stable_frame_count_ >= kEnterFastLapThreshold) {
        mode_ = Mode::FAST_LAP;
      }
      break;
    case Mode::FAST_LAP:
      if (stable_frame_count_ <= -kExitFastLapThreshold) {
        mode_ = Mode::MAP_BUILD_SAFE;
      }
      break;
  }
}

// Centerline Optimizer
std::vector<std::pair<double, double>> CenterlineOptimizer::Optimize(
    const DrivableCorridor& corridor) {
  std::vector<std::pair<double, double>> optimized;

  if (!corridor.is_valid || corridor.centerline.empty()) {
    return optimized;
  }

  // Start with initial centerline
  optimized = corridor.centerline;

  // Apply smoothing (simple moving average for now)
  // In a full implementation, this would use quadratic programming
  const int window = 3;
  std::vector<std::pair<double, double>> smoothed;
  smoothed.reserve(optimized.size());

  for (size_t i = 0; i < optimized.size(); ++i) {
    double sum_x = 0.0, sum_y = 0.0;
    int count = 0;

    for (int j = -window; j <= window; ++j) {
      int idx = static_cast<int>(i) + j;
      if (idx >= 0 && idx < static_cast<int>(optimized.size())) {
        sum_x += optimized[idx].first;
        sum_y += optimized[idx].second;
        count++;
      }
    }

    smoothed.emplace_back(sum_x / count, sum_y / count);
  }

  return smoothed;
}

}  // namespace planning_core
