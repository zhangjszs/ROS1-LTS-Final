#include "planning_core/boundary_graph.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

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

void BoundaryGraph::BuildEdges() {
  edges_.clear();

  // Build edges between nearby nodes on same boundary
  auto connect_boundary = [this](const std::vector<uint32_t>& boundary_ids) {
    for (size_t i = 0; i < boundary_ids.size(); ++i) {
      for (size_t j = i + 1; j < boundary_ids.size(); ++j) {
        const auto& node_i = nodes_[boundary_ids[i]];
        const auto& node_j = nodes_[boundary_ids[j]];

        double dx = node_i.x - node_j.x;
        double dy = node_i.y - node_j.y;
        double dist = std::hypot(dx, dy);

        if (dist < config_.max_edge_length) {
          BoundaryEdge edge;
          edge.from = boundary_ids[i];
          edge.to = boundary_ids[j];
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
  for (uint32_t left_id : left_boundary_ids_) {
    for (uint32_t right_id : right_boundary_ids_) {
      const auto& left_node = nodes_[left_id];
      const auto& right_node = nodes_[right_id];

      double dx = left_node.x - right_node.x;
      double dy = left_node.y - right_node.y;
      double dist = std::hypot(dx, dy);

      // Check if within track width bounds
      if (dist >= config_.min_track_width && dist <= config_.max_track_width) {
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

  // Clamp to reasonable range
  stable_frame_count_ = std::max(0, stable_frame_count_);

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
