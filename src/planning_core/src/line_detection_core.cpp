#include "planning_core/line_detection_core.hpp"

#include <algorithm>
#include <cmath>

namespace planning_core
{

LineDetectionCore::LineDetectionCore(const LineDetectionParams &params)
  : params_(params)
{
  finish_line_x_ = params_.max_path_distance;
}

void LineDetectionCore::SetParams(const LineDetectionParams &params)
{
  params_ = params;
  finish_line_x_ = params_.max_path_distance;
}

void LineDetectionCore::UpdateCones(const std::vector<ConePoint> &cones)
{
  cone_positions_ = cones;
}

void LineDetectionCore::UpdateVehicleState(const VehicleState &state)
{
  vehicle_state_ = state;
}

std::vector<ConePoint> LineDetectionCore::FilterCones(const std::vector<ConePoint> &cones) const
{
  std::vector<ConePoint> filtered;
  filtered.reserve(cones.size());

  for (const ConePoint &cone : cones)
  {
    double distance = std::sqrt(cone.x * cone.x + cone.y * cone.y);

    if (distance >= params_.min_cone_distance && distance <= params_.max_cone_distance)
    {
      if (std::abs(cone.y) <= params_.max_cone_lateral_distance)
      {
        filtered.push_back(cone);
      }
    }
  }

  return filtered;
}

void LineDetectionCore::InitializeAccumulator(int num_rho, int num_theta)
{
  if (num_rho != cached_num_rho_ || num_theta != cached_num_theta_)
  {
    hough_accumulator_.resize(num_rho);
    for (auto &row : hough_accumulator_)
    {
      row.resize(num_theta);
    }
    cached_num_rho_ = num_rho;
    cached_num_theta_ = num_theta;
  }

  for (auto &row : hough_accumulator_)
  {
    std::fill(row.begin(), row.end(), 0);
  }
}

std::vector<HoughLine> LineDetectionCore::HoughTransform(const std::vector<ConePoint> &cones) const
{
  double max_rho = std::sqrt(params_.max_cone_distance * params_.max_cone_distance + 10.0 * 10.0);
  int num_rho = static_cast<int>(2 * max_rho / params_.hough_rho_resolution);
  int num_theta = static_cast<int>(M_PI / params_.hough_theta_resolution);

  const_cast<LineDetectionCore*>(this)->InitializeAccumulator(num_rho, num_theta);

  for (const ConePoint &cone : cones)
  {
    for (int theta_idx = 0; theta_idx < num_theta; ++theta_idx)
    {
      double theta = theta_idx * params_.hough_theta_resolution;
      double rho = cone.x * std::cos(theta) + cone.y * std::sin(theta);

      int rho_idx = static_cast<int>((rho + max_rho) / params_.hough_rho_resolution);

      if (rho_idx >= 0 && rho_idx < num_rho)
      {
        hough_accumulator_[rho_idx][theta_idx]++;
      }
    }
  }

  std::vector<HoughLine> lines;
  for (int rho_idx = 0; rho_idx < num_rho; ++rho_idx)
  {
    for (int theta_idx = 0; theta_idx < num_theta; ++theta_idx)
    {
      int votes = hough_accumulator_[rho_idx][theta_idx];
      if (votes >= params_.hough_min_votes)
      {
        HoughLine line;
        line.rho = (rho_idx * params_.hough_rho_resolution) - max_rho;
        line.theta = theta_idx * params_.hough_theta_resolution;
        line.votes = votes;
        lines.push_back(line);
      }
    }
  }

  std::sort(lines.begin(), lines.end(),
            [](const HoughLine &a, const HoughLine &b)
            {
              return a.votes > b.votes;
            });

  return lines;
}

std::pair<HoughLine, HoughLine> LineDetectionCore::SelectBoundaryLines(
    const std::vector<HoughLine> &lines) const
{
  if (lines.size() < 2)
  {
    return {HoughLine(), HoughLine()};
  }

  HoughLine best_left;
  HoughLine best_right;
  int best_votes = 0;

  for (size_t i = 0; i < std::min(lines.size(), static_cast<size_t>(10)); ++i)
  {
    for (size_t j = i + 1; j < std::min(lines.size(), static_cast<size_t>(10)); ++j)
    {
      const HoughLine &line1 = lines[i];
      const HoughLine &line2 = lines[j];

      double theta_diff = std::abs(line1.theta - line2.theta);
      if (theta_diff > M_PI / 2.0)
      {
        theta_diff = M_PI - theta_diff;
      }

      if (theta_diff < params_.theta_tolerance)
      {
        double rho_diff = std::abs(line1.rho - line2.rho);

        if (rho_diff > params_.min_rho_diff && rho_diff < params_.max_rho_diff)
        {
          int total_votes = line1.votes + line2.votes;
          if (total_votes > best_votes)
          {
            best_votes = total_votes;

            if (line1.rho > line2.rho)
            {
              best_left = line2;
              best_right = line1;
            }
            else
            {
              best_left = line1;
              best_right = line2;
            }
          }
        }
      }
    }
  }

  return {best_left, best_right};
}

HoughLine LineDetectionCore::CalculateCenterLine(const HoughLine &left_line,
                                                 const HoughLine &right_line) const
{
  HoughLine center;
  center.rho = (left_line.rho + right_line.rho) / 2.0;
  center.theta = (left_line.theta + right_line.theta) / 2.0;
  center.votes = left_line.votes + right_line.votes;
  return center;
}

std::vector<Pose> LineDetectionCore::GeneratePath(const HoughLine &center_line) const
{
  std::vector<Pose> path;

  bool is_horizontal = (std::abs(center_line.theta) < params_.theta_tolerance ||
                        std::abs(center_line.theta - M_PI) < params_.theta_tolerance);

  if (is_horizontal)
  {
    double y_offset = center_line.rho;

    for (double x = 1.0; x <= params_.max_path_distance; x += params_.path_interval)
    {
      Pose pose;
      pose.x = x;
      pose.y = y_offset;
      pose.z = 0.0;
      path.push_back(pose);
    }
  }
  else
  {
    double sin_theta = std::sin(center_line.theta);
    if (std::abs(sin_theta) < 1e-6)
    {
      double y_offset = center_line.rho;
      for (double x = 1.0; x <= params_.max_path_distance; x += params_.path_interval)
      {
        Pose pose;
        pose.x = x;
        pose.y = y_offset;
        pose.z = 0.0;
        path.push_back(pose);
      }
    }
    else
    {
      for (double x = 1.0; x <= params_.max_path_distance; x += params_.path_interval)
      {
        Pose pose;
        pose.x = x;
        pose.y = (center_line.rho - x * std::cos(center_line.theta)) / sin_theta;
        pose.z = 0.0;
        path.push_back(pose);
      }
    }
  }

  return path;
}

std::vector<Pose> LineDetectionCore::ConvertToWorldCoordinates(const std::vector<Pose> &path) const
{
  std::vector<Pose> world_path;
  world_path.reserve(path.size());

  double cos_theta = std::cos(vehicle_state_.theta);
  double sin_theta = std::sin(vehicle_state_.theta);

  for (const auto &pose : path)
  {
    Pose world_pose;

    double x_shifted = pose.x + params_.imu_offset_x;
    double y_shifted = pose.y;

    world_pose.x = x_shifted * cos_theta - y_shifted * sin_theta + vehicle_state_.x;
    world_pose.y = x_shifted * sin_theta + y_shifted * cos_theta + vehicle_state_.y;
    world_pose.z = pose.z;
    world_pose.qx = pose.qx;
    world_pose.qy = pose.qy;
    world_pose.qz = pose.qz;
    world_pose.qw = pose.qw;

    world_path.push_back(world_pose);
  }

  return world_path;
}

bool LineDetectionCore::CheckFinishLine(double current_x, double finish_x) const
{
  return std::abs(current_x - finish_x) < params_.finish_line_threshold;
}

void LineDetectionCore::RunAlgorithm()
{
  if (finished_)
  {
    return;
  }

  if (first_detection_done_)
  {
    if (CheckFinishLine(vehicle_state_.x, finish_line_x_))
    {
      finished_ = true;
    }

    return;
  }

  if (cone_positions_.size() < 4)
  {
    return;
  }

  std::vector<ConePoint> filtered_cones = FilterCones(cone_positions_);

  if (filtered_cones.size() < 4)
  {
    return;
  }

  std::vector<HoughLine> lines = HoughTransform(filtered_cones);

  if (lines.empty())
  {
    return;
  }

  auto [left_line, right_line] = SelectBoundaryLines(lines);

  if (left_line.votes == 0 || right_line.votes == 0)
  {
    return;
  }

  center_line_ = CalculateCenterLine(left_line, right_line);

  std::vector<Pose> path_points = GeneratePath(center_line_);

  planned_path_ = ConvertToWorldCoordinates(path_points);

  finish_line_x_ = params_.max_path_distance;
  first_detection_done_ = true;
}

} // namespace planning_core
