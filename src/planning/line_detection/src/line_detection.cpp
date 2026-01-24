#include "line_detection/line_detection.hpp"
#include <algorithm>
#include <limits>
#include <ros/package.h>

namespace fsac
{

LineDetection::LineDetection(ros::NodeHandle &nh) : nh_(nh)
{
  loadParameters();

  // Subscribe to cone positions and vehicle state
  cone_sub_ = nh_.subscribe(cone_topic_, 1, &LineDetection::coneCallback, this);
  car_state_sub_ = nh_.subscribe(car_state_topic_, 1, &LineDetection::carStateCallback, this);

  // Advertise planned path and finish signal
  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1);
  finish_pub_ = nh_.advertise<std_msgs::Bool>("line_finish_signal", 1);

  // Initialize algorithm state
  first_detection_done_ = false;
  finish_line_x_ = max_path_distance_;
  finished_ = false;

  ROS_INFO("[LineDetection] Node initialized");
}

void LineDetection::loadParameters()
{
  // Hough transform parameters
  nh_.param("hough/rho_resolution", hough_rho_resolution_, 0.1);
  nh_.param("hough/theta_resolution", hough_theta_resolution_, 0.01);
  nh_.param("hough/min_votes", hough_min_votes_, 3);
  nh_.param("hough/theta_tolerance", theta_tolerance_, 0.2);

  // Cone filtering parameters
  nh_.param("cones/max_distance", max_cone_distance_, 50.0);
  nh_.param("cones/min_distance", min_cone_distance_, 2.0);

  // Path generation parameters
  nh_.param("path/interval", path_interval_, 0.1);
  nh_.param("path/max_distance", max_path_distance_, 75.0);

  // Vehicle parameters
  nh_.param("vehicle/imu_offset_x", imu_offset_x_, 1.88);

  // Finish line detection
  nh_.param("finish/threshold", finish_line_threshold_, 2.0);

  // Topic names
  nh_.param<std::string>("topics/cone", cone_topic_, "/cone_position");
  nh_.param<std::string>("topics/car_state", car_state_topic_, "/Carstate");
  nh_.param<std::string>("topics/path", path_topic_, "/line_planned_path");

  ROS_INFO("[LineDetection] Parameters loaded:");
  ROS_INFO("  Hough: rho=%.2f, theta=%.2f, min_votes=%d",
           hough_rho_resolution_, hough_theta_resolution_, hough_min_votes_);
  ROS_INFO("  Path: interval=%.2f, max_distance=%.2f", path_interval_, max_path_distance_);
  ROS_INFO("  Topics: cone=%s, car_state=%s, path=%s",
           cone_topic_.c_str(), car_state_topic_.c_str(), path_topic_.c_str());
}

void LineDetection::coneCallback(const autodrive_msgs::HUAT_ConeDetections::ConstPtr &cone_msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  cone_positions_.clear();

  if (cone_msg->points.empty())
  {
    ROS_WARN("[LineDetection] Received empty cone message");
    return;
  }

  // Convert cone positions to local format
  for (const geometry_msgs::Point32 &point : cone_msg->points)
  {
    cone_positions_.emplace_back(point.x, point.y, point.z);
  }

  ROS_INFO("[LineDetection] Received %zu cones", cone_positions_.size());
}

void LineDetection::carStateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &car_state)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Update vehicle state
  vehicle_state_.x = car_state->car_state.x;
  vehicle_state_.y = car_state->car_state.y;
  vehicle_state_.theta = car_state->car_state.theta;
  vehicle_state_.v = car_state->V;
}

std::vector<ConePoint> LineDetection::filterCones(
    const std::vector<ConePoint> &cones)
{
  std::vector<ConePoint> filtered;

  for (const ConePoint &cone : cones)
  {
    double distance = std::sqrt(cone.x * cone.x + cone.y * cone.y);

    // Filter by distance
    if (distance >= min_cone_distance_ && distance <= max_cone_distance_)
    {
      // Also filter by lateral range (reasonable width for straight track)
      if (std::abs(cone.y) <= 10.0)  // 5 meters on each side
      {
        filtered.push_back(cone);
      }
    }
  }

  ROS_INFO("[LineDetection] Filtered %zu cones from %zu total",
           filtered.size(), cones.size());
  return filtered;
}

std::vector<HoughLine> LineDetection::houghTransform(const std::vector<ConePoint> &cones)
{
  // Calculate accumulator dimensions
  double max_rho = std::sqrt(max_cone_distance_ * max_cone_distance_ +
                             10.0 * 10.0);  // Add some margin
  int num_rho = static_cast<int>(2 * max_rho / hough_rho_resolution_);
  int num_theta = static_cast<int>(M_PI / hough_theta_resolution_);

  // Create accumulator array
  std::vector<std::vector<int>> accumulator(num_rho, std::vector<int>(num_theta, 0));

  // Vote for each cone
  for (const ConePoint &cone : cones)
  {
    for (int theta_idx = 0; theta_idx < num_theta; ++theta_idx)
    {
      double theta = theta_idx * hough_theta_resolution_;
      double rho = cone.x * std::cos(theta) + cone.y * std::sin(theta);

      // Find rho index (shift by half to handle negative values)
      int rho_idx = static_cast<int>((rho + max_rho) / hough_rho_resolution_);

      // Check bounds
      if (rho_idx >= 0 && rho_idx < num_rho)
      {
        accumulator[rho_idx][theta_idx]++;
      }
    }
  }

  // Extract lines from accumulator
  std::vector<HoughLine> lines;
  for (int rho_idx = 0; rho_idx < num_rho; ++rho_idx)
  {
    for (int theta_idx = 0; theta_idx < num_theta; ++theta_idx)
    {
      int votes = accumulator[rho_idx][theta_idx];
      if (votes >= hough_min_votes_)
      {
        HoughLine line;
        line.rho = (rho_idx * hough_rho_resolution_) - max_rho;
        line.theta = theta_idx * hough_theta_resolution_;
        line.votes = votes;
        lines.push_back(line);
      }
    }
  }

  // Sort by number of votes (descending)
  std::sort(lines.begin(), lines.end(),
            [](const HoughLine &a, const HoughLine &b)
            {
              return a.votes > b.votes;
            });

  ROS_INFO("[LineDetection] Hough transform found %zu candidate lines", lines.size());
  return lines;
}

std::pair<HoughLine, HoughLine> LineDetection::selectBoundaryLines(
    const std::vector<HoughLine> &lines)
{
  if (lines.size() < 2)
  {
    ROS_WARN("[LineDetection] Not enough lines detected for boundaries");
    return {HoughLine(), HoughLine()};
  }

  // For straight track, boundary lines should be approximately parallel
  // and on opposite sides (theta differs by ~0 or ~PI)

  HoughLine best_left, best_right;
  int best_votes = 0;

  for (size_t i = 0; i < std::min(lines.size(), static_cast<size_t>(10)); ++i)
  {
    for (size_t j = i + 1; j < std::min(lines.size(), static_cast<size_t>(10)); ++j)
    {
      const HoughLine &line1 = lines[i];
      const HoughLine &line2 = lines[j];

      // Check if lines are approximately parallel
      double theta_diff = std::abs(line1.theta - line2.theta);
      if (theta_diff > M_PI / 2.0)
      {
        theta_diff = M_PI - theta_diff;
      }

      if (theta_diff < theta_tolerance_)
      {
        // Check if lines are on opposite sides (different rho values)
        double rho_diff = std::abs(line1.rho - line2.rho);

        // For a straight track, we expect reasonable distance between boundaries
        if (rho_diff > 2.0 && rho_diff < 20.0)  // Between 2m and 20m
        {
          int total_votes = line1.votes + line2.votes;
          if (total_votes > best_votes)
          {
            best_votes = total_votes;

            // Assign left and right based on rho values
            // (this depends on coordinate system convention)
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

  if (best_votes == 0)
  {
    ROS_WARN("[LineDetection] Could not find suitable boundary lines");
  }
  else
  {
    ROS_INFO("[LineDetection] Selected boundary lines: left(votes=%d), right(votes=%d)",
             best_left.votes, best_right.votes);
  }

  return {best_left, best_right};
}

HoughLine LineDetection::calculateCenterLine(const HoughLine &left_line,
                                               const HoughLine &right_line)
{
  HoughLine center;

  // The center line is the average of the two boundary lines
  center.rho = (left_line.rho + right_line.rho) / 2.0;
  center.theta = (left_line.theta + right_line.theta) / 2.0;
  center.votes = left_line.votes + right_line.votes;

  ROS_INFO("[LineDetection] Center line: rho=%.2f, theta=%.2f", center.rho, center.theta);
  return center;
}

std::vector<geometry_msgs::PoseStamped> LineDetection::generatePath(const HoughLine &center_line)
{
  std::vector<geometry_msgs::PoseStamped> path;

  // Convert line from polar (rho, theta) to Cartesian form
  // Line equation: x*cos(theta) + y*sin(theta) = rho
  // For a straight track (approximately aligned with x-axis), we can simplify

  // Check if line is approximately horizontal (theta ~ 0 or ~PI)
  bool is_horizontal = (std::abs(center_line.theta) < theta_tolerance_ ||
                       std::abs(center_line.theta - M_PI) < theta_tolerance_);

  if (is_horizontal)
  {
    // Line is approximately: y = constant
    double y_offset = center_line.rho;

    // Generate path points along x-axis with constant y
    for (double x = 1.0; x <= max_path_distance_; x += path_interval_)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "velodyne";  // Vehicle frame
      pose.pose.position.x = x;
      pose.pose.position.y = y_offset;
      pose.pose.position.z = 0.0;

      // Set orientation to face forward
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;

      path.push_back(pose);
    }
  }
  else
  {
    // General case: compute y from line equation
    // y = (rho - x*cos(theta)) / sin(theta)

    for (double x = 1.0; x <= max_path_distance_; x += path_interval_)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "velodyne";  // Vehicle frame

      pose.pose.position.x = x;
      pose.pose.position.y = (center_line.rho - x * std::cos(center_line.theta)) /
                             std::sin(center_line.theta);
      pose.pose.position.z = 0.0;

      // Set orientation to face forward
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;

      path.push_back(pose);
    }
  }

  ROS_INFO("[LineDetection] Generated %zu path points", path.size());
  return path;
}

nav_msgs::Path LineDetection::convertToWorldCoordinates(
    const std::vector<geometry_msgs::PoseStamped> &path)
{
  nav_msgs::Path world_path;
  world_path.header.frame_id = "world";
  world_path.header.stamp = ros::Time::now();

  // Transform each point from vehicle to world coordinates
  // Using equation (5-31):
  // X = (x + 1.87) * sin(theta)
  // Y = (y + 1.87) * cos(theta)

  for (const auto &pose : path)
  {
    geometry_msgs::PoseStamped world_pose;
    world_pose.header = world_path.header;

    double x_shifted = pose.pose.position.x + imu_offset_x_;
    double y_shifted = pose.pose.position.y;

    world_pose.pose.position.x = x_shifted * std::sin(vehicle_state_.theta);
    world_pose.pose.position.y = y_shifted * std::cos(vehicle_state_.theta);
    world_pose.pose.position.z = pose.pose.position.z;

    world_path.poses.push_back(world_pose);
  }

  return world_path;
}

bool LineDetection::checkFinishLine(double current_x, double finish_x)
{
  return std::abs(current_x - finish_x) < finish_line_threshold_;
}

void LineDetection::publishPath()
{
  if (planned_path_.poses.empty())
  {
    ROS_WARN("[LineDetection] No path to publish");
    return;
  }

  path_pub_.publish(planned_path_);
  ROS_INFO("[LineDetection] Published path with %zu points", planned_path_.poses.size());
}

void LineDetection::runAlgorithm()
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Check if already finished
  if (finished_)
  {
    return;
  }

  // Check if finish line reached
  if (first_detection_done_)
  {
    if (checkFinishLine(vehicle_state_.x, finish_line_x_))
    {
      finished_ = true;

      // Publish finish signal
      std_msgs::Bool finish_msg;
      finish_msg.data = true;
      finish_pub_.publish(finish_msg);

      ROS_INFO("[LineDetection] Finish line reached!");
      return;
    }

    // Continue publishing the pre-planned path
    publishPath();
    return;
  }

  // Check if we have enough cones
  if (cone_positions_.size() < 4)
  {
    ROS_WARN("[LineDetection] Not enough cones for line detection");
    return;
  }

  // Filter cones
  std::vector<ConePoint> filtered_cones = filterCones(cone_positions_);

  if (filtered_cones.size() < 4)
  {
    ROS_WARN("[LineDetection] Not enough cones after filtering");
    return;
  }

  // Detect lines using Hough transform
  std::vector<HoughLine> lines = houghTransform(filtered_cones);

  if (lines.empty())
  {
    ROS_WARN("[LineDetection] No lines detected");
    return;
  }

  // Select boundary lines
  auto [left_line, right_line] = selectBoundaryLines(lines);

  if (left_line.votes == 0 || right_line.votes == 0)
  {
    ROS_WARN("[LineDetection] Could not determine boundary lines");
    return;
  }

  // Calculate center line
  center_line_ = calculateCenterLine(left_line, right_line);

  // Generate path along center line
  std::vector<geometry_msgs::PoseStamped> path_points = generatePath(center_line_);

  // Convert to world coordinates
  planned_path_ = convertToWorldCoordinates(path_points);

  // Set finish line position
  finish_line_x_ = max_path_distance_;

  // Mark first detection as done
  first_detection_done_ = true;

  // Publish the path
  publishPath();
}

} // namespace fsac
