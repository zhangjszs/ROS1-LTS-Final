#include "planning_core/skidpad_detection_core.hpp"

namespace planning_core
{

SkidpadDetectionCore::SkidpadDetectionCore(const SkidpadParams &params)
  : params_(params)
{
  matchFlag_ = true;
  skidpad_msg_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  SetParams(params_);
}

void SkidpadDetectionCore::SetParams(const SkidpadParams &params)
{
  params_ = params;
  circle2lidar_ = params_.circle2lidar;
  targetX_ = params_.targetX;
  targetY_ = params_.targetY;
  FinTargetX_ = params_.FinTargetX;
  FInTargetY_ = params_.FinTargetY;
  distanceThreshold_ = params_.distanceThreshold;
  LeavedistanceThreshold_ = params_.leavedistanceThreshold;
  inverse_flag_ = params_.inverse_flag;
  stopdistance_ = params_.stopdistance;
}

void SkidpadDetectionCore::ProcessConeDetections(const std::vector<ConePoint> &cones)
{
  skidpad_msg_ptr_->clear();
  if (!matchFlag_ && !at2_angle_calced_)
  {
    return;
  }

  for (const auto &point : cones)
  {
    pcl::PointXYZ pcl_point;
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    skidpad_msg_ptr_->push_back(pcl_point);
    // std::cout << "Point: x = " << point.x << ", y = " << point.y << ", z = " << point.z << std::endl;
  }

  PassThrough(skidpad_msg_ptr_);
  orderCloud_.clear();
  find_four_bucket_ = false;
  at2_angle_calced_ = false;

  if (matchFlag_ && !find_four_bucket_)
  {
    int obtain_four = 0;
    std::vector<pcl::PointXYZ> order;
    for (int i = 0; i < static_cast<int>(skidpad_msg_ptr_->points.size()); i++)
    {
      pc_trans_.x = skidpad_msg_ptr_->points[i].x;
      pc_trans_.y = skidpad_msg_ptr_->points[i].y;
      pc_trans_.z = skidpad_msg_ptr_->points[i].z;
      orderCloud_.insert(pc_trans_);
    }

    std::set<pcl::PointXYZ, PointComparator>::iterator it = orderCloud_.begin();
    for (; it != orderCloud_.end(); it++)
    {
      if (obtain_four == 4)
      {
        // std::cout << "find four " << std::endl;
        find_four_bucket_ = true;
        break;
      }
      order.push_back(*it);
      obtain_four++;
    }

    if (!find_four_bucket_)
    {
      // std::cout << "miss four -bucket！" << std::endl;
      return;
    }

    // std::cout << "[0]:" << order[0].x << " " << order[0].y << std::endl;
    // std::cout << "[1]:" << order[1].x << " " << order[1].y << std::endl;
    // std::cout << "[2]:" << order[2].x << " " << order[2].y << std::endl;
    // std::cout << "[3]:" << order[3].x << " " << order[3].y << std::endl;

    mid_x_fir_ = (order[0].x + order[1].x) / 2;
    mid_y_fir_ = (order[0].y + order[1].y) / 2;

    mid_x_sec_ = (order[2].x + order[3].x) / 2;
    mid_y_sec_ = (order[2].y + order[3].y) / 2;

    at2_angle_mid_ = atan2(std::abs(mid_y_sec_ - mid_y_fir_), std::abs(mid_x_sec_ - mid_x_fir_));
    lipu = at2_angle_mid_;
    // std::cout << "angle_mid:" << at2_angle_mid_ << std::endl;
    if (find_four_bucket_)
      at2_angle_calced_ = true;
    return;
  }
  else
  {
    // std::cout << "wrong situation (matchFlag: " << matchFlag_ << ", find_four_bucket: " << find_four_bucket_ << ")" << std::endl;
    return;
  }
}

void SkidpadDetectionCore::UpdateVehicleState(const Trajectory &state)
{
  current_pose_ = state;
}

void SkidpadDetectionCore::PassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_ptr)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(in_ptr);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.1, 15);
  pass.filter(*in_ptr);

  pass.setFilterFieldName("y");
  pass.setFilterLimits(-3, 3);
  pass.filter(*in_ptr);
}

bool SkidpadDetectionCore::ChangPathFlag(double current_x, double current_y, double TargetX_, double TargetY_, double DistanceThreshold_)
{
  double dx = TargetX_ - current_x;
  double dy = TargetY_ - current_y;
  double distance = std::hypot(dx, dy);
  return distance < DistanceThreshold_;
}

void SkidpadDetectionCore::ChangLeavePathFlag(double current_x, double current_y, double TargetX_, double TargetY_, double LeaveDistanceThreshold_)
{
  double dx = TargetX_ - current_x;
  double dy = TargetY_ - current_y;
  double distance = std::hypot(dx, dy);
  if (distance >= LeaveDistanceThreshold_)
    haschanged_ = true;
}

void SkidpadDetectionCore::UpdateApproaching(double current_x, double current_y, double FinTargetX_, double FInTargetY_, double stopdistance_)
{
  double dx = FinTargetX_ - current_x;
  double dy = FInTargetY_ - current_y;
  double distance = std::hypot(dx, dy);
  if (distance <= stopdistance_)
  {
    approaching_goal_ = true;
  }
  else
  {
    approaching_goal_ = false;
  }
}

std::vector<Pose> SkidpadDetectionCore::TransformPath(const std::vector<Pose> &path) const
{
  std::vector<Pose> transformed_path = path;

  double angle = at2_angle_mid_ - lipu;
  Eigen::AngleAxisf t_V(angle, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f rotate_matrix = t_V.matrix();
  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();

  if (inverse_flag_)
    transform_matrix.block<3, 3>(0, 0) = rotate_matrix.inverse();
  else
    transform_matrix.block<3, 3>(0, 0) = rotate_matrix;

  for (size_t i = 0; i < transformed_path.size(); i++)
  {
    double temp_x = transformed_path[i].x;
    double temp_y = transformed_path[i].y;

    Eigen::Vector4f temp(temp_x, temp_y, 0, 1);
    Eigen::Vector4f result = transform_matrix * temp;

    transformed_path[i].x = result[0] / result[3];
    transformed_path[i].y = result[1] / result[3];
  }

  return transformed_path;
}

void SkidpadDetectionCore::RunAlgorithm()
{
  double interval = 0.05;
  double forward_distance = circle2lidar_;
  double circle_radius = 9.125;
  const double car_length = 1.87;
  double right_circle_x = forward_distance + car_length;
  double right_circle_y = -circle_radius;
  double left_circle_x = right_circle_x;
  double left_circle_y = circle_radius;

  changFlag_ = ChangPathFlag(current_pose_.x, current_pose_.y, targetX_, targetY_, distanceThreshold_);
  ChangLeavePathFlag(current_pose_.x, current_pose_.y, targetX_, targetY_, LeavedistanceThreshold_);
  UpdateApproaching(current_pose_.x, current_pose_.y, FinTargetX_, FInTargetY_, stopdistance_);
  // std::cout << "runAlgorithm:" << matchFlag_ << "  " << at2_angle_calced_ << std::endl;

  if (matchFlag_ && at2_angle_calced_)
  {
    std::vector<Pose> path;
    Pose pose;
    pose.qx = 0.0;
    pose.qy = 0.0;
    pose.qz = 0.0;
    pose.qw = 0.0;

    modeFlag_ = 1;
    for (double i = 0.5; i < (car_length + forward_distance); i += interval)
    {
      pose.x = i;
      pose.y = 0;
      path.push_back(pose);
    }

    path_output_ = TransformPath(path);
    path_updated_ = true;
    // std::cout << "发布第一段路径 " << modeFlag_ << std::endl;
    modeFlag_++;
    matchFlag_ = false;
    at2_angle_calced_ = false;
  }

  if (changFlag_)
  {
    if (haschanged_)
    {
      std::vector<Pose> path;
      Pose pose;
      pose.qx = 0.0;
      pose.qy = 0.0;
      pose.qz = 0.0;
      pose.qw = 0.0;

      switch (modeFlag_)
      {
      case 4:
      case 5:
        pose.x = car_length + forward_distance;
        pose.y = 0;
        for (double i = 0; i < 2 * M_PI; i += interval / circle_radius)
        {
          pose.x = circle_radius * std::cos(90 * M_PI / 180 - i) + right_circle_x;
          pose.y = circle_radius * std::sin(90 * M_PI / 180 - i) + right_circle_y;
          path.push_back(pose);
        }

        path_output_ = TransformPath(path);
        path_updated_ = true;
        // std::cout << "发布第二段路径 " << modeFlag_ << std::endl;
        modeFlag_++;
        haschanged_ = false;
        break;
      case 2:
      case 3:
        pose.x = car_length + forward_distance;
        pose.y = 0;
        for (double i = 0; i < 2 * M_PI; i += interval / circle_radius)
        {
          pose.x = circle_radius * std::cos(-90 * M_PI / 180 + i) + left_circle_x;
          pose.y = circle_radius * std::sin(-90 * M_PI / 180 + i) + left_circle_y;
          path.push_back(pose);
        }

        path_output_ = TransformPath(path);
        path_updated_ = true;
        // std::cout << "发布第三段路径 " << modeFlag_ << std::endl;
        modeFlag_++;
        haschanged_ = false;
        break;
      case 6:
        pose.x = car_length + forward_distance;
        pose.y = 0;
        for (float i = (car_length + forward_distance) + 2.0; i < (car_length + forward_distance) + 20.0; i += interval)
        {
          pose.x = i;
          pose.y = 0;
          path.push_back(pose);
        }

        path_output_ = TransformPath(path);
        path_updated_ = true;
        // std::cout << "发布第四段路径 " << modeFlag_ << std::endl;
        modeFlag_++;
        haschanged_ = false;
        break;
      default:
        // std::cout << modeFlag_ << std::endl;
        break;
      }
    }
  }
}

} // namespace planning_core
