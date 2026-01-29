#include <localization_core/location_mapper.hpp>

#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>

namespace localization_core {

namespace {
constexpr double kPi = 3.14159265358979;
}

LocationMapper::LocationMapper(const LocationParams &params)
    : params_(params)
{
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void LocationMapper::Configure(const LocationParams &params)
{
  params_ = params;
}

void LocationMapper::SetDataDirectory(const std::string &path)
{
  data_root_ = path;
}

bool LocationMapper::UpdateFromIns(const Asensing &imu, CarState *state_out)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (imu.status < params_.min_ins_status)
  {
    return false;
  }
  if (imu.nsv1 < params_.min_satellite_count)
  {
    return false;
  }
  if (imu.age > params_.max_diff_age)
  {
    return false;
  }

  mimu_ = imu;
  const Asensing &imu_data = mimu_;

  mins_.east_velocity = imu_data.east_velocity;
  mins_.north_velocity = imu_data.north_velocity;
  mins_.ground_velocity = imu_data.ground_velocity;
  mins_.azimuth = imu_data.azimuth;

  mins_.x_angular_velocity = imu_data.x_angular_velocity;
  mins_.y_angular_velocity = imu_data.y_angular_velocity;
  mins_.z_angular_velocity = imu_data.z_angular_velocity;

  mins_.x_acc = imu_data.x_acc * 9.79;
  mins_.y_acc = imu_data.y_acc * 9.79;
  mins_.z_acc = (imu_data.z_acc + std::cos(imu_data.roll) * std::cos(imu_data.pitch)) * 9.79;

  if (!has_carstate_)
  {
    car_state_.car_state.theta = 0.0;
    standard_azimuth_ = imu_data.azimuth;
    car_state_.V = std::sqrt(std::pow(mins_.east_velocity, 2) +
                             std::pow(mins_.north_velocity, 2) +
                             std::pow(mins_.ground_velocity, 2));
    car_state_.W = std::sqrt(std::pow(mins_.x_angular_velocity, 2) +
                             std::pow(mins_.y_angular_velocity, 2) +
                             std::pow(mins_.z_angular_velocity, 2));
    car_state_.A = std::sqrt(std::pow(mins_.x_acc, 2) +
                             std::pow(mins_.y_acc, 2) +
                             std::pow(mins_.z_acc, 2));

    first_lat_ = imu_data.latitude;
    first_lon_ = imu_data.longitude;
    first_alt_ = imu_data.altitude;

    car_state_.car_state.x = 0.0;
    car_state_.car_state.y = 0.0;
    has_carstate_ = true;
  }
  else
  {
    double diff = -(imu_data.azimuth - standard_azimuth_);
    car_state_.car_state.theta = diff * kPi / 180.0;
    if (car_state_.car_state.theta > kPi)
    {
      car_state_.car_state.theta -= 2 * kPi;
      diff -= 360.0;
    }
    else if (car_state_.car_state.theta < -kPi)
    {
      car_state_.car_state.theta += 2 * kPi;
      diff += 360.0;
    }

    car_state_.V = std::sqrt(std::pow(mins_.east_velocity, 2) +
                             std::pow(mins_.north_velocity, 2) +
                             std::pow(mins_.ground_velocity, 2));
    car_state_.W = std::sqrt(std::pow(mins_.x_angular_velocity, 2) +
                             std::pow(mins_.y_angular_velocity, 2) +
                             std::pow(mins_.z_angular_velocity, 2));
    car_state_.A = std::sqrt(std::pow(mins_.x_acc, 2) +
                             std::pow(mins_.y_acc, 2) +
                             std::pow(mins_.z_acc, 2));

    GeoDeticToENU(imu_data.latitude * kPi / 180.0,
                  imu_data.longitude * kPi / 180.0,
                  imu_data.altitude,
                  first_lat_ * kPi / 180.0,
                  first_lon_ * kPi / 180.0,
                  first_alt_,
                  &enu_xyz_[0]);
  }

  if (state_out)
  {
    *state_out = car_state_;
  }
  return true;
}

void LocationMapper::UpdateFromCarState(const CarState &state)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  car_state_ = state;
  has_carstate_ = true;
}

void LocationMapper::GeoDeticToENU(double lat, double lon, double h,
                                  double lat0, double lon0, double h0,
                                  double enu_xyz[3])
{
  const double a = 6378137.0;
  const double b = 6356752.3142;
  const double f = (a - b) / a;
  const double e_sq = f * (2 - f);

  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);

  const double N = a / std::sqrt(1 - e_sq * sin_lat * sin_lat);
  const double x = (h + N) * cos_lat * cos_lon;
  const double y = (h + N) * cos_lat * sin_lon;
  const double z = (h + (1 - e_sq) * N) * sin_lat;

  const double sin_lat0 = std::sin(lat0);
  const double cos_lat0 = std::cos(lat0);
  const double sin_lon0 = std::sin(lon0);
  const double cos_lon0 = std::cos(lon0);
  const double N0 = a / std::sqrt(1 - e_sq * sin_lat0 * sin_lat0);

  const double x0 = (h0 + N0) * cos_lat0 * cos_lon0;
  const double y0 = (h0 + N0) * cos_lat0 * sin_lon0;
  const double z0 = (h0 + (1 - e_sq) * N0) * sin_lat0;

  const double xd = x - x0;
  const double yd = y - y0;
  const double zd = z - z0;

  enu_xyz[0] = -sin_lon0 * xd + cos_lon0 * yd;
  enu_xyz[1] = (-cos_lon0 * xd - sin_lon0 * yd) * sin_lat0 + cos_lat0 * zd;
  enu_xyz[2] = cos_lat0 * cos_lon0 * xd + cos_lat0 * sin_lon0 * yd + sin_lat0 * zd;

  front_wheel_[0] = enu_xyz[0] + params_.front_to_imu_x;
  front_wheel_[1] = enu_xyz[1] + params_.front_to_imu_y;
  front_wheel_[2] = enu_xyz[2] + params_.front_to_imu_z;

  rear_wheel_[0] = enu_xyz[0] + params_.rear_to_imu_x;
  rear_wheel_[1] = enu_xyz[1] + params_.rear_to_imu_y;
  rear_wheel_[2] = enu_xyz[2] + params_.rear_to_imu_z;

  const double yaw = (standard_azimuth_ - 90.0) * (kPi / 180.0);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double state_x = cos_yaw * enu_xyz[0] - sin_yaw * enu_xyz[1];
  const double state_y = sin_yaw * enu_xyz[0] + cos_yaw * enu_xyz[1];

  const double front_x = cos_yaw * front_wheel_[0] - sin_yaw * front_wheel_[1];
  const double front_y = sin_yaw * front_wheel_[0] + cos_yaw * front_wheel_[1];
  const double rear_x = cos_yaw * rear_wheel_[0] - sin_yaw * rear_wheel_[1];
  const double rear_y = sin_yaw * rear_wheel_[0] + cos_yaw * rear_wheel_[1];

  car_state_.car_state.x = state_x;
  car_state_.car_state.y = state_y;

  car_state_.car_state_front.x = front_x;
  car_state_.car_state_front.y = front_y;
  car_state_.car_state_front.z = front_wheel_[2];

  car_state_.car_state_rear.x = rear_x;
  car_state_.car_state_rear.y = rear_y;
  car_state_.car_state_rear.z = rear_wheel_[2];

  has_carstate_ = true;
}

bool LocationMapper::UpdateFromCones(const ConeDetections &detections,
                                    ConeMap *map_out,
                                    PointCloudPtr *cloud_out)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!has_carstate_)
  {
    return false;
  }
  if (detections.points.empty())
  {
    return false;
  }

  if (!map_out)
  {
    return false;
  }

  map_out->cones.clear();

  const double merge_distance = 2.5;
  const double merge_distance_sq = merge_distance * merge_distance;

  const double yaw = car_state_.car_state.theta;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double lidar_offset_x = cos_yaw * params_.lidar_to_imu_dist;
  const double lidar_offset_y = sin_yaw * params_.lidar_to_imu_dist;

  if (!first_cone_msg_)
  {
    first_cone_msg_ = true;
    for (size_t i = 0; i < detections.points.size(); i++)
    {
      const double lx = detections.points[i].x;
      const double ly = detections.points[i].y;

      const double rot_x = cos_yaw * lx - sin_yaw * ly;
      const double rot_y = sin_yaw * lx + cos_yaw * ly;

      Cone cone;
      cone.position_base_link.x = lx + params_.lidar_to_imu_dist;
      cone.position_base_link.y = ly;
      cone.position_base_link.z = detections.points[i].z;

      cone.position_global.x = rot_x + car_state_.car_state.x;
      cone.position_global.y = rot_y + car_state_.car_state.y;
      cone.position_global.z = detections.points[i].z;

      cone.id = static_cast<std::uint32_t>(getNewId());

      pcl::PointXYZ point;
      point.x = cone.position_global.x;
      point.y = cone.position_global.y;
      point.z = cone.position_global.z;

      cloud_->push_back(point);
      cloud_->width = cloud_->points.size();
      cloud_->height = 1;
      point_ids_.push_back(static_cast<int>(cone.id));
    }
    kdtree_.setInputCloud(cloud_);
  }
  else
  {
    bool cloud_modified = false;
    for (size_t i = 0; i < detections.points.size(); i++)
    {
      const double lx = detections.points[i].x;
      const double ly = detections.points[i].y;

      const double rot_x = cos_yaw * lx - sin_yaw * ly;
      const double rot_y = sin_yaw * lx + cos_yaw * ly;

      Cone cone;
      cone.position_base_link.x = lx + params_.lidar_to_imu_dist;
      cone.position_base_link.y = ly;
      cone.position_base_link.z = detections.points[i].z;

      cone.position_global.x = rot_x + car_state_.car_state.x;
      cone.position_global.y = rot_y + car_state_.car_state.y;
      cone.position_global.z = detections.points[i].z;

      pcl::PointXYZ point;
      point.x = cone.position_global.x;
      point.y = cone.position_global.y;
      point.z = cone.position_global.z;

      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);

      if (kdtree_.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        if (pointNKNSquaredDistance[0] <= merge_distance_sq &&
            pointIdxNKNSearch[0] >= 0 && pointIdxNKNSearch[0] < static_cast<int>(cloud_->size()))
        {
          cloud_->points[pointIdxNKNSearch[0]].x = (cloud_->points[pointIdxNKNSearch[0]].x + point.x) / 2.0;
          cloud_->points[pointIdxNKNSearch[0]].y = (cloud_->points[pointIdxNKNSearch[0]].y + point.y) / 2.0;
          cloud_->points[pointIdxNKNSearch[0]].z = (cloud_->points[pointIdxNKNSearch[0]].z + point.z) / 2.0;
          cone.position_global.x = cloud_->points[pointIdxNKNSearch[0]].x;
          cone.position_global.y = cloud_->points[pointIdxNKNSearch[0]].y;
          cone.id = static_cast<std::uint32_t>(point_ids_[pointIdxNKNSearch[0]]);
        }
        else
        {
          int id = getNewId();
          cone.id = static_cast<std::uint32_t>(id);
          cloud_->push_back(point);
          cloud_->width = cloud_->points.size();
          cloud_->height = 1;
          point_ids_.push_back(id);
          cloud_modified = true;
        }
      }
      else
      {
        int id = getNewId();
        cone.id = static_cast<std::uint32_t>(id);
        cloud_->push_back(point);
        cloud_->width = cloud_->points.size();
        cloud_->height = 1;
        point_ids_.push_back(id);
        cloud_modified = true;
      }
    }

    if (cloud_modified)
    {
      kdtree_.setInputCloud(cloud_);
    }
  }

  const double cos_theta = std::cos(car_state_.car_state.theta);
  const double sin_theta = std::sin(car_state_.car_state.theta);

  map_out->cones.reserve(cloud_->points.size());
  for (size_t i = 0; i < cloud_->points.size() && i < point_ids_.size(); ++i)
  {
    Cone cone_msg;
    cone_msg.id = static_cast<std::uint32_t>(point_ids_[i]);
    cone_msg.position_global.x = cloud_->points[i].x;
    cone_msg.position_global.y = cloud_->points[i].y;
    cone_msg.position_global.z = cloud_->points[i].z;

    const double dx = cone_msg.position_global.x - car_state_.car_state.x;
    const double dy = cone_msg.position_global.y - car_state_.car_state.y;
    cone_msg.position_base_link.x = cos_theta * dx + sin_theta * dy;
    cone_msg.position_base_link.y = -sin_theta * dx + cos_theta * dy;
    cone_msg.position_base_link.z = cone_msg.position_global.z;
    map_out->cones.push_back(cone_msg);
  }

  if (cloud_out)
  {
    *cloud_out = cloud_;
  }

  return true;
}

int LocationMapper::getNewId()
{
  next_id_++;
  return next_id_;
}

void LocationMapper::saveCarstate(double x, double y)
{
  if (data_root_.empty())
  {
    return;
  }
  std::stringstream ss;
  ss << x << "\t" << y << std::endl;
  std::string path = data_root_ + "/testData/carstate.txt";
  std::ofstream f;
  f.open(path.c_str(), std::ios_base::app);
  if (f.fail())
  {
    return;
  }
  f << ss.str();
  f.close();
}

}  // namespace localization_core
