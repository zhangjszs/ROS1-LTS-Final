#include <localization_core/location_mapper.hpp>
#include <localization_core/confidence_utils.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>

namespace localization_core {

namespace {
constexpr double kPi = 3.14159265358979;

constexpr std::uint8_t kConeBlue = 0;
constexpr std::uint8_t kConeYellow = 1;
constexpr std::uint8_t kConeOrangeSmall = 2;
constexpr std::uint8_t kConeOrangeBig = 3;
constexpr std::uint8_t kConeNone = 4;
constexpr std::uint8_t kConeRed = 5;

std::uint8_t normalizeConeType(std::uint8_t raw_type)
{
  switch (raw_type)
  {
    case kConeBlue:
    case kConeYellow:
    case kConeOrangeSmall:
    case kConeOrangeBig:
    case kConeNone:
    case kConeRed:
      return raw_type;
    default:
      return kConeNone;
  }
}
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
  if (std::max(imu.nsv1, imu.nsv2) < params_.min_satellite_count)
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

  // HUAT_InsP2 加速度已经是 m/s²，不再乘以 g
  mins_.x_acc = imu_data.x_acc;
  mins_.y_acc = imu_data.y_acc;
  // z 轴去除重力分量：FRD 车体系下 gz = g * cos(roll) * cos(pitch)
  constexpr double kGravity = 9.79;
  mins_.z_acc = imu_data.z_acc + kGravity * std::cos(imu_data.roll) * std::cos(imu_data.pitch);

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

    // FSSIM-style body-frame state (heading=0 at init)
    car_state_.Vy = 0.0;
    car_state_.Wz = mins_.z_angular_velocity;
    car_state_.Ax = mins_.x_acc;
    car_state_.Ay = mins_.y_acc;

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

    // FSSIM-style body-frame state
    // Rotate NED velocity to body frame using heading
    double heading = car_state_.car_state.theta;
    double cos_h = std::cos(heading);
    double sin_h = std::sin(heading);
    // ENU velocity: east=Vx_enu, north=Vy_enu
    double vx_enu = mins_.east_velocity;
    double vy_enu = mins_.north_velocity;
    // Body frame: Vx_body = cos(h)*vx_enu + sin(h)*vy_enu
    //             Vy_body = -sin(h)*vx_enu + cos(h)*vy_enu
    car_state_.Vy = -sin_h * vx_enu + cos_h * vy_enu;
    car_state_.Wz = mins_.z_angular_velocity;
    car_state_.Ax = mins_.x_acc;
    car_state_.Ay = mins_.y_acc;

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

  const double yaw = (standard_azimuth_ - 90.0) * (kPi / 180.0);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double state_x = cos_yaw * enu_xyz[0] - sin_yaw * enu_xyz[1];
  const double state_y = sin_yaw * enu_xyz[0] + cos_yaw * enu_xyz[1];

  // 车体坐标系偏移需要用车辆航向旋转到地图坐标系
  const double heading = car_state_.car_state.theta;
  const double cos_h = std::cos(heading);
  const double sin_h = std::sin(heading);

  const double front_dx = cos_h * params_.front_to_imu_x - sin_h * params_.front_to_imu_y;
  const double front_dy = sin_h * params_.front_to_imu_x + cos_h * params_.front_to_imu_y;
  const double rear_dx = cos_h * params_.rear_to_imu_x - sin_h * params_.rear_to_imu_y;
  const double rear_dy = sin_h * params_.rear_to_imu_x + cos_h * params_.rear_to_imu_y;

  car_state_.car_state.x = state_x;
  car_state_.car_state.y = state_y;

  car_state_.car_state_front.x = state_x + front_dx;
  car_state_.car_state_front.y = state_y + front_dy;
  car_state_.car_state_front.z = enu_xyz[2] + params_.front_to_imu_z;

  car_state_.car_state_rear.x = state_x + rear_dx;
  car_state_.car_state_rear.y = state_y + rear_dy;
  car_state_.car_state_rear.z = enu_xyz[2] + params_.rear_to_imu_z;

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
  if (detections.detections.empty())
  {
    return false;
  }

  if (!map_out)
  {
    return false;
  }

  map_out->cones.clear();

  const double merge_distance = params_.merge_distance;
  const double merge_distance_sq = merge_distance * merge_distance;

  const double yaw = car_state_.car_state.theta;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double lidar_offset_x = cos_yaw * params_.lidar_to_imu_dist;
  const double lidar_offset_y = sin_yaw * params_.lidar_to_imu_dist;

  if (!first_cone_msg_)
  {
    first_cone_msg_ = true;
    for (size_t i = 0; i < detections.detections.size(); i++)
    {
      const auto &det = detections.detections[i];

      // 过滤层1：bbox 硬约束
      const double height = det.bbox_max.z - det.bbox_min.z;
      const double width_x = det.bbox_max.x - det.bbox_min.x;
      const double width_y = det.bbox_max.y - det.bbox_min.y;
      const double width = std::max(width_x, width_y);
      if (height > params_.max_cone_height || width > params_.max_cone_width)
        continue;
      if (height < params_.min_cone_height)
        continue;

      const double lx = det.point.x;
      const double ly = det.point.y;

      // 过滤层1.5：几何约束验证
      if (!passesGeometryFilter(lx, ly))
        continue;

      // 过滤层2：新锥桶 confidence 门控
      if (det.confidence < params_.min_confidence_to_add)
        continue;

      const double rot_x = cos_yaw * lx - sin_yaw * ly;
      const double rot_y = sin_yaw * lx + cos_yaw * ly;

      Cone cone;
      cone.position_base_link.x = lx + params_.lidar_to_imu_dist;
      cone.position_base_link.y = ly;
      cone.position_base_link.z = det.point.z;

      cone.position_global.x = rot_x + lidar_offset_x + car_state_.car_state.x;
      cone.position_global.y = rot_y + lidar_offset_y + car_state_.car_state.y;
      cone.position_global.z = det.point.z;

      cone.id = static_cast<std::uint32_t>(getNewId());

      pcl::PointXYZ point;
      point.x = cone.position_global.x;
      point.y = cone.position_global.y;
      point.z = cone.position_global.z;

      cloud_->push_back(point);
      cloud_->width = cloud_->points.size();
      cloud_->height = 1;
      point_ids_.push_back(static_cast<int>(cone.id));
      point_obs_counts_.push_back(1);
      point_types_.push_back(normalizeConeType(det.color_type));
      point_confidences_.push_back(det.confidence);
    }
    kdtree_.setInputCloud(cloud_);
  }
  else
  {
    bool cloud_modified = false;
    for (size_t i = 0; i < detections.detections.size(); i++)
    {
      const auto &det = detections.detections[i];

      // 过滤层1：bbox 硬约束
      const double height = det.bbox_max.z - det.bbox_min.z;
      const double width_x = det.bbox_max.x - det.bbox_min.x;
      const double width_y = det.bbox_max.y - det.bbox_min.y;
      const double width = std::max(width_x, width_y);
      if (height > params_.max_cone_height || width > params_.max_cone_width)
        continue;
      if (height < params_.min_cone_height)
        continue;

      const double lx = det.point.x;
      const double ly = det.point.y;

      // 过滤层1.5：几何约束验证
      if (!passesGeometryFilter(lx, ly))
        continue;

      const double rot_x = cos_yaw * lx - sin_yaw * ly;
      const double rot_y = sin_yaw * lx + cos_yaw * ly;

      Cone cone;
      cone.position_base_link.x = lx + params_.lidar_to_imu_dist;
      cone.position_base_link.y = ly;
      cone.position_base_link.z = det.point.z;

      cone.position_global.x = rot_x + lidar_offset_x + car_state_.car_state.x;
      cone.position_global.y = rot_y + lidar_offset_y + car_state_.car_state.y;
      cone.position_global.z = det.point.z;

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
          // 过滤层2：合并 confidence 门控
          if (det.confidence < params_.min_confidence_to_merge)
            continue;

          // 加权合并：使用递增均值 new = old * n/(n+1) + obs * 1/(n+1)
          const int idx = pointIdxNKNSearch[0];
          const int n = point_obs_counts_[idx];
          const double w_old = static_cast<double>(n) / static_cast<double>(n + 1);
          const double w_new = 1.0 / static_cast<double>(n + 1);
          cloud_->points[idx].x = static_cast<float>(cloud_->points[idx].x * w_old + point.x * w_new);
          cloud_->points[idx].y = static_cast<float>(cloud_->points[idx].y * w_old + point.y * w_new);
          cloud_->points[idx].z = static_cast<float>(cloud_->points[idx].z * w_old + point.z * w_new);
          point_obs_counts_[idx]++;
          if (idx < static_cast<int>(point_types_.size()))
          {
            const std::uint8_t obs_type = normalizeConeType(det.color_type);
            if (point_types_[idx] == kConeNone && obs_type != kConeNone)
            {
              point_types_[idx] = obs_type;
            }
          }
          // 置信度递增均值更新
          if (idx < static_cast<int>(point_confidences_.size()))
          {
            point_confidences_[idx] = point_confidences_[idx] * w_old + det.confidence * w_new;
          }
          cone.position_global.x = cloud_->points[idx].x;
          cone.position_global.y = cloud_->points[idx].y;
          cone.id = static_cast<std::uint32_t>(point_ids_[idx]);
          cloud_modified = true;
        }
        else
        {
          // 过滤层2：新锥桶 confidence 门控
          if (det.confidence < params_.min_confidence_to_add)
            continue;

          int id = getNewId();
          cone.id = static_cast<std::uint32_t>(id);
          cloud_->push_back(point);
          cloud_->width = cloud_->points.size();
          cloud_->height = 1;
          point_ids_.push_back(id);
          point_obs_counts_.push_back(1);
          point_types_.push_back(normalizeConeType(det.color_type));
          point_confidences_.push_back(det.confidence);
          cloud_modified = true;
        }
      }
      else
      {
        // 过滤层2：新锥桶 confidence 门控
        if (det.confidence < params_.min_confidence_to_add)
          continue;

        int id = getNewId();
        cone.id = static_cast<std::uint32_t>(id);
        cloud_->push_back(point);
        cloud_->width = cloud_->points.size();
        cloud_->height = 1;
        point_ids_.push_back(id);
        point_obs_counts_.push_back(1);
        point_types_.push_back(normalizeConeType(det.color_type));
        point_confidences_.push_back(det.confidence);
        cloud_modified = true;
      }
    }

    if (cloud_modified)
    {
      kdtree_.setInputCloud(cloud_);
    }
  }

  // 地图清理：超过最大容量时，移除低观测次数的锥桶
  if (params_.max_map_size > 0 &&
      static_cast<int>(cloud_->points.size()) > params_.max_map_size)
  {
    PointCloudPtr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> new_ids;
    std::vector<int> new_obs;
    std::vector<std::uint8_t> new_types;
    std::vector<double> new_confidences;
    new_cloud->reserve(cloud_->points.size());
    new_ids.reserve(cloud_->points.size());
    new_obs.reserve(cloud_->points.size());
    new_types.reserve(cloud_->points.size());
    new_confidences.reserve(cloud_->points.size());

    for (size_t i = 0; i < cloud_->points.size() && i < point_obs_counts_.size(); ++i)
    {
      if (point_obs_counts_[i] >= params_.min_obs_to_keep)
      {
        new_cloud->push_back(cloud_->points[i]);
        new_ids.push_back(point_ids_[i]);
        new_obs.push_back(point_obs_counts_[i]);
        if (i < point_types_.size())
        {
          new_types.push_back(point_types_[i]);
        }
        else
        {
          new_types.push_back(kConeNone);
        }
        new_confidences.push_back(
            (i < point_confidences_.size()) ? point_confidences_[i] : 0.0);
      }
    }
    new_cloud->width = new_cloud->points.size();
    new_cloud->height = 1;
    cloud_ = new_cloud;
    point_ids_ = std::move(new_ids);
    point_obs_counts_ = std::move(new_obs);
    point_types_ = std::move(new_types);
    point_confidences_ = std::move(new_confidences);
    kdtree_.setInputCloud(cloud_);
  }

  const double cos_theta = std::cos(car_state_.car_state.theta);
  const double sin_theta = std::sin(car_state_.car_state.theta);
  const double range_sq = params_.local_cone_range * params_.local_cone_range;

  std::vector<size_t> local_indices;
  local_indices.reserve(cloud_->points.size());
  for (size_t i = 0; i < cloud_->points.size() && i < point_ids_.size(); ++i)
  {
    const double dx = cloud_->points[i].x - car_state_.car_state.x;
    const double dy = cloud_->points[i].y - car_state_.car_state.y;

    if (dx * dx + dy * dy > range_sq)
    {
      continue;
    }
    local_indices.push_back(i);
  }

  // 发布前做一次近邻去重，抑制同一物理锥桶被重复建图导致的多锥桶显示
  const double dedup_radius = std::max(0.6, params_.merge_distance * 0.4);
  const double dedup_radius_sq = dedup_radius * dedup_radius;

  map_out->cones.reserve(local_indices.size());
  for (const size_t i : local_indices)
  {
    bool suppressed = false;
    const int obs_i = (i < point_obs_counts_.size()) ? point_obs_counts_[i] : 1;
    for (const size_t j : local_indices)
    {
      if (i == j)
      {
        continue;
      }
      const double ddx = cloud_->points[i].x - cloud_->points[j].x;
      const double ddy = cloud_->points[i].y - cloud_->points[j].y;
      if (ddx * ddx + ddy * ddy > dedup_radius_sq)
      {
        continue;
      }
      const int obs_j = (j < point_obs_counts_.size()) ? point_obs_counts_[j] : 1;
      // 保留观测次数更多的点；次数相同保留ID更小（更早建立）的点
      if (obs_j > obs_i || (obs_j == obs_i && point_ids_[j] < point_ids_[i]))
      {
        suppressed = true;
        break;
      }
    }
    if (suppressed)
    {
      continue;
    }

    const double dx = cloud_->points[i].x - car_state_.car_state.x;
    const double dy = cloud_->points[i].y - car_state_.car_state.y;
    Cone cone_msg;
    cone_msg.id = static_cast<std::uint32_t>(point_ids_[i]);
    cone_msg.position_global.x = cloud_->points[i].x;
    cone_msg.position_global.y = cloud_->points[i].y;
    cone_msg.position_global.z = cloud_->points[i].z;

    cone_msg.position_base_link.x = cos_theta * dx + sin_theta * dy;
    cone_msg.position_base_link.y = -sin_theta * dx + cos_theta * dy;
    cone_msg.position_base_link.z = cone_msg.position_global.z;
    cone_msg.type = (i < point_types_.size()) ? point_types_[i] : kConeNone;
    // 置信度传播：将 [0.0, 1.0] 映射到 [0, 1000]
    {
      double conf = (i < point_confidences_.size()) ? point_confidences_[i] : 0.0;
      // 观测次数越多，置信度越高（上限 +0.2 加成）
      int obs = (i < point_obs_counts_.size()) ? point_obs_counts_[i] : 1;
      double obs_bonus = std::min(0.2, 0.05 * (obs - 1));
      conf = std::min(1.0, std::max(0.0, conf + obs_bonus));
      cone_msg.confidence = confidence::EncodeScaled(conf);
    }
    map_out->cones.push_back(cone_msg);
  }

  // 缺锥插值：检测相邻锥间距过大的间隙，线性插值虚拟锥
  if (params_.missing_cone_fallback.enabled)
  {
    interpolateMissingCones(map_out);
  }

  // 短路径抑制：锥桶数量或空间范围不足时清空输出
  if (params_.short_path_suppression.enabled && !map_out->cones.empty())
  {
    const auto &sps = params_.short_path_suppression;
    const int cone_count = static_cast<int>(map_out->cones.size());

    if (sps.reject_single_cone_paths && cone_count <= 1)
    {
      map_out->cones.clear();
    }
    else if (cone_count < sps.min_cone_count)
    {
      map_out->cones.clear();
    }
    else
    {
      // 计算锥桶空间范围（base_link 坐标系下的 X 跨度）
      double x_min = map_out->cones[0].position_base_link.x;
      double x_max = x_min;
      for (const auto &c : map_out->cones)
      {
        if (c.position_base_link.x < x_min) x_min = c.position_base_link.x;
        if (c.position_base_link.x > x_max) x_max = c.position_base_link.x;
      }
      if ((x_max - x_min) < sps.min_path_length)
      {
        map_out->cones.clear();
      }
    }
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

bool LocationMapper::passesGeometryFilter(double lx, double ly) const
{
  if (params_.map_mode == "accel")
  {
    // 加速赛：锥桶只在 Y 轴 ±cone_y_max 范围内
    if (params_.cone_y_max > 0.0 && std::abs(ly) > params_.cone_y_max)
    {
      return false;
    }
  }
  else if (params_.map_mode == "skidpad")
  {
    if (params_.enable_circle_validation)
    {
      // 八字绕环：检测点到两个预期圆心的最近距离是否接近 circle_radius
      // 两个圆心在 X 轴方向，间距 circle_center_dist，关于原点对称
      const double half_dist = params_.circle_center_dist * 0.5;
      const double dx1 = lx;
      const double dy1 = ly - half_dist;
      const double dist1 = std::sqrt(dx1 * dx1 + dy1 * dy1);

      const double dx2 = lx;
      const double dy2 = ly + half_dist;
      const double dist2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

      const double err1 = std::abs(dist1 - params_.circle_radius);
      const double err2 = std::abs(dist2 - params_.circle_radius);
      const double min_err = std::min(err1, err2);

      if (min_err > params_.circle_tolerance)
      {
        return false;
      }
    }
  }
  // track 模式：不做额外约束
  return true;
}

void LocationMapper::interpolateMissingCones(ConeMap *map_out)
{
  if (!map_out || map_out->cones.size() < 2)
  {
    return;
  }

  const auto &cfg = params_.missing_cone_fallback;
  const double gap_threshold = cfg.expected_spacing * 1.5;

  // 按 base_link.x 排序（前方为正）
  std::vector<size_t> order(map_out->cones.size());
  for (size_t i = 0; i < order.size(); ++i) order[i] = i;
  std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
    return map_out->cones[a].position_base_link.x < map_out->cones[b].position_base_link.x;
  });

  std::vector<Cone> interpolated;
  for (size_t k = 0; k + 1 < order.size(); ++k)
  {
    const auto &c1 = map_out->cones[order[k]];
    const auto &c2 = map_out->cones[order[k + 1]];

    const double dx = c2.position_base_link.x - c1.position_base_link.x;
    const double dy = c2.position_base_link.y - c1.position_base_link.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= gap_threshold || dist > cfg.max_interpolation_distance)
    {
      continue;
    }

    // 计算需要插值的锥桶数量
    int n_missing = static_cast<int>(std::round(dist / cfg.expected_spacing)) - 1;
    n_missing = std::min(n_missing, cfg.max_consecutive_missing);
    if (n_missing <= 0)
    {
      continue;
    }

    for (int m = 1; m <= n_missing; ++m)
    {
      const double t = static_cast<double>(m) / static_cast<double>(n_missing + 1);
      Cone vc;
      vc.position_base_link.x = c1.position_base_link.x + t * dx;
      vc.position_base_link.y = c1.position_base_link.y + t * dy;
      vc.position_base_link.z = (c1.position_base_link.z + c2.position_base_link.z) * 0.5;

      // 全局坐标也做线性插值
      vc.position_global.x = c1.position_global.x + t * (c2.position_global.x - c1.position_global.x);
      vc.position_global.y = c1.position_global.y + t * (c2.position_global.y - c1.position_global.y);
      vc.position_global.z = (c1.position_global.z + c2.position_global.z) * 0.5;

      vc.id = static_cast<std::uint32_t>(getNewId());  // 虚拟锥使用会话内唯一ID（禁止ID=0）
      vc.confidence = confidence::EncodeScaled(cfg.min_confidence_for_interpolation);
      vc.type = kConeNone;
      interpolated.push_back(vc);
    }
  }

  for (auto &c : interpolated)
  {
    map_out->cones.push_back(std::move(c));
  }
}

}  // namespace localization_core
