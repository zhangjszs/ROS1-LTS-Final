#include <localization_core/location_mapper.hpp>
#include <localization_core/confidence_utils.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
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
                                     PointCloudPtr *cloud_out,
                                     MapUpdateStats *stats_out)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!has_carstate_ || detections.detections.empty() || !map_out)
  {
    return false;
  }

  ++cone_update_frames_;
  ++map_update_seq_;

  MapUpdateStats local_stats;
  local_stats.input_count = static_cast<int>(detections.detections.size());
  local_stats.update_seq = map_update_seq_;
  local_stats.map_frozen = map_frozen_;
  local_stats.map_size = static_cast<int>(cloud_->size());

  auto push_outlier = [&local_stats](const Cone &cone) {
    if (local_stats.outlier_cones.size() < 200)
    {
      local_stats.outlier_cones.push_back(cone);
    }
  };
  auto push_inlier = [&local_stats](const Cone &cone) {
    if (local_stats.inlier_cones.size() < 200)
    {
      local_stats.inlier_cones.push_back(cone);
    }
  };
  auto push_assoc = [&local_stats](const ConeAssociationDebug &assoc) {
    if (local_stats.associations.size() < 200)
    {
      local_stats.associations.push_back(assoc);
    }
  };

  const double merge_distance = params_.merge_distance;
  const double merge_distance_sq = merge_distance * merge_distance;
  const double yaw = car_state_.car_state.theta;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double lidar_offset_x = cos_yaw * params_.lidar_to_imu_dist;
  const double lidar_offset_y = sin_yaw * params_.lidar_to_imu_dist;

  auto make_cone_from_detection = [&](const ConeDetection &det) {
    Cone cone;
    const double lx = det.point.x;
    const double ly = det.point.y;
    const double rot_x = cos_yaw * lx - sin_yaw * ly;
    const double rot_y = sin_yaw * lx + cos_yaw * ly;

    cone.position_base_link.x = lx + params_.lidar_to_imu_dist;
    cone.position_base_link.y = ly;
    cone.position_base_link.z = det.point.z;
    cone.position_global.x = rot_x + lidar_offset_x + car_state_.car_state.x;
    cone.position_global.y = rot_y + lidar_offset_y + car_state_.car_state.y;
    cone.position_global.z = det.point.z;
    cone.type = normalizeConeType(det.color_type);
    cone.confidence = confidence::EncodeScaled(std::max(0.0, std::min(1.0, det.confidence)));
    return cone;
  };

  bool cloud_modified = false;
  first_cone_msg_ = !cloud_->empty();
  for (const auto &det : detections.detections)
  {
    const double height = det.bbox_max.z - det.bbox_min.z;
    const double width_x = det.bbox_max.x - det.bbox_min.x;
    const double width_y = det.bbox_max.y - det.bbox_min.y;
    const double width = std::max(width_x, width_y);
    Cone det_cone = make_cone_from_detection(det);

    if (height > params_.max_cone_height || width > params_.max_cone_width || height < params_.min_cone_height)
    {
      local_stats.bbox_reject_count++;
      push_outlier(det_cone);
      continue;
    }

    if (!passesGeometryFilter(det.point.x, det.point.y))
    {
      local_stats.geometry_reject_count++;
      push_outlier(det_cone);
      continue;
    }

    pcl::PointXYZ point;
    point.x = det_cone.position_global.x;
    point.y = det_cone.position_global.y;
    point.z = det_cone.position_global.z;

    int nearest_idx = -1;
    float nearest_sq_dist = 0.0f;
    if (!cloud_->empty())
    {
      std::vector<int> idx(1);
      std::vector<float> sqdist(1);
      if (kdtree_.nearestKSearch(point, 1, idx, sqdist) > 0 &&
          idx[0] >= 0 && idx[0] < static_cast<int>(cloud_->size()))
      {
        nearest_idx = idx[0];
        nearest_sq_dist = sqdist[0];
      }
    }

    const bool can_merge = (nearest_idx >= 0 && nearest_sq_dist <= merge_distance_sq);
    if (can_merge)
    {
      if (det.confidence < params_.min_confidence_to_merge)
      {
        local_stats.conf_merge_reject_count++;
        push_outlier(det_cone);
        continue;
      }
      if (map_frozen_ && !params_.map_freeze.allow_merge_when_frozen)
      {
        local_stats.frozen_reject_count++;
        push_outlier(det_cone);
        continue;
      }

      const int n = point_obs_counts_[nearest_idx];
      const double w_old = static_cast<double>(n) / static_cast<double>(n + 1);
      const double w_new = 1.0 / static_cast<double>(n + 1);
      cloud_->points[nearest_idx].x = static_cast<float>(cloud_->points[nearest_idx].x * w_old + point.x * w_new);
      cloud_->points[nearest_idx].y = static_cast<float>(cloud_->points[nearest_idx].y * w_old + point.y * w_new);
      cloud_->points[nearest_idx].z = static_cast<float>(cloud_->points[nearest_idx].z * w_old + point.z * w_new);
      point_obs_counts_[nearest_idx]++;
      if (nearest_idx < static_cast<int>(point_types_.size()))
      {
        const std::uint8_t obs_type = normalizeConeType(det.color_type);
        if (point_types_[nearest_idx] == kConeNone && obs_type != kConeNone)
        {
          point_types_[nearest_idx] = obs_type;
        }
      }
      if (nearest_idx < static_cast<int>(point_confidences_.size()))
      {
        point_confidences_[nearest_idx] = point_confidences_[nearest_idx] * w_old + det.confidence * w_new;
      }

      Cone merged_cone = det_cone;
      merged_cone.id = static_cast<std::uint32_t>(point_ids_[nearest_idx]);
      merged_cone.position_global.x = cloud_->points[nearest_idx].x;
      merged_cone.position_global.y = cloud_->points[nearest_idx].y;
      merged_cone.position_global.z = cloud_->points[nearest_idx].z;
      push_inlier(merged_cone);

      ConeAssociationDebug assoc;
      assoc.detection_base = det_cone.position_base_link;
      assoc.detection_global = det_cone.position_global;
      assoc.matched_global = merged_cone.position_global;
      assoc.matched_id = merged_cone.id;
      assoc.distance = std::sqrt(std::max(0.0f, nearest_sq_dist));
      assoc.merged = true;
      push_assoc(assoc);

      local_stats.merged_count++;
      local_stats.mean_match_distance += assoc.distance;
      local_stats.max_match_distance = std::max(local_stats.max_match_distance, assoc.distance);
      cloud_modified = true;
      continue;
    }

    if (det.confidence < params_.min_confidence_to_add)
    {
      local_stats.conf_add_reject_count++;
      push_outlier(det_cone);
      continue;
    }
    if (map_frozen_ && !params_.map_freeze.allow_new_cones_when_frozen)
    {
      local_stats.frozen_reject_count++;
      push_outlier(det_cone);
      continue;
    }

    int id = getNewId();
    det_cone.id = static_cast<std::uint32_t>(id);
    cloud_->push_back(point);
    point_ids_.push_back(id);
    point_obs_counts_.push_back(1);
    point_types_.push_back(normalizeConeType(det.color_type));
    point_confidences_.push_back(det.confidence);
    push_inlier(det_cone);

    ConeAssociationDebug assoc;
    assoc.detection_base = det_cone.position_base_link;
    assoc.detection_global = det_cone.position_global;
    assoc.matched_global = det_cone.position_global;
    assoc.matched_id = det_cone.id;
    assoc.distance = 0.0;
    assoc.merged = false;
    push_assoc(assoc);
    local_stats.inserted_count++;
    cloud_modified = true;
  }

  if (cloud_modified)
  {
    cloud_->width = cloud_->points.size();
    cloud_->height = 1;
    kdtree_.setInputCloud(cloud_);
    first_cone_msg_ = true;
  }

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
        new_types.push_back((i < point_types_.size()) ? point_types_[i] : kConeNone);
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

  if (!map_frozen_ && params_.map_freeze.enabled)
  {
    const bool frame_hit =
        params_.map_freeze.freeze_after_frames > 0 &&
        cone_update_frames_ >= params_.map_freeze.freeze_after_frames;
    const bool cone_hit =
        params_.map_freeze.freeze_after_cones > 0 &&
        static_cast<int>(cloud_->size()) >= params_.map_freeze.freeze_after_cones;
    if (frame_hit || cone_hit)
    {
      map_frozen_ = true;
      checkpoint_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
      checkpoint_ids_ = point_ids_;
      checkpoint_obs_counts_ = point_obs_counts_;
      checkpoint_types_ = point_types_;
      checkpoint_confidences_ = point_confidences_;
      has_checkpoint_ = true;
    }
  }

  const double cos_theta = std::cos(car_state_.car_state.theta);
  const double sin_theta = std::sin(car_state_.car_state.theta);
  const double range_sq = params_.local_cone_range * params_.local_cone_range;

  map_out->cones.clear();
  std::vector<size_t> local_indices;
  local_indices.reserve(cloud_->points.size());
  for (size_t i = 0; i < cloud_->points.size() && i < point_ids_.size(); ++i)
  {
    const double dx = cloud_->points[i].x - car_state_.car_state.x;
    const double dy = cloud_->points[i].y - car_state_.car_state.y;
    if (dx * dx + dy * dy <= range_sq)
    {
      local_indices.push_back(i);
    }
  }

  const double dedup_radius = std::max(0.6, params_.merge_distance * 0.4);
  const double dedup_radius_sq = dedup_radius * dedup_radius;
  map_out->cones.reserve(local_indices.size());
  for (const size_t i : local_indices)
  {
    bool suppressed = false;
    const int obs_i = (i < point_obs_counts_.size()) ? point_obs_counts_[i] : 1;
    for (const size_t j : local_indices)
    {
      if (i == j) continue;
      const double ddx = cloud_->points[i].x - cloud_->points[j].x;
      const double ddy = cloud_->points[i].y - cloud_->points[j].y;
      if (ddx * ddx + ddy * ddy > dedup_radius_sq) continue;
      const int obs_j = (j < point_obs_counts_.size()) ? point_obs_counts_[j] : 1;
      if (obs_j > obs_i || (obs_j == obs_i && point_ids_[j] < point_ids_[i]))
      {
        suppressed = true;
        break;
      }
    }
    if (suppressed) continue;

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
    {
      double conf = (i < point_confidences_.size()) ? point_confidences_[i] : 0.0;
      int obs = (i < point_obs_counts_.size()) ? point_obs_counts_[i] : 1;
      double obs_bonus = std::min(0.2, 0.05 * (obs - 1));
      conf = std::min(1.0, std::max(0.0, conf + obs_bonus));
      cone_msg.confidence = confidence::EncodeScaled(conf);
    }
    map_out->cones.push_back(cone_msg);
  }

  if (params_.missing_cone_fallback.enabled)
  {
    interpolateMissingCones(map_out);
  }

  if (params_.short_path_suppression.enabled && !map_out->cones.empty())
  {
    const auto &sps = params_.short_path_suppression;
    const int cone_count = static_cast<int>(map_out->cones.size());
    if ((sps.reject_single_cone_paths && cone_count <= 1) || cone_count < sps.min_cone_count)
    {
      map_out->cones.clear();
    }
    else
    {
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

  if (local_stats.merged_count > 0)
  {
    local_stats.mean_match_distance /= static_cast<double>(local_stats.merged_count);
  }
  local_stats.map_frozen = map_frozen_;
  local_stats.map_size = static_cast<int>(cloud_->size());
  local_stats.local_output_count = static_cast<int>(map_out->cones.size());
  if (stats_out)
  {
    *stats_out = std::move(local_stats);
  }
  return true;
}

int LocationMapper::getNewId()
{
  next_id_++;
  return next_id_;
}

void LocationMapper::ResetMap(bool keep_checkpoint)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_->width = 0;
  cloud_->height = 1;
  point_ids_.clear();
  point_obs_counts_.clear();
  point_types_.clear();
  point_confidences_.clear();
  first_cone_msg_ = false;
  map_frozen_ = false;
  cone_update_frames_ = 0;
  map_update_seq_ = 0;
  next_id_ = 0;
  kdtree_.setInputCloud(cloud_);
  if (!keep_checkpoint)
  {
    has_checkpoint_ = false;
    checkpoint_cloud_.reset();
    checkpoint_ids_.clear();
    checkpoint_obs_counts_.clear();
    checkpoint_types_.clear();
    checkpoint_confidences_.clear();
  }
}

bool LocationMapper::RollbackToCheckpoint()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!has_checkpoint_ || !checkpoint_cloud_)
  {
    return false;
  }
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(*checkpoint_cloud_));
  point_ids_ = checkpoint_ids_;
  point_obs_counts_ = checkpoint_obs_counts_;
  point_types_ = checkpoint_types_;
  point_confidences_ = checkpoint_confidences_;
  cloud_->width = cloud_->points.size();
  cloud_->height = 1;
  kdtree_.setInputCloud(cloud_);
  first_cone_msg_ = !cloud_->empty();
  map_frozen_ = true;
  next_id_ = 0;
  for (int id : point_ids_)
  {
    next_id_ = std::max(next_id_, id);
  }
  return true;
}

bool LocationMapper::SaveMapToFile(const std::string &path, std::string *error_msg) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::ofstream out(path.c_str(), std::ios::out | std::ios::trunc);
  if (!out.is_open())
  {
    if (error_msg) *error_msg = "failed_to_open_file";
    return false;
  }

  out << "# localization_map_v1\n";
  out << "# map_mode=" << params_.map_mode
      << ",map_frozen=" << (map_frozen_ ? 1 : 0)
      << ",next_id=" << next_id_ << "\n";
  out << "id,x,y,z,obs_count,type,confidence\n";
  for (size_t i = 0; i < cloud_->size(); ++i)
  {
    const int id = (i < point_ids_.size()) ? point_ids_[i] : 0;
    const int obs = (i < point_obs_counts_.size()) ? point_obs_counts_[i] : 1;
    const std::uint8_t type = (i < point_types_.size()) ? point_types_[i] : kConeNone;
    const double conf = (i < point_confidences_.size()) ? point_confidences_[i] : 0.0;
    out << id << ","
        << cloud_->points[i].x << ","
        << cloud_->points[i].y << ","
        << cloud_->points[i].z << ","
        << obs << ","
        << static_cast<int>(type) << ","
        << conf << "\n";
  }

  if (!out.good())
  {
    if (error_msg) *error_msg = "failed_to_write_file";
    return false;
  }
  return true;
}

bool LocationMapper::LoadMapFromFile(const std::string &path, std::string *error_msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::ifstream in(path.c_str(), std::ios::in);
  if (!in.is_open())
  {
    if (error_msg) *error_msg = "failed_to_open_file";
    return false;
  }

  PointCloudPtr loaded_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> loaded_ids;
  std::vector<int> loaded_obs;
  std::vector<std::uint8_t> loaded_types;
  std::vector<double> loaded_conf;

  std::string line;
  while (std::getline(in, line))
  {
    if (line.empty() || line[0] == '#')
    {
      continue;
    }
    if (line.find("id,") == 0)
    {
      continue;
    }
    std::stringstream ss(line);
    std::string tok;
    std::vector<std::string> fields;
    while (std::getline(ss, tok, ','))
    {
      fields.push_back(tok);
    }
    if (fields.size() < 7)
    {
      continue;
    }
    try
    {
      int id = std::stoi(fields[0]);
      pcl::PointXYZ pt;
      pt.x = static_cast<float>(std::stod(fields[1]));
      pt.y = static_cast<float>(std::stod(fields[2]));
      pt.z = static_cast<float>(std::stod(fields[3]));
      int obs = std::stoi(fields[4]);
      int type = std::stoi(fields[5]);
      double conf = std::stod(fields[6]);

      loaded_cloud->push_back(pt);
      loaded_ids.push_back(id);
      loaded_obs.push_back(std::max(1, obs));
      loaded_types.push_back(normalizeConeType(static_cast<std::uint8_t>(type)));
      loaded_conf.push_back(std::min(1.0, std::max(0.0, conf)));
    }
    catch (const std::exception &)
    {
      continue;
    }
  }

  loaded_cloud->width = loaded_cloud->points.size();
  loaded_cloud->height = 1;
  cloud_ = loaded_cloud;
  point_ids_ = std::move(loaded_ids);
  point_obs_counts_ = std::move(loaded_obs);
  point_types_ = std::move(loaded_types);
  point_confidences_ = std::move(loaded_conf);
  kdtree_.setInputCloud(cloud_);
  first_cone_msg_ = !cloud_->empty();
  map_frozen_ = false;

  next_id_ = 0;
  for (int id : point_ids_)
  {
    next_id_ = std::max(next_id_, id);
  }

  checkpoint_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
  checkpoint_ids_ = point_ids_;
  checkpoint_obs_counts_ = point_obs_counts_;
  checkpoint_types_ = point_types_;
  checkpoint_confidences_ = point_confidences_;
  has_checkpoint_ = true;
  return true;
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
