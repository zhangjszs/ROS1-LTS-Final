#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <localization_core/types.hpp>
#include <pcl/kdtree/kdtree_flann.h>

namespace localization_core {

class LocationMapper {
 public:
  explicit LocationMapper(const LocationParams &params);

  void Configure(const LocationParams &params);
  void SetDataDirectory(const std::string &path);

  bool has_carstate() const { std::lock_guard<std::mutex> lk(state_mutex_); return has_carstate_; }
  CarState car_state() const { std::lock_guard<std::mutex> lk(state_mutex_); return car_state_; }
  double standard_azimuth() const { std::lock_guard<std::mutex> lk(state_mutex_); return standard_azimuth_; }

  bool UpdateFromIns(const Asensing &imu, CarState *state_out);
  void UpdateFromCarState(const CarState &state);
  bool UpdateFromCones(const ConeDetections &detections,
                       ConeMap *map_out,
                       PointCloudPtr *cloud_out);

 private:
  void GeoDeticToENU(double lat, double lon, double h,
                     double lat0, double lon0, double h0,
                     double enu_xyz[3]);

  int getNewId();

  void saveCarstate(double x, double y);

  bool passesGeometryFilter(double lx, double ly) const;

  void interpolateMissingCones(ConeMap *map_out);

  LocationParams params_;
  std::string data_root_;

  bool first_cone_msg_ = false;
  bool has_carstate_ = false;

  CarState car_state_;
  Asensing mimu_;
  Asensing mins_;

  int next_id_ = 0;
  double enu_xyz_[3] = {0.0, 0.0, 0.0};
  double first_lat_ = 0.0;
  double first_lon_ = 0.0;
  double first_alt_ = 0.0;
  double standard_azimuth_ = 0.0;

  PointCloudPtr cloud_;
  std::vector<int> point_ids_;
  std::vector<int> point_obs_counts_;  // 每个锥桶的观测次数（用于加权合并）
  std::vector<std::uint8_t> point_types_;  // 每个锥桶类型: 0/1/2/3/4
  std::vector<double> point_confidences_;  // 每个锥桶的置信度 [0.0, 1.0]
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;

  mutable std::mutex state_mutex_;
};

}  // namespace localization_core
