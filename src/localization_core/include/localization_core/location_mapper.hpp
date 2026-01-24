#pragma once

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

  bool has_carstate() const { return has_carstate_; }
  const CarState &car_state() const { return car_state_; }

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

  LocationParams params_;
  std::string data_root_;

  bool first_cone_msg_ = false;
  bool has_carstate_ = false;

  CarState car_state_;
  Asensing mimu_;
  Asensing mins_;

  int next_id_ = 0;
  double enu_xyz_[3] = {0.0, 0.0, 0.0};
  double front_wheel_[3] = {0.0, 0.0, 0.0};
  double rear_wheel_[3] = {0.0, 0.0, 0.0};
  double first_lat_ = 0.0;
  double first_lon_ = 0.0;
  double first_alt_ = 0.0;
  double standard_azimuth_ = 0.0;

  PointCloudPtr cloud_;
  std::vector<int> point_ids_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
};

}  // namespace localization_core
