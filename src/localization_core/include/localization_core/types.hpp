#pragma once

#include <cstdint>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace localization_core {

struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Pose2 {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

struct CarState {
  Pose2 car_state;
  Point3 car_state_front;
  Point3 car_state_rear;
  double V = 0.0;
  double W = 0.0;
  double A = 0.0;
};

struct Asensing {
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double north_velocity = 0.0;
  double east_velocity = 0.0;
  double ground_velocity = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double azimuth = 0.0;
  double x_angular_velocity = 0.0;
  double y_angular_velocity = 0.0;
  double z_angular_velocity = 0.0;
  double x_acc = 0.0;
  double y_acc = 0.0;
  double z_acc = 0.0;
};

struct Cone {
  Point3 position_base_link;
  Point3 position_global;
  std::uint32_t id = 0;
  std::uint32_t confidence = 0;
  std::uint32_t type = 0;
};

struct ConeMap {
  std::vector<Cone> cones;
};

struct ConeDetections {
  std::vector<Point3> points;
};

struct LocationParams {
  double lidar_to_imu_dist = 1.87;
  double front_to_imu_x = 0.0;
  double front_to_imu_y = 0.0;
  double front_to_imu_z = 0.0;
  double rear_to_imu_x = 0.0;
  double rear_to_imu_y = 0.0;
  double rear_to_imu_z = 0.0;
};

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;

}  // namespace localization_core
