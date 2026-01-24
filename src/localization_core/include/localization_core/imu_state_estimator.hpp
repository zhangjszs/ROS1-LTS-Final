#pragma once

#include <Eigen/Dense>

#include <localization_core/types.hpp>

namespace localization_core {

struct ImuStateEstimatorParams {
  bool use_gnss = true;
  bool use_velocity = true;
  bool use_yaw = true;

  double accel_noise = 1.0;
  double gyro_noise = 0.05;

  double meas_pos_noise = 0.5;
  double meas_vel_noise = 0.2;
  double meas_yaw_noise = 0.05;

  double init_pos_var = 1.0;
  double init_vel_var = 1.0;
  double init_yaw_var = 0.1;

  double min_dt = 0.001;
  double max_dt = 0.1;

  double accel_gravity = 9.79;
  double gyro_scale = 1.0;

  double front_to_imu_x = 0.0;
  double front_to_imu_y = 0.0;
  double front_to_imu_z = 0.0;
  double rear_to_imu_x = 0.0;
  double rear_to_imu_y = 0.0;
  double rear_to_imu_z = 0.0;
};

class ImuStateEstimator {
 public:
  explicit ImuStateEstimator(const ImuStateEstimatorParams &params);

  bool Process(const Asensing &msg, double stamp_sec, CarState *out);

  bool initialized() const { return initialized_; }

 private:
  void Initialize(const Asensing &msg, double stamp_sec);
  void Predict(double dt, double ax, double ay, double gyro_z);
  void Update(const Eigen::VectorXd &z,
              const Eigen::MatrixXd &H,
              const Eigen::MatrixXd &R,
              int yaw_index);

  void GeoDeticToENU(double lat, double lon, double h,
                     double lat0, double lon0, double h0,
                     double &east, double &north, double &up) const;

  double NormalizeAngle(double angle) const;

  void ToMapFrame(double east, double north, double &x, double &y) const;

  ImuStateEstimatorParams params_;

  bool initialized_ = false;
  double last_time_sec_ = 0.0;

  double origin_lat_ = 0.0;
  double origin_lon_ = 0.0;
  double origin_alt_ = 0.0;
  double standard_azimuth_deg_ = 0.0;
  double origin_rot_rad_ = 0.0;
  double cos_origin_ = 1.0;
  double sin_origin_ = 0.0;
  double last_up_ = 0.0;

  Eigen::Matrix<double, 5, 1> x_;
  Eigen::Matrix<double, 5, 5> P_;
};

}  // namespace localization_core
