#ifndef IMU_STATE_ESTIMATOR_HPP
#define IMU_STATE_ESTIMATOR_HPP

#include <Eigen/Dense>
#include <ros/ros.h>

#include "common_msgs/HUAT_ASENSING.h"
#include "common_msgs/HUAT_Carstate.h"

namespace state_estimation
{
struct ImuStateEstimatorParams
{
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

class ImuStateEstimator
{
public:
  explicit ImuStateEstimator(const ImuStateEstimatorParams &params);

  bool Process(const common_msgs::HUAT_ASENSING &msg,
               const ros::Time &stamp,
               common_msgs::HUAT_Carstate *out);

  bool initialized() const { return initialized_; }

private:
  void Initialize(const common_msgs::HUAT_ASENSING &msg, const ros::Time &stamp);
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
  ros::Time last_time_;

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
}  // namespace state_estimation

#endif  // IMU_STATE_ESTIMATOR_HPP
