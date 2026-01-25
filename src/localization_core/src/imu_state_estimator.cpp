#include <localization_core/imu_state_estimator.hpp>

#include <cmath>

namespace localization_core {
namespace {
constexpr double kDegToRad = M_PI / 180.0;
}

ImuStateEstimator::ImuStateEstimator(const ImuStateEstimatorParams &params)
    : params_(params)
{
  x_.setZero();
  P_.setZero();
}

bool ImuStateEstimator::Process(const Asensing &msg, double stamp_sec, CarState *out)
{
  if (!initialized_)
  {
    Initialize(msg, stamp_sec);
    if (out)
    {
      out->car_state.x = x_(0);
      out->car_state.y = x_(1);
      out->car_state.theta = NormalizeAngle(x_(4));
      out->V = std::hypot(x_(2), x_(3));
      out->W = std::sqrt(msg.x_angular_velocity * msg.x_angular_velocity +
                         msg.y_angular_velocity * msg.y_angular_velocity +
                         msg.z_angular_velocity * msg.z_angular_velocity);
      const double ax = msg.x_acc * params_.accel_gravity;
      const double ay = msg.y_acc * params_.accel_gravity;
      const double az = msg.z_acc * params_.accel_gravity;
      out->A = std::sqrt(ax * ax + ay * ay + az * az);

      double front_dx = 0.0;
      double front_dy = 0.0;
      double rear_dx = 0.0;
      double rear_dy = 0.0;
      ToMapFrame(params_.front_to_imu_x, params_.front_to_imu_y, front_dx, front_dy);
      ToMapFrame(params_.rear_to_imu_x, params_.rear_to_imu_y, rear_dx, rear_dy);

      out->car_state_front.x = x_(0) + front_dx;
      out->car_state_front.y = x_(1) + front_dy;
      out->car_state_front.z = last_up_ + params_.front_to_imu_z;

      out->car_state_rear.x = x_(0) + rear_dx;
      out->car_state_rear.y = x_(1) + rear_dy;
      out->car_state_rear.z = last_up_ + params_.rear_to_imu_z;
    }
    return true;
  }

  double dt = stamp_sec - last_time_sec_;
  if (dt <= 0.0)
  {
    return false;
  }
  if (dt < params_.min_dt)
  {
    dt = params_.min_dt;
  }
  else if (dt > params_.max_dt)
  {
    dt = params_.max_dt;
  }

  const double ax = msg.x_acc * params_.accel_gravity;
  const double ay = msg.y_acc * params_.accel_gravity;
  const double gyro_z = msg.z_angular_velocity;

  Predict(dt, ax, ay, gyro_z);

  int meas_dim = 0;
  if (params_.use_gnss)
  {
    meas_dim += 2;
  }
  if (params_.use_velocity)
  {
    meas_dim += 2;
  }
  if (params_.use_yaw)
  {
    meas_dim += 1;
  }

  if (meas_dim > 0)
  {
    Eigen::VectorXd z(meas_dim);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(meas_dim, 5);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(meas_dim, meas_dim);

    int idx = 0;
    int yaw_index = -1;

    if (params_.use_gnss)
    {
      double east = 0.0;
      double north = 0.0;
      double up = 0.0;
      GeoDeticToENU(msg.latitude * kDegToRad,
                    msg.longitude * kDegToRad,
                    msg.altitude,
                    origin_lat_,
                    origin_lon_,
                    origin_alt_,
                    east,
                    north,
                    up);
      last_up_ = up;
      double meas_x = 0.0;
      double meas_y = 0.0;
      ToMapFrame(east, north, meas_x, meas_y);

      z(idx) = meas_x;
      H(idx, 0) = 1.0;
      R(idx, idx) = params_.meas_pos_noise * params_.meas_pos_noise;
      idx++;

      z(idx) = meas_y;
      H(idx, 1) = 1.0;
      R(idx, idx) = params_.meas_pos_noise * params_.meas_pos_noise;
      idx++;
    }

    if (params_.use_velocity)
    {
      double vel_x = 0.0;
      double vel_y = 0.0;
      ToMapFrame(msg.east_velocity, msg.north_velocity, vel_x, vel_y);

      z(idx) = vel_x;
      H(idx, 2) = 1.0;
      R(idx, idx) = params_.meas_vel_noise * params_.meas_vel_noise;
      idx++;

      z(idx) = vel_y;
      H(idx, 3) = 1.0;
      R(idx, idx) = params_.meas_vel_noise * params_.meas_vel_noise;
      idx++;
    }

    if (params_.use_yaw)
    {
      const double diff = -(msg.azimuth - standard_azimuth_deg_);
      const double meas_yaw = NormalizeAngle(diff * kDegToRad);
      yaw_index = idx;

      z(idx) = meas_yaw;
      H(idx, 4) = 1.0;
      R(idx, idx) = params_.meas_yaw_noise * params_.meas_yaw_noise;
      idx++;
    }

    Update(z, H, R, yaw_index);
  }

  last_time_sec_ = stamp_sec;

  if (out)
  {
    out->car_state.x = x_(0);
    out->car_state.y = x_(1);
    out->car_state.theta = NormalizeAngle(x_(4));
    out->V = std::hypot(x_(2), x_(3));
    out->W = std::sqrt(msg.x_angular_velocity * msg.x_angular_velocity +
                       msg.y_angular_velocity * msg.y_angular_velocity +
                       msg.z_angular_velocity * msg.z_angular_velocity);
    const double ax_m = msg.x_acc * params_.accel_gravity;
    const double ay_m = msg.y_acc * params_.accel_gravity;
    const double az_m = msg.z_acc * params_.accel_gravity;
    out->A = std::sqrt(ax_m * ax_m + ay_m * ay_m + az_m * az_m);

    double front_dx = 0.0;
    double front_dy = 0.0;
    double rear_dx = 0.0;
    double rear_dy = 0.0;
    ToMapFrame(params_.front_to_imu_x, params_.front_to_imu_y, front_dx, front_dy);
    ToMapFrame(params_.rear_to_imu_x, params_.rear_to_imu_y, rear_dx, rear_dy);

    out->car_state_front.x = x_(0) + front_dx;
    out->car_state_front.y = x_(1) + front_dy;
    out->car_state_front.z = last_up_ + params_.front_to_imu_z;

    out->car_state_rear.x = x_(0) + rear_dx;
    out->car_state_rear.y = x_(1) + rear_dy;
    out->car_state_rear.z = last_up_ + params_.rear_to_imu_z;
  }

  return true;
}

void ImuStateEstimator::Initialize(const Asensing &msg, double stamp_sec)
{
  origin_lat_ = msg.latitude * kDegToRad;
  origin_lon_ = msg.longitude * kDegToRad;
  origin_alt_ = msg.altitude;
  standard_azimuth_deg_ = msg.azimuth;

  origin_rot_rad_ = (standard_azimuth_deg_ - 90.0) * kDegToRad;
  cos_origin_ = std::cos(origin_rot_rad_);
  sin_origin_ = std::sin(origin_rot_rad_);

  x_.setZero();
  double vel_x = 0.0;
  double vel_y = 0.0;
  ToMapFrame(msg.east_velocity, msg.north_velocity, vel_x, vel_y);
  x_(2) = vel_x;
  x_(3) = vel_y;
  x_(4) = 0.0;

  P_.setZero();
  P_(0, 0) = params_.init_pos_var;
  P_(1, 1) = params_.init_pos_var;
  P_(2, 2) = params_.init_vel_var;
  P_(3, 3) = params_.init_vel_var;
  P_(4, 4) = params_.init_yaw_var;

  last_time_sec_ = stamp_sec;
  initialized_ = true;
}

void ImuStateEstimator::Predict(double dt, double ax, double ay, double gyro_z)
{
  const double yaw = x_(4);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double a_world_x = cos_yaw * ax - sin_yaw * ay;
  const double a_world_y = sin_yaw * ax + cos_yaw * ay;

  x_(0) += x_(2) * dt + 0.5 * a_world_x * dt * dt;
  x_(1) += x_(3) * dt + 0.5 * a_world_y * dt * dt;
  x_(2) += a_world_x * dt;
  x_(3) += a_world_y * dt;
  x_(4) = NormalizeAngle(x_(4) + gyro_z * params_.gyro_scale * dt);

  Eigen::Matrix<double, 5, 5> F = Eigen::Matrix<double, 5, 5>::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;
  F(2, 4) = (-sin_yaw * ax - cos_yaw * ay) * dt;
  F(3, 4) = (cos_yaw * ax - sin_yaw * ay) * dt;

  Eigen::Matrix<double, 5, 5> Q = Eigen::Matrix<double, 5, 5>::Zero();
  Q(0, 0) = params_.accel_noise * params_.accel_noise * dt * dt;
  Q(1, 1) = params_.accel_noise * params_.accel_noise * dt * dt;
  Q(2, 2) = params_.accel_noise * params_.accel_noise * dt * dt;
  Q(3, 3) = params_.accel_noise * params_.accel_noise * dt * dt;
  Q(4, 4) = params_.gyro_noise * params_.gyro_noise * dt * dt;

  P_ = F * P_ * F.transpose() + Q;
}

void ImuStateEstimator::Update(const Eigen::VectorXd &z,
                               const Eigen::MatrixXd &H,
                               const Eigen::MatrixXd &R,
                               int yaw_index)
{
  Eigen::VectorXd y = z - H * x_;
  if (yaw_index >= 0)
  {
    y(yaw_index) = NormalizeAngle(y(yaw_index));
  }

  Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  // Use LDLT decomposition for better numerical stability instead of direct inverse
  // K = P * H^T * S^-1
  // K^T = S^-1 * H * P
  // S * K^T = H * P
  Eigen::MatrixXd K = (S.ldlt().solve(H * P_)).transpose();

  x_ = x_ + K * y;
  if (yaw_index >= 0)
  {
    x_(4) = NormalizeAngle(x_(4));
  }

  Eigen::Matrix<double, 5, 5> I = Eigen::Matrix<double, 5, 5>::Identity();
  P_ = (I - K * H) * P_;
}

void ImuStateEstimator::GeoDeticToENU(double lat, double lon, double h,
                                      double lat0, double lon0, double h0,
                                      double &east, double &north, double &up) const
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

  const double dx = x - x0;
  const double dy = y - y0;
  const double dz = z - z0;

  east = -sin_lon0 * dx + cos_lon0 * dy;
  north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz;
  up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz;
}

double ImuStateEstimator::NormalizeAngle(double angle) const
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

void ImuStateEstimator::ToMapFrame(double east, double north, double &x, double &y) const
{
  x = east * cos_origin_ - north * sin_origin_;
  y = east * sin_origin_ + north * cos_origin_;
}

}  // namespace localization_core
