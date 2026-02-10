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
      // HUAT_InsP2 加速度已经是 m/s²，不再乘以 g
      out->A = std::sqrt(msg.x_acc * msg.x_acc + msg.y_acc * msg.y_acc + msg.z_acc * msg.z_acc);

      // FSSIM风格扩展状态
      out->Vy = 0.0;                         // 初始横向速度为0
      out->Wz = msg.z_angular_velocity;      // 偏航角速度
      out->Ax = msg.x_acc;                   // 纵向加速度
      out->Ay = msg.y_acc;                   // 横向加速度

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

  // 去除重力在车体坐标系x/y轴上的投影分量
  // FRD车体系下，重力投影: gx = g*sin(pitch), gy = -g*sin(roll)*cos(pitch)
  // HUAT_InsP2 加速度已经是 m/s²，不再乘以 g
  const double g = params_.accel_gravity;
  const double roll_rad = msg.roll * kDegToRad;
  const double pitch_rad = msg.pitch * kDegToRad;
  const double ax = msg.x_acc - g * std::sin(pitch_rad);
  const double ay = msg.y_acc - (-g * std::sin(roll_rad) * std::cos(pitch_rad));
  const double gyro_z = msg.z_angular_velocity;

  Predict(dt, ax, ay, gyro_z);

  // FSSIM-style low-speed kinematic correction
  // 从角速度和速度估计转向角: delta ≈ atan(L * r / vx)
  if (params_.enable_kinematic_correction)
  {
    const double speed = std::hypot(x_(2), x_(3));
    if (speed > 0.1)  // 避免除零
    {
      // 估计转向角
      const double estimated_steering = std::atan(params_.wheelbase * gyro_z / speed);
      last_steering_ = estimated_steering;
    }
    ApplyKinematicCorrection(last_steering_);
  }

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
    // HUAT_InsP2 加速度已经是 m/s²，不再乘以 g
    out->A = std::sqrt(msg.x_acc * msg.x_acc + msg.y_acc * msg.y_acc + msg.z_acc * msg.z_acc);

    // FSSIM风格扩展状态
    // 计算车体坐标系下的速度
    const double yaw = x_(4);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    // 从世界坐标系速度转换到车体坐标系
    // vx_body = vx_world * cos(yaw) + vy_world * sin(yaw)
    // vy_body = -vx_world * sin(yaw) + vy_world * cos(yaw)
    out->Vy = -x_(2) * sin_yaw + x_(3) * cos_yaw;  // 横向速度
    out->Wz = msg.z_angular_velocity;              // 偏航角速度
    out->Ax = msg.x_acc;                           // 纵向加速度
    out->Ay = msg.y_acc;                           // 横向加速度

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
  // ∂(pos)/∂θ = 0.5 * ∂(a_world)/∂θ * dt²
  F(0, 4) = 0.5 * (-sin_yaw * ax - cos_yaw * ay) * dt * dt;
  F(1, 4) = 0.5 * (cos_yaw * ax - sin_yaw * ay) * dt * dt;
  // ∂(vel)/∂θ = ∂(a_world)/∂θ * dt
  F(2, 4) = (-sin_yaw * ax - cos_yaw * ay) * dt;
  F(3, 4) = (cos_yaw * ax - sin_yaw * ay) * dt;

  // Q矩阵：基于连续白噪声加速度模型的离散化
  // 位置噪声 ~ σ_a² * dt⁴/4, 速度噪声 ~ σ_a² * dt², 交叉项 ~ σ_a² * dt³/2
  const double sa2 = params_.accel_noise * params_.accel_noise;
  const double sg2 = params_.gyro_noise * params_.gyro_noise;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt2 * dt2;

  Eigen::Matrix<double, 5, 5> Q = Eigen::Matrix<double, 5, 5>::Zero();
  Q(0, 0) = sa2 * dt4 * 0.25;   // pos_x
  Q(1, 1) = sa2 * dt4 * 0.25;   // pos_y
  Q(2, 2) = sa2 * dt2;           // vel_x
  Q(3, 3) = sa2 * dt2;           // vel_y
  Q(0, 2) = sa2 * dt3 * 0.5;    // pos_x - vel_x 交叉项
  Q(2, 0) = sa2 * dt3 * 0.5;
  Q(1, 3) = sa2 * dt3 * 0.5;    // pos_y - vel_y 交叉项
  Q(3, 1) = sa2 * dt3 * 0.5;
  Q(4, 4) = sg2 * dt2;           // yaw

  P_ = F * P_ * F.transpose() + Q;

  // 协方差健康检查：NaN时重置为初始值，否则限制对角线范围
  if (P_.hasNaN() || !P_.allFinite())
  {
    P_.setZero();
    P_(0, 0) = params_.init_pos_var;
    P_(1, 1) = params_.init_pos_var;
    P_(2, 2) = params_.init_vel_var;
    P_(3, 3) = params_.init_vel_var;
    P_(4, 4) = params_.init_yaw_var;
  }
  else
  {
    // 限制对角线元素范围，防止协方差爆炸或收缩到零
    constexpr double kMinVar = 1e-6;
    constexpr double kMaxVar = 1e4;
    for (int i = 0; i < 5; ++i)
    {
      P_(i, i) = std::fmax(kMinVar, std::fmin(kMaxVar, P_(i, i)));
    }
  }
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
  Eigen::Matrix<double, 5, 5> I_KH = I - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();

  if (P_.hasNaN() || !P_.allFinite())
  {
    P_.setZero();
    P_(0, 0) = params_.init_pos_var;
    P_(1, 1) = params_.init_pos_var;
    P_(2, 2) = params_.init_vel_var;
    P_(3, 3) = params_.init_vel_var;
    P_(4, 4) = params_.init_yaw_var;
  }
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
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0)
  {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

void ImuStateEstimator::ToMapFrame(double east, double north, double &x, double &y) const
{
  x = east * cos_origin_ - north * sin_origin_;
  y = east * sin_origin_ + north * cos_origin_;
}

void ImuStateEstimator::ApplyKinematicCorrection(double steering)
{
  if (!params_.enable_kinematic_correction)
  {
    return;
  }

  // 计算当前速度（世界坐标系）
  const double vx_world = x_(2);
  const double vy_world = x_(3);
  const double yaw = x_(4);

  // 转换到车体坐标系
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double vx_body = vx_world * cos_yaw + vy_world * sin_yaw;   // 纵向速度
  const double vy_body = -vx_world * sin_yaw + vy_world * cos_yaw;  // 横向速度

  const double speed = std::hypot(vx_body, vy_body);

  // 计算混合因子 (FSSIM风格)
  // blend = 0: 纯运动学模型 (低速)
  // blend = 1: 纯动力学模型 (高速)
  const double v_blend = (speed - params_.kinematic_blend_speed) / params_.kinematic_blend_range;
  const double blend = std::fmax(std::fmin(v_blend, 1.0), 0.0);

  if (blend >= 1.0)
  {
    // 高速：不需要修正，使用动力学模型结果
    return;
  }

  // 运动学模型计算横向速度和偏航角速度
  // 基于自行车模型：
  // vy_kin = tan(delta) * vx * l_r / L
  // r_kin = tan(delta) * vx / L
  const double L = params_.wheelbase;
  const double l_r = params_.cg_to_rear;

  // 限制转向角避免tan发散
  const double max_steering = 0.6;  // ~34度
  const double delta = std::fmax(std::fmin(steering, max_steering), -max_steering);

  const double tan_delta = std::tan(delta);
  const double vy_kin = tan_delta * std::fabs(vx_body) * l_r / L;
  const double r_kin = tan_delta * vx_body / L;

  // 混合运动学和动力学结果
  const double vy_corrected = blend * vy_body + (1.0 - blend) * vy_kin;

  // 将修正后的车体速度转回世界坐标系
  // vx_world = vx_body * cos(yaw) - vy_body * sin(yaw)
  // vy_world = vx_body * sin(yaw) + vy_body * cos(yaw)
  x_(2) = vx_body * cos_yaw - vy_corrected * sin_yaw;
  x_(3) = vx_body * sin_yaw + vy_corrected * cos_yaw;

  // 注意：我们不直接修正偏航角速度，因为它来自陀螺仪测量
  // 但可以用于调试输出
  (void)r_kin;
}

}  // namespace localization_core
