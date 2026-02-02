#include <localization_core/imu_state_estimator.hpp>

#include <ros/ros.h>
#include <autodrive_msgs/HUAT_InsP2.h>
#include <autodrive_msgs/HUAT_CarState.h>

class StateEstimatorNode
{
public:
  StateEstimatorNode()
      : nh_(),
        pnh_("~"),
        params_(LoadParams()),
        estimator_(params_)
  {
    pnh_.param<std::string>("topics/ins", imu_topic_, "sensors/ins");
    pnh_.param<std::string>("topics/car_state", carstate_topic_, "localization/car_state");
    pnh_.param<std::string>("frames/world", world_frame_, "world");

    imu_sub_ = nh_.subscribe(imu_topic_, 50, &StateEstimatorNode::ImuCallback, this);
    carstate_pub_ = nh_.advertise<autodrive_msgs::HUAT_CarState>(carstate_topic_, 10);
  }

private:
  localization_core::ImuStateEstimatorParams LoadParams()
  {
    localization_core::ImuStateEstimatorParams params;

    pnh_.param("use_gnss", params.use_gnss, true);
    pnh_.param("use_velocity", params.use_velocity, true);
    pnh_.param("use_yaw", params.use_yaw, true);

    pnh_.param("accel_noise", params.accel_noise, 1.0);
    pnh_.param("gyro_noise", params.gyro_noise, 0.05);

    pnh_.param("meas_pos_noise", params.meas_pos_noise, 0.5);
    pnh_.param("meas_vel_noise", params.meas_vel_noise, 0.2);
    pnh_.param("meas_yaw_noise", params.meas_yaw_noise, 0.05);

    pnh_.param("init_pos_var", params.init_pos_var, 1.0);
    pnh_.param("init_vel_var", params.init_vel_var, 1.0);
    pnh_.param("init_yaw_var", params.init_yaw_var, 0.1);

    pnh_.param("min_dt", params.min_dt, 0.001);
    pnh_.param("max_dt", params.max_dt, 0.1);

    pnh_.param("accel_gravity", params.accel_gravity, 9.79);

    if (!pnh_.param("length/frontToIMUdistanceX", params.front_to_imu_x, 0.0))
    {
      nh_.param("length/frontToIMUdistanceX", params.front_to_imu_x, 0.0);
    }
    if (!pnh_.param("length/frontToIMUdistanceY", params.front_to_imu_y, 0.0))
    {
      nh_.param("length/frontToIMUdistanceY", params.front_to_imu_y, 0.0);
    }
    if (!pnh_.param("length/frontToIMUdistanceZ", params.front_to_imu_z, 0.0))
    {
      nh_.param("length/frontToIMUdistanceZ", params.front_to_imu_z, 0.0);
    }
    if (!pnh_.param("length/rearToIMUdistanceX", params.rear_to_imu_x, 0.0))
    {
      nh_.param("length/rearToIMUdistanceX", params.rear_to_imu_x, 0.0);
    }
    if (!pnh_.param("length/rearToIMUdistanceY", params.rear_to_imu_y, 0.0))
    {
      nh_.param("length/rearToIMUdistanceY", params.rear_to_imu_y, 0.0);
    }
    if (!pnh_.param("length/rearToIMUdistanceZ", params.rear_to_imu_z, 0.0))
    {
      nh_.param("length/rearToIMUdistanceZ", params.rear_to_imu_z, 0.0);
    }

    return params;
  }

  static localization_core::Asensing ToCore(const autodrive_msgs::HUAT_InsP2 &msg)
  {
    localization_core::Asensing out;
    
    // 位置 (WGS84)
    out.latitude = msg.Lat;
    out.longitude = msg.Lon;
    out.altitude = msg.Altitude;
    
    // 速度 (m/s, NED坐标系)
    // HUAT_InsP2.Vd 向下为正
    out.north_velocity = msg.Vn;
    out.east_velocity = msg.Ve;
    out.ground_velocity = msg.Vd;  // Vd(向下)
    
    // 姿态角 (度)
    out.roll = msg.Roll;
    out.pitch = msg.Pitch;
    out.azimuth = msg.Heading;
    
    // 角速度 (rad/s, FRD车体系)
    out.x_angular_velocity = msg.gyro_x;
    out.y_angular_velocity = msg.gyro_y;
    out.z_angular_velocity = msg.gyro_z;
    
    // 加速度 (m/s², FRD车体系)
    out.x_acc = msg.acc_x;
    out.y_acc = msg.acc_y;
    out.z_acc = msg.acc_z;
    
    return out;
  }

  static void ToRos(const localization_core::CarState &state, autodrive_msgs::HUAT_CarState *out)
  {
    if (!out)
    {
      return;
    }
    out->car_state.x = state.car_state.x;
    out->car_state.y = state.car_state.y;
    out->car_state.theta = state.car_state.theta;
    out->car_state_front.x = state.car_state_front.x;
    out->car_state_front.y = state.car_state_front.y;
    out->car_state_front.z = state.car_state_front.z;
    out->car_state_rear.x = state.car_state_rear.x;
    out->car_state_rear.y = state.car_state_rear.y;
    out->car_state_rear.z = state.car_state_rear.z;
    out->V = static_cast<float>(state.V);
    out->W = static_cast<float>(state.W);
    out->A = static_cast<float>(state.A);
  }

  void ImuCallback(const autodrive_msgs::HUAT_InsP2::ConstPtr &msg)
  {
    autodrive_msgs::HUAT_CarState state_msg;
    localization_core::CarState state;
    const double stamp = msg->header.stamp.toSec();
    if (estimator_.Process(ToCore(*msg), stamp, &state))
    {
      ToRos(state, &state_msg);
      state_msg.header.stamp = msg->header.stamp;
      state_msg.header.frame_id = world_frame_;
      carstate_pub_.publish(state_msg);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string imu_topic_;
  std::string carstate_topic_;
  std::string world_frame_;
  localization_core::ImuStateEstimatorParams params_;
  localization_core::ImuStateEstimator estimator_;
  ros::Subscriber imu_sub_;
  ros::Publisher carstate_pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_estimator_node");
  StateEstimatorNode node;
  ros::spin();
  return 0;
}
