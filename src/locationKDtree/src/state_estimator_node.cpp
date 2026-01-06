#include "imu_state_estimator.hpp"

#include <ros/ros.h>

class StateEstimatorNode
{
public:
  StateEstimatorNode()
      : nh_(),
        pnh_("~"),
        params_(LoadParams()),
        estimator_(params_)
  {
    pnh_.param<std::string>("imu_topic", imu_topic_, "/pbox_pub/Ins");
    pnh_.param<std::string>("carstate_topic", carstate_topic_, "/Carstate");

    imu_sub_ = nh_.subscribe(imu_topic_, 50, &StateEstimatorNode::ImuCallback, this);
    carstate_pub_ = nh_.advertise<common_msgs::HUAT_Carstate>(carstate_topic_, 10);
  }

private:
  state_estimation::ImuStateEstimatorParams LoadParams()
  {
    state_estimation::ImuStateEstimatorParams params;

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

    nh_.param("length/frontToIMUdistanceX", params.front_to_imu_x, 0.0);
    nh_.param("length/frontToIMUdistanceY", params.front_to_imu_y, 0.0);
    nh_.param("length/frontToIMUdistanceZ", params.front_to_imu_z, 0.0);
    nh_.param("length/rearToIMUdistanceX", params.rear_to_imu_x, 0.0);
    nh_.param("length/rearToIMUdistanceY", params.rear_to_imu_y, 0.0);
    nh_.param("length/rearToIMUdistanceZ", params.rear_to_imu_z, 0.0);

    return params;
  }

  void ImuCallback(const common_msgs::HUAT_ASENSING::ConstPtr &msg)
  {
    common_msgs::HUAT_Carstate state_msg;
    const ros::Time stamp = ros::Time::now();
    if (estimator_.Process(*msg, stamp, &state_msg))
    {
      carstate_pub_.publish(state_msg);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string imu_topic_;
  std::string carstate_topic_;
  state_estimation::ImuStateEstimatorParams params_;
  state_estimation::ImuStateEstimator estimator_;
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
