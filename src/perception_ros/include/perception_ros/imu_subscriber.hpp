#ifndef MULTI_SENSOR_FUSION_IMU_SUBSCRIBER_HPP_
#define MULTI_SENSOR_FUSION_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>

#include <autodrive_msgs/HUAT_InsP2.h>
#include <perception_core/imu_data.hpp>
class IMUSubscriber{
public:
      IMUSubscriber(ros::NodeHandle nh,std::string topic_name, size_t buff_size);
      IMUSubscriber() = default;


      void ParseData(std::deque<IMUData>& imu_data_buff);
      void SetInsInfo(autodrive_msgs::HUAT_InsP2ConstPtr imu_msg_ptr);
      bool SyncData(std::deque<IMUData>& UnsyncedData, IMUData& synced_data,double sync_time);
      bool readAndSync(std::deque<IMUData>& unsynced_imu_, IMUData& synced_data, double sync_time);
  private:
  	ros::NodeHandle nh_;
  	ros::Subscriber subscriber_;
  	std::deque<IMUData> imu_data_buff_;
    std::deque<IMUData> new_imu_data_;
    IMUData pre_imu_data;
    std::mutex mtx; 
    void msg_callback(const autodrive_msgs::HUAT_InsP2ConstPtr imu_msg_ptr);
    Eigen::Matrix3f GetOrientationMatrix();
    
};
#endif
