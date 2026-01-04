#ifndef MULTI_SENSOR_FUSION_IMU_SUBSCRIBER_HPP_
#define MULTI_SENSOR_FUSION_IMU_SUBSCRIBER_HPP_

#include <imu_data.hpp>
#include <common_msgs/ins_p2.h>
#include <Eigen/Dense>
class IMUSubscriber{
public:
      IMUSubscriber(ros::NodeHandle nh,std::string topic_name, size_t buff_size);
      IMUSubscriber() = default;
      bool buff_mutex_;
     
    
      void ParseData(std::deque<IMUData>& imu_data_buff);
      void SetInsInfo(common_msgs::ins_p2ConstPtr imu_msg_ptr);
      bool SyncData(std::deque<IMUData>& UnsyncedData, IMUData& synced_data,double sync_time);
      bool readAndSync(std::deque<IMUData>& unsynced_imu_, IMUData& synced_data, double sync_time);
  private:
  	ros::NodeHandle nh_;
  	ros::Subscriber subscriber_;
    // boost::shared_ptr<IMUSubscriber> imusub_; 
  	std::deque<IMUData> imu_data_buff_;
    std::deque<IMUData> new_imu_data_;
    IMUData pre_imu_data;
    std::mutex mtx; 
    void msg_callback(const common_msgs::ins_p2ConstPtr imu_msg_ptr);
    // 把四元数转换成旋转矩阵送出去
    // void ParseData(std::deque<IMUData>& imu_data_buff);
      Eigen::Matrix3f GetOrientationMatrix();
    
};
#endif