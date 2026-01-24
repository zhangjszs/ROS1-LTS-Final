#include <ros/ros.h>
#include <autodrive_msgs/HUAT_VehicleStatus.h>

#include "vehicle_racing_num_core/racing_num_writer.hpp"

class RacingNumWriterNode
{
public:
  RacingNumWriterNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  {
    pnh.param<std::string>("output_file", output_file_, vehicle_racing_num_core::DefaultOutputPath());
    pnh.param<std::string>("topic", topic_, "vehicleStatusMsg");
    pnh.param("queue_size", queue_size_, 1);
    pnh.param("exit_on_nonzero", exit_on_nonzero_, true);

    sub_ = nh.subscribe(topic_, queue_size_, &RacingNumWriterNode::handleStatus, this);
  }

private:
  void handleStatus(const autodrive_msgs::HUAT_VehicleStatus::ConstPtr &msg)
  {
    const int work_mode = static_cast<int>(msg->work_mode);
    const int racing_num = static_cast<int>(msg->racing_num);

    if (!vehicle_racing_num_core::WriteRacingNum(output_file_, racing_num))
    {
      ROS_ERROR_STREAM("Failed to write racing_num to " << output_file_);
      return;
    }

    ROS_INFO_STREAM("work_mode: " << work_mode << " racing_num: " << racing_num);

    if (exit_on_nonzero_ && racing_num != 0)
    {
      ROS_INFO("racing_num is non-zero, shutting down.");
      ros::shutdown();
    }
  }

  ros::Subscriber sub_;
  std::string output_file_;
  std::string topic_;
  int queue_size_ = 1;
  bool exit_on_nonzero_ = true;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vehicle_racing_num_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  RacingNumWriterNode node(nh, pnh);
  ros::spin();
  return 0;
}
