#include <ros/ros.h>

#include <perception_ros/lidar_cluster_ros.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_cluster_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  perception_ros::LidarClusterRos lc(nh, private_nh);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    lc.RunOnce();
    loop_rate.sleep();
  }
  return 0;
}
