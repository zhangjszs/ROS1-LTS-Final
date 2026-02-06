#include <ros/ros.h>

#include <perception_ros/lidar_cluster_ros.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_cluster_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  perception_ros::LidarClusterRos lc(nh, private_nh);

  if (lc.IsLegacyPollMode())
  {
    ros::Rate loop_rate(lc.LegacyPollHz());
    while (ros::ok())
    {
      ros::spinOnce();
      lc.RunOnce();
      loop_rate.sleep();
    }
  }
  else
  {
    // 事件驱动：点云回调直接触发处理，避免固定频率轮询带来的等待
    ros::spin();
  }
  return 0;
}
