#include "planning_ros/line_detection_node.hpp"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  planning_ros::LineDetectionNode node(nh);

  int rate = 10;
  pnh.param("loop_rate", rate, 10);
  ros::Rate loop_rate(rate);
  ROS_INFO("[LineDetection] Node started, running at %d Hz", rate);

  while (ros::ok())
  {
    ros::spinOnce();
    node.RunOnce();

    if (node.IsFinished())
    {
      ROS_INFO("[LineDetection] Path planning completed, shutting down");
      ros::shutdown();
      break;
    }

    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
