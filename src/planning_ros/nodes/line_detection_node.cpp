#include "planning_ros/line_detection_node.hpp"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_detection");
  ros::NodeHandle nh("~");

  planning_ros::LineDetectionNode node(nh);

  ros::Rate loop_rate(10);
  ROS_INFO("[LineDetection] Node started, running at 10 Hz");

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
