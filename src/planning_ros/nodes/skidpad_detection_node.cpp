#include "planning_ros/skidpad_detection_node.hpp"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "skidpad_detection");
  ros::NodeHandle nh("~");

  planning_ros::SkidpadDetectionNode node(nh);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    node.RunOnce();
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
