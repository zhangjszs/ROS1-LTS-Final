#include "planning_ros/skidpad_detection_node.hpp"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "skidpad_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  planning_ros::SkidpadDetectionNode node(nh);

  int rate = 10;
  pnh.param("loop_rate", rate, 10);
  ros::Rate loop_rate(rate);
  while (ros::ok())
  {
    ros::spinOnce();
    node.RunOnce();
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
