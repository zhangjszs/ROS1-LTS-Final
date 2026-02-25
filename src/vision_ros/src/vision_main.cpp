#include <ros/ros.h>
#include "vision_ros/vision_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  vision_ros::VisionNode node(nh, pnh);
  node.spin();
  return 0;
}
