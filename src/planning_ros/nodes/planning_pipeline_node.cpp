#include "planning_ros/planning_pipeline_node.hpp"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_pipeline");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string mission;
  pnh.param<std::string>("mission", mission, "high_speed");

  planning_ros::PlanningPipelineNode node(nh, mission);

  int rate = 20;
  pnh.param("loop_rate", rate, 20);
  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    ros::spinOnce();
    if (node.SpinOnce())
    {
      ros::Duration(2.0).sleep();
      ros::shutdown();
      break;
    }
    loop_rate.sleep();
  }

  return 0;
}
