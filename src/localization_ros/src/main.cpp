#include <localization_ros/location_node.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "location_kdtree");
  ros::NodeHandle nh;

  localization_ros::LocationNode node(nh);

  ros::Rate rate(50);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
