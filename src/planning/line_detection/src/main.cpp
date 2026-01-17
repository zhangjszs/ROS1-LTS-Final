#include "line_detection/line_detection.hpp"
#include <ros/ros.h>

using namespace fsac;

/**
 * @brief Main function for the line_detection node.
 *
 * Initializes the ROS node, creates a LineDetection object, and runs the
 * algorithm in a loop. This node performs straight line detection using
 * Hough transform and generates a path for the acceleration event.
 *
 * @param argc Number of command line arguments.
 * @param argv Command line arguments.
 * @return int Exit code.
 */
int main(int argc, char **argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "line_detection");

  // Create a NodeHandle for the current node
  ros::NodeHandle nh("~");

  // Create a shared pointer to a LineDetection object
  std::shared_ptr<LineDetection> line_detector =
      std::make_shared<LineDetection>(nh);

  // Create a Rate object with 10 Hz
  ros::Rate loop_rate(10);

  ROS_INFO("[LineDetection] Node started, running at 10 Hz");

  // Main loop
  while (ros::ok())
  {
    // Process any incoming ROS messages
    ros::spinOnce();

    // Run the line detection and path planning algorithm
    line_detector->runAlgorithm();

    // Check if finished
    if (line_detector->isFinished())
    {
      ROS_INFO("[LineDetection] Path planning completed, shutting down");
      ros::shutdown();
      break;
    }

    // Sleep for the remaining time to maintain the desired frequency
    loop_rate.sleep();
  }

  // Spin the ROS node to process any remaining messages
  ros::spin();

  return 0;
}
