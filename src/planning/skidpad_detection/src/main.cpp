#include "skidpad_detection.hpp"

using namespace fsac;

/**
 * @brief Main function for the skidpad_detection node.
 * 
 * Initializes the ROS node, creates a Skidpad_detection object, and runs the algorithm in a loop.
 * 
 * @param argc Number of command line arguments.
 * @param argv Command line arguments.
 * @return int Exit code.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "skidpad_detection"); // Initialize the ROS node with the given arguments and name.
	ros::NodeHandle nh("~"); // Create a NodeHandle for the current node with the given namespace.
	std::shared_ptr<Skidpad_detection> skid_ = std::make_shared<Skidpad_detection>(nh); // Create a shared pointer to a Skidpad_detection object with the given NodeHandle.
	ros::Rate loop_rate(10); // Create a Rate object with the given frequency.
	while (ros::ok()) // Loop while the ROS node is running.
	{
		ros::spinOnce(); // Process any incoming ROS messages.
		skid_->runAlgorithm(); // Run the skidpad detection algorithm.
		loop_rate.sleep(); // Sleep for the remaining time to maintain the desired frequency.
	}
	ros::spin(); // Spin the ROS node to process any remaining messages.
	return 0; // Return 0 to indicate successful execution.
}
