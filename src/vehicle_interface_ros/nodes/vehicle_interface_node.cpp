#include "App.h"

#include <csignal>
#include <iostream>

int main(int argc, char** argv) {
  Application app(argc, argv);

  // Let ROS handle SIGINT for clean shutdown (ros::ok() will return false on Ctrl-C)
  // Do NOT override with SIG_DFL as that breaks ros::isShuttingDown() detection.

  while (ros::ok()) {
    ros::Duration(0.1).sleep();
  }
  return 0;
}
