#pragma once

// Redirect to shared param utilities in autodrive_msgs.
// perception_ros::LoadParam and perception_ros::LoadVector are now
// aliases for autodrive_msgs::LoadParam and autodrive_msgs::LoadVector.

#include <autodrive_msgs/param_utils.hpp>

namespace perception_ros {

using autodrive_msgs::LoadParam;
using autodrive_msgs::LoadVector;

}  // namespace perception_ros
