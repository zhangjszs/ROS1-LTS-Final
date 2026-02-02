#pragma once

#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <vector>
#include <string>

namespace perception_ros {

template<typename T>
bool LoadParam(ros::NodeHandle& nh, const std::string& name, T& value, const T& default_value) {
  if (!nh.param<T>(name, value, default_value)) {
    ROS_WARN_STREAM("Did not load " << name << ". Using default: " << default_value);
    return false;
  }
  return true;
}

template<typename T>
bool LoadVector(ros::NodeHandle& nh, const std::string& name, std::vector<T>& vec) {
  XmlRpc::XmlRpcValue list;
  if (!nh.getParam(name, list)) {
    ROS_ERROR_STREAM("Failed to load parameter: " << name);
    return false;
  }

  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM("Parameter " << name << " is not an array");
    return false;
  }

  vec.clear();
  vec.reserve(list.size());

  for (size_t i = 0; i < list.size(); ++i) {
    if constexpr (std::is_same_v<T, int>) {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
        ROS_ERROR_STREAM("Element " << i << " in " << name << " is not an int");
        return false;
      }
      vec.push_back(static_cast<int>(list[i]));
    } else if constexpr (std::is_same_v<T, double>) {
      if (list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        vec.push_back(static_cast<double>(list[i]));
      } else if (list[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        vec.push_back(static_cast<double>(static_cast<int>(list[i])));
      } else {
        ROS_ERROR_STREAM("Element " << i << " in " << name << " is not a number");
        return false;
      }
    }
  }

  return true;
}

}  // namespace perception_ros
