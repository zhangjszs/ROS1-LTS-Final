#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <XmlRpcValue.h>

namespace autodrive_msgs {

/**
 * @brief Shared parameter loading utilities.
 *
 * Provides consistent parameter loading with warn-on-default behavior
 * and XmlRpcValue vector parsing. Replaces duplicated LoadParam/LoadVector
 * implementations in perception_ros, localization_ros, and others.
 */

/// Load a parameter from a single NodeHandle with debug logging on default.
template <typename T>
bool LoadParam(ros::NodeHandle &nh, const std::string &key, T &out, const T &default_value)
{
  if (nh.param<T>(key, out, default_value))
  {
    return true;
  }
  ROS_DEBUG_STREAM("[param] Using default for " << key << ": " << default_value);
  return false;
}

/// Load a parameter with private-then-global fallback (localization pattern).
template <typename T>
bool LoadParam(ros::NodeHandle &pnh, ros::NodeHandle &nh,
               const std::string &key, T &out, const T &default_value)
{
  if (pnh.param<T>(key, out, default_value))
  {
    return true;
  }
  if (nh.param<T>(key, out, default_value))
  {
    return true;
  }
  return false;
}

/// Load a vector parameter from XmlRpcValue array.
template <typename T>
bool LoadVector(ros::NodeHandle &nh, const std::string &key, std::vector<T> &vec)
{
  XmlRpc::XmlRpcValue list;
  if (!nh.getParam(key, list))
  {
    ROS_ERROR_STREAM("[param] Failed to load parameter: " << key);
    return false;
  }
  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("[param] Parameter " << key << " is not an array");
    return false;
  }

  vec.clear();
  vec.reserve(list.size());

  for (size_t i = 0; i < list.size(); ++i)
  {
    if constexpr (std::is_same_v<T, int>)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR_STREAM("[param] Element " << i << " in " << key << " is not an int");
        return false;
      }
      vec.push_back(static_cast<int>(list[i]));
    }
    else if constexpr (std::is_same_v<T, double>)
    {
      if (list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      {
        vec.push_back(static_cast<double>(list[i]));
      }
      else if (list[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        vec.push_back(static_cast<double>(static_cast<int>(list[i])));
      }
      else
      {
        ROS_ERROR_STREAM("[param] Element " << i << " in " << key << " is not a number");
        return false;
      }
    }
  }
  return true;
}

}  // namespace autodrive_msgs