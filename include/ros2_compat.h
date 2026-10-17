/*
ROS 2 compatibility helpers for FAST-Calib2.

Keeps the ROS 1 style logging macros used throughout the algorithm code and
provides parameter getters that accept an int where a double is expected (a
flat YAML file writes `cx: 960` without a decimal point).
*/

#ifndef ROS2_COMPAT_H
#define ROS2_COMPAT_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cmath>
#include <string>

using CloudPub = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

namespace fast_calib_compat
{
inline rclcpp::Logger logger() { return rclcpp::get_logger("fast_calib"); }

// Node that accepts every parameter given on the command line / launch file.
inline rclcpp::Node::SharedPtr makeNode(const std::string& name)
{
  return std::make_shared<rclcpp::Node>(
      name, rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true));
}

inline bool readParam(const rclcpp::Node::SharedPtr& node, const std::string& name,
                      rclcpp::Parameter& p)
{
  if (!node->has_parameter(name)) return false;
  p = node->get_parameter(name);
  return p.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET;
}

inline bool getParam(const rclcpp::Node::SharedPtr& node, const std::string& name, double& out)
{
  rclcpp::Parameter p;
  if (!readParam(node, name, p)) return false;
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) out = p.as_double();
  else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) out = static_cast<double>(p.as_int());
  else
  {
    RCLCPP_WARN(logger(), "[Config] Parameter '%s' is not numeric; using default.", name.c_str());
    return false;
  }
  return true;
}

inline bool getParam(const rclcpp::Node::SharedPtr& node, const std::string& name, int& out)
{
  double value;
  if (!getParam(node, name, value)) return false;
  out = static_cast<int>(std::lround(value));
  return true;
}

inline bool getParam(const rclcpp::Node::SharedPtr& node, const std::string& name, bool& out)
{
  rclcpp::Parameter p;
  if (!readParam(node, name, p)) return false;
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
  {
    RCLCPP_WARN(logger(), "[Config] Parameter '%s' is not a bool; using default.", name.c_str());
    return false;
  }
  out = p.as_bool();
  return true;
}

inline bool getParam(const rclcpp::Node::SharedPtr& node, const std::string& name, std::string& out)
{
  rclcpp::Parameter p;
  if (!readParam(node, name, p)) return false;
  out = p.value_to_string();
  return true;
}

template <typename T, typename D>
inline void param(const rclcpp::Node::SharedPtr& node, const std::string& name, T& out,
                  const D& default_value)
{
  if (!getParam(node, name, out)) out = default_value;
}
}  // namespace fast_calib_compat

#define ROS_INFO(...) RCLCPP_INFO(fast_calib_compat::logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(fast_calib_compat::logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(fast_calib_compat::logger(), __VA_ARGS__)
#define ROS_INFO_STREAM(x) RCLCPP_INFO_STREAM(fast_calib_compat::logger(), x)
#define ROS_WARN_STREAM(x) RCLCPP_WARN_STREAM(fast_calib_compat::logger(), x)
#define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(fast_calib_compat::logger(), x)

#endif  // ROS2_COMPAT_H
