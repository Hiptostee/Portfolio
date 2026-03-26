#pragma once

#include <cstdint>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace odom_node
{
class OdomNode : public rclcpp::Node
{
public:
  explicit OdomNode(const rclcpp::NodeOptions & options);

private:
  void odomCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  bool initialized_{false};
  int32_t last_fl_{0};
  int32_t last_fr_{0};
  int32_t last_bl_{0};
  int32_t last_br_{0};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
  double distance_per_tick_{0.0};
  double half_length_{0.0};
  double half_width_{0.0};
  double wheel_radius_{0.0};

  std::string encoder_topic_{"/wheel_encoders"};
  std::string odom_topic_{"/odom"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
};
} // namespace odom_node
