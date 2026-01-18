#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace odom_node
{
  class OdomNode : public rclcpp::Node
  {
  public:
    explicit OdomNode(const rclcpp::NodeOptions &options);
    ~OdomNode();
  private:
    void odomCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    double x=0.0, y=0.0, theta=0.0;
    double distance_per_tick{};
    double L{};
    double W{};
    double R{};
  };
} // namespace odom_node