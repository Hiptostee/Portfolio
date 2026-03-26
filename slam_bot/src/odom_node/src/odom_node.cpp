#include "odom_node/odom_node.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <cmath>      // std::sin, std::cos, std::isfinite
#include <functional>

namespace odom_node
{
OdomNode::OdomNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("odom_node", options)
{
  RCLCPP_INFO(get_logger(), "Odom Node Started");

  wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0485);
  const double base_length = declare_parameter<double>("base_length", 0.212);
  const double base_width = declare_parameter<double>("base_width", 0.194);
  const double ticks_per_rev = declare_parameter<double>("ticks_per_rev", 2882.0);
  encoder_topic_ = declare_parameter<std::string>("encoder_topic", "/wheel_encoders");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

  half_length_ = base_length / 2.0;
  half_width_ = base_width / 2.0;
  distance_per_tick_ = (2.0 * 3.141592653589793 * wheel_radius_) / ticks_per_rev;

  sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    encoder_topic_,
    10,
    std::bind(&OdomNode::odomCallback, this, std::placeholders::_1));

  // Publish raw wheel odometry on /odom so localization uses the same
  // topic name in both simulation and hardware.
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 50);
}

void OdomNode::odomCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring encoder message with fewer than 4 values");
    return;
  }

  int32_t encFL = msg->data[0];
  int32_t encFR = msg->data[1];
  int32_t encBL = msg->data[2];
  int32_t encBR = msg->data[3];

  if (!initialized_) {
    last_fl_ = encFL;
    last_fr_ = encFR;
    last_bl_ = encBL;
    last_br_ = encBR;
    initialized_ = true;
    last_time_ = now();
    return;
  }

  int32_t dFL = encFL - last_fl_;
  int32_t dFR = encFR - last_fr_;
  int32_t dBL = encBL - last_bl_;
  int32_t dBR = encBR - last_br_;

  last_fl_ = encFL;
  last_fr_ = encFR;
  last_bl_ = encBL;
  last_br_ = encBR;

  const auto current_time = now();
  const double dt = (current_time - last_time_).seconds();
  last_time_ = current_time;

  // Guard dt
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    return;
  }

  // velocity in ticks/s
  double vFL = dFL / dt;
  double vFR = dFR / dt;
  double vBL = dBL / dt;
  double vBR = dBR / dt;

  // velocity in m/s at wheels
  double mFL = vFL * distance_per_tick_;
  double mFR = vFR * distance_per_tick_;
  double mBL = vBL * distance_per_tick_;
  double mBR = vBR * distance_per_tick_;

  // vx, vy, omega in body frame (base_link)
  double vx_body = (mFL + mFR + mBL + mBR) / 4.0;
  double vy_body = (-mFL + mFR + mBL - mBR) / 4.0;
  double omega   = (-mFL + mFR - mBL + mBR) / (4.0 * (half_length_ + half_width_));

  // integrate pose in odom/world
  double cos_t = std::cos(theta_);
  double sin_t = std::sin(theta_);

  double vx_world = vx_body * cos_t - vy_body * sin_t;
  double vy_world = vx_body * sin_t + vy_body * cos_t;

  x_ += vx_world * dt;
  y_ += vy_world * dt;
  theta_ += omega * dt;

  // populate odom message
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = std::sin(theta_ / 2.0);
  odom_msg.pose.pose.orientation.w = std::cos(theta_ / 2.0);

  // publish twist in base_link frame
  odom_msg.twist.twist.linear.x  = vx_body;
  odom_msg.twist.twist.linear.y  = vy_body;
  odom_msg.twist.twist.angular.z = omega;

  const double base   = 0.5;  
  const double mag = 10.0;
  const double pos_to_vel = 0.5;
  const double IGN = 1e6;

  // Pose covariance (6x6 row-major: x y z roll pitch yaw)
  for (double &c : odom_msg.pose.covariance) c = 0.0;
  odom_msg.pose.covariance[0]  = pos_to_vel * (base * base);
  odom_msg.pose.covariance[7]  = mag * pos_to_vel * (base * base);
  odom_msg.pose.covariance[14] = IGN;
  odom_msg.pose.covariance[21] = IGN;
  odom_msg.pose.covariance[28] = IGN;
  odom_msg.pose.covariance[35] = mag * mag * pos_to_vel * (base * base);

  for (double &c : odom_msg.twist.covariance) c = 0.0;
  odom_msg.twist.covariance[0]  = (base * base);
  odom_msg.twist.covariance[7]  = mag * (base * base);
  odom_msg.twist.covariance[14] = IGN;
  odom_msg.twist.covariance[21] = IGN;
  odom_msg.twist.covariance[28] = IGN;
  odom_msg.twist.covariance[35] = mag * mag * (base * base);

  odom_pub_->publish(odom_msg);
}
}

RCLCPP_COMPONENTS_REGISTER_NODE(odom_node::OdomNode)
