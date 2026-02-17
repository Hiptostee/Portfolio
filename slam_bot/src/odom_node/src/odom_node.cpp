#include "odom_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>      // std::sin, std::cos, std::isfinite

namespace odom_node
{
OdomNode::OdomNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("odom_node", options)
{
  RCLCPP_INFO(get_logger(), "Odom Node Started");

  double wheel_radius = declare_parameter<double>("wheel_radius", 0.0485);
  double base_length  = declare_parameter<double>("base_length", 0.21);
  double base_width   = declare_parameter<double>("base_width", 0.200);

  L = base_length / 2;
  W = base_width / 2;
  R = wheel_radius;

  double ticks_per_rev = declare_parameter<double>("distance_per_tick", 2882.0);
  distance_per_tick = (2 * 3.141592653589793 * R) / ticks_per_rev;

  sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/wheel_encoders", 10,
      std::bind(&OdomNode::odomCallback, this, std::placeholders::_1));

  // Publish raw wheel odometry on /odom so localization uses the same
  // topic name in both simulation and hardware.
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
}

OdomNode::~OdomNode(){}

void OdomNode::odomCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  int32_t encFL = msg->data[0];
  int32_t encFR = msg->data[1];
  int32_t encBL = msg->data[2];
  int32_t encBR = msg->data[3];

  static bool init=false;
  static int32_t lastFL=0,lastFR=0,lastBL=0,lastBR=0;
  static auto last_time = now();
  static double x=0.0, y=0.0, theta=0.0;

  if (!init){
    lastFL=encFL; lastFR=encFR; lastBL=encBL; lastBR=encBR;
    init=true;
    last_time = now();
    return;
  }

  int32_t dFL = encFL - lastFL;
  int32_t dFR = encFR - lastFR;
  int32_t dBL = encBL - lastBL;
  int32_t dBR = encBR - lastBR;

  lastFL=encFL; lastFR=encFR; lastBL=encBL; lastBR=encBR;

  auto current_time = now();
  double dt = (current_time - last_time).seconds();
  last_time = current_time;

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
  double mFL = vFL * distance_per_tick;
  double mFR = vFR * distance_per_tick;
  double mBL = vBL * distance_per_tick;
  double mBR = vBR * distance_per_tick;

  // vx, vy, omega in body frame (base_link)
  double vx_body = (mFL + mFR + mBL + mBR) / 4.0;
  double vy_body = (-mFL + mFR + mBL - mBR) / 4.0;
  double omega   = (-mFL + mFR - mBL + mBR) / (4.0 * (L + W));

  // integrate pose in odom/world
  double cos_t = std::cos(theta);
  double sin_t = std::sin(theta);

  double vx_world = vx_body * cos_t - vy_body * sin_t;
  double vy_world = vx_body * sin_t + vy_body * cos_t;

  x     += vx_world * dt;
  y     += vy_world * dt;
  theta += omega * dt;

  // populate odom message
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = std::sin(theta / 2.0);
  odom_msg.pose.pose.orientation.w = std::cos(theta / 2.0);

  // publish twist in base_link frame
  odom_msg.twist.twist.linear.x  = vx_body;
  odom_msg.twist.twist.linear.y  = vy_body;
  odom_msg.twist.twist.angular.z = omega;

  // Fixed covariances tuned for mecanum on medium-slip flooring.
  // Lateral and yaw are less certain than forward due to scrub/slip.
  const double sx   = 0.06;  // pose x sigma [m]
  const double sy   = 0.10;  // pose y sigma [m]
  const double syaw = 0.12;  // pose yaw sigma [rad]
  const double svx  = 0.12;  // twist vx sigma [m/s]
  const double svy  = 0.18;  // twist vy sigma [m/s]
  const double swz  = 0.20;  // twist wz sigma [rad/s]

  // Unused axes: very large variance so EKF ignores them.
  const double IGN = 1e6;

  // Pose covariance (6x6 row-major: x y z roll pitch yaw)
  for (double &c : odom_msg.pose.covariance) c = 0.0;
  odom_msg.pose.covariance[0]  = sx*sx;
  odom_msg.pose.covariance[7]  = sy*sy;
  odom_msg.pose.covariance[14] = IGN;
  odom_msg.pose.covariance[21] = IGN;
  odom_msg.pose.covariance[28] = IGN;
  odom_msg.pose.covariance[35] = syaw*syaw;

  for (double &c : odom_msg.twist.covariance) c = 0.0;
  odom_msg.twist.covariance[0]  = svx*svx;
  odom_msg.twist.covariance[7]  = svy*svy;
  odom_msg.twist.covariance[14] = IGN;
  odom_msg.twist.covariance[21] = IGN;
  odom_msg.twist.covariance[28] = IGN;
  odom_msg.twist.covariance[35] = swz*swz;

  odom_pub_->publish(odom_msg);
}
}

RCLCPP_COMPONENTS_REGISTER_NODE(odom_node::OdomNode)
