#include "odom_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>      // std::sin, std::cos, std::abs, std::hypot, std::isfinite
#include <algorithm>  // std::clamp

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

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
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

  // Motion accumulation for covariance shaping
  static double accum_trans = 0.0; // meters traveled (recent history)
  static double accum_rot   = 0.0; // radians rotated (recent history)

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

  // ============================================================
  // SIMPLE, STABLE COVARIANCES (light motion-accumulated)
  // Base: 2cm (x), 5cm (y). Inflates slowly with recent motion.
  // ============================================================
  const double vxf = std::abs(vx_body);
  const double vyf = std::abs(vy_body);
  const double wzf = std::abs(omega);

  // "Recent motion" memory with exponential decay
  // Larger tau = longer memory but smoother; 4–8s feels good.
  const double tau   = 6.0;                    // seconds
  const double decay = std::exp(-dt / tau);

  const double dtrans = std::hypot(vx_body, vy_body) * dt; // meters traveled this step
  const double drot   = wzf * dt;                          // radians rotated this step

  accum_trans = decay * accum_trans + dtrans;
  accum_rot   = decay * accum_rot   + drot;

  // --------- BASE FLOORS (your request) ----------
  const double sx_floor = 0.02;  // 2 cm
  const double sy_floor = 0.05;  // 5 cm
  const double syaw_floor = 0.03; // rad (~2.9 deg) -- wheels' yaw uncertainty

  // --------- VERY LIGHT INFLATION (tunable) ------
  // distance term: small growth with recent distance
  // rotation term: small growth with recent turning (mecanum scrub)
  // strafe term: small instantaneous penalty on y only
  double sx = sx_floor
            + 0.015 * accum_trans   // ~+1.5 cm per 1 m of *recent* travel
            + 0.010 * accum_rot;    // ~+1.0 cm per 1 rad of *recent* rotation

  double sy = sy_floor
            + 0.030 * accum_trans   // y grows ~2x x (strafe worse)
            + 0.020 * accum_rot
            + 0.020 * vyf;          // instantaneous strafe penalty (m/s -> m)

  // yaw from wheels should be bounded (IMU owns yaw in your EKF)
  double syaw = syaw_floor
              + 0.025 * accum_rot;  // grows with turning, lightly

  // --------- CLAMPS (keep it sane) ---------------
  sx   = std::clamp(sx,   0.02, 0.12);  // x σ max 12 cm
  sy   = std::clamp(sy,   0.05, 0.20);  // y σ max 20 cm
  syaw = std::clamp(syaw, 0.05, 0.40);  // yaw σ max ~34 deg (wheel yaw)

  // Unused axes huge
  const double IGN = 1e6;

  // Pose covariance (6x6 row-major: x y z roll pitch yaw)
  for (double &c : odom_msg.pose.covariance) c = 0.0;
  odom_msg.pose.covariance[0]  = sx*sx;
  odom_msg.pose.covariance[7]  = sy*sy;
  odom_msg.pose.covariance[14] = IGN;
  odom_msg.pose.covariance[21] = IGN;
  odom_msg.pose.covariance[28] = IGN;
  odom_msg.pose.covariance[35] = syaw*syaw;

  // --------- Twist covariance (simple + stable) --
  // Keep moderate, not crazy small, so EKF doesn't get overconfident
  double svx = 0.05 + 0.10 * vxf + 0.10 * vyf;
  double svy = 0.07 + 0.13 * vyf;
  double swz = 0.04 + 0.075* wzf;

  svx = std::clamp(svx, 0.05, 0.80);
  svy = std::clamp(svy, 0.07, 1.20);
  swz = std::clamp(swz, 0.04, 1.50);

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