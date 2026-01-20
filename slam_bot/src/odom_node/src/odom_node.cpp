#include "odom_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace odom_node
{
OdomNode::OdomNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("odom_node", options)
  {
    RCLCPP_INFO(get_logger(), "Odom Node Started");

    double wheel_radius = declare_parameter<double>("wheel_radius", 0.0485);
    double base_length = declare_parameter<double>("base_length", 0.21);  
    double base_width = declare_parameter<double>("base_width", 0.200);    

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

    //velocity in t/s
    double vFL = dFL / dt;
    double vFR = dFR / dt;
    double vBL = dBL / dt;
    double vBR = dBR / dt;

    //velocity in m/s
    double mFL = vFL * distance_per_tick;
    double mFR = vFR * distance_per_tick;
    double mBL = vBL * distance_per_tick;
    double mBR = vBR * distance_per_tick;

    //vx, vy, omega
    double vx = (mFL + mFR + mBL + mBR)  / 4.0;
    double vy = (-mFL + mFR + mBL - mBR)  / 4.0;
    double omega = (-mFL + mFR - mBL + mBR) / (4.0 * (L + W));

    //transform to world frame
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double body_vx = vx * cos_theta - vy * sin_theta;
    double body_vy = vx * sin_theta + vy * cos_theta;
    vx = body_vx;
    vy = body_vy;

    x += vx * dt;
    y += vy * dt;
    theta += omega * dt;

    //populate odom message

    nav_msgs::msg::Odometry odom_msg;

    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.angular.z = omega;

    odom_msg.pose.covariance[0] = 0.01;
    odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[35] = 0.02;

    odom_msg.twist.covariance[0] = 0.01;
    odom_msg.twist.covariance[7] = 0.01;
    odom_msg.twist.covariance[35] = 0.02;

    odom_pub_->publish(odom_msg);
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(odom_node::OdomNode)
