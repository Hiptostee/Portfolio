#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class MecanumDriveController : public rclcpp::Node
{
public:
  MecanumDriveController()
  : Node("mecanum_drive_controller")
  {
    // Parameters (meters, radians). Defaults match slambot.xacro
    wheel_radius_ = declare_parameter("wheel_radius", 0.0485);
    base_length_  = declare_parameter("base_length", 0.297);  // x dimension (front-back)
    base_width_   = declare_parameter("base_width", 0.256);   // y dimension (left-right)
    wheel_signs_  = declare_parameter<std::vector<double>>("wheel_signs", {1.0, 1.0, 1.0, 1.0});

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&MecanumDriveController::cmdVelCallback, this, std::placeholders::_1)
    );

    wheel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/wheel_controller/commands",
      10
    );
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double wz = msg->angular.z;

    // Half distances from robot center to wheels
    const double L = base_length_ / 2.0;
    const double W = base_width_  / 2.0;
    const double R = wheel_radius_;

    // Standard mecanum inverse kinematics (x forward, y left, z yaw)
    // Order: front_left, front_right, back_left, back_right
    double fl = (vx - vy - (L + W) * wz) / R;
    double fr = (vx + vy + (L + W) * wz) / R;
    double bl = (vx + vy - (L + W) * wz) / R;
    double br = (vx - vy + (L + W) * wz) / R;

    std_msgs::msg::Float64MultiArray out;
    // Apply optional per-wheel sign corrections [fl, fr, bl, br]
    if (wheel_signs_.size() == 4) {
      out.data = {fl * wheel_signs_[0],
                  fr * wheel_signs_[1],
                  bl * wheel_signs_[2],
                  br * wheel_signs_[3]};
    } else {
      out.data = {fl, fr, bl, br};
    }

    wheel_pub_->publish(out);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
  double wheel_radius_;
  double base_length_;
  double base_width_;
  std::vector<double> wheel_signs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MecanumDriveController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
