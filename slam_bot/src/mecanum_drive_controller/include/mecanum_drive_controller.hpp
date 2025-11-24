// Header for mecanum_drive_controller node
#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace mecanum_drive_controller
{
class MecanumDriveController : public rclcpp::Node
{
public:
  explicit MecanumDriveController(const rclcpp::NodeOptions &options);
  ~MecanumDriveController();

private:
  // Periodic read of encoders and publish
  void readEncoders();

  // Handle incoming velocity commands
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Low-level I2C helpers
  bool i2cReadArray(uint8_t reg, uint8_t *data, size_t len);
  bool i2cWriteArray(uint8_t reg, int8_t *data, size_t len);

  // Members
  int i2c_file_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  double wheel_radius_{};
  double base_length_{};
  double base_width_{};
  std::vector<double> wheel_signs_;
};
} // namespace mecanum_drive_controller

