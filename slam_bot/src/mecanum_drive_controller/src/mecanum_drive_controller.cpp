#include "mecanum_drive_controller.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

// C++ STL headers used in this file
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <chrono>
#include <vector>

#define MOTOR_I2C_ADDR 0x34
#define MOTOR_FIXED_SPEED_ADDR 51 // fixed speed closed-loop control

namespace mecanum_drive_controller
{

MecanumDriveController::MecanumDriveController(const rclcpp::NodeOptions &options)
    : rclcpp::Node("mecanum_drive_controller", options)
{
  // Correct parameter declarations
  wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0485);
  base_length_ = declare_parameter<double>("base_length", 0.297);
  base_width_ = declare_parameter<double>("base_width", 0.256);

  // I2C init
  i2c_file_ = open("/dev/i2c-1", O_RDWR);
  if (i2c_file_ < 0)
    RCLCPP_FATAL(get_logger(), "Failed to open /dev/i2c-1");

  if (ioctl(i2c_file_, I2C_SLAVE, MOTOR_I2C_ADDR) < 0)
    RCLCPP_FATAL(get_logger(), "Failed to set I2C address to 0x34");

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MecanumDriveController::readEncoders, this));

  // ROS interfaces
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&MecanumDriveController::cmdVelCallback, this, std::placeholders::_1));
  pub = create_publisher<std_msgs::msg::Int32MultiArray>("/wheel_encoders", 10);
}

MecanumDriveController::~MecanumDriveController()
{
  if (i2c_file_ >= 0)
    close(i2c_file_);
}

bool MecanumDriveController::i2cReadArray(uint8_t reg, uint8_t *data, size_t len)
{
  // select register
  if (write(i2c_file_, &reg, 1) != 1)
    return false;

  // read bytes
  if (read(i2c_file_, data, len) != (ssize_t)len)
    return false;

  return true;
}

bool MecanumDriveController::i2cWriteArray(uint8_t reg, int8_t *data, size_t len)
{
  uint8_t buffer[len + 1];
  buffer[0] = reg;
  memcpy(&buffer[1], data, len);

  if (write(i2c_file_, buffer, len + 1) != (ssize_t)(len + 1))
    return false;

  return true;
}

void MecanumDriveController::readEncoders()
{
  uint8_t raw[16]; // 4 motors * 4 bytes each

  if (!i2cReadArray(60, raw, 16))
  {
    RCLCPP_ERROR(get_logger(), "Failed to read encoder data");
    return;
  }

  int32_t encFL = (raw[0] | raw[1] << 8 | raw[2] << 16 | raw[3] << 24);
  int32_t encFR = (raw[4] | raw[5] << 8 | raw[6] << 16 | raw[7] << 24);
  int32_t encBL = (raw[8] | raw[9] << 8 | raw[10] << 16 | raw[11] << 24);
  int32_t encBR = (raw[12] | raw[13] << 8 | raw[14] << 16 | raw[15] << 24);

  std_msgs::msg::Int32MultiArray msg;

  msg.data = {encFL, encFR, encBL, encBR};
  pub->publish(msg);

  RCLCPP_INFO(get_logger(),
              "Encoders: FL=%d FR=%d BL=%d BR=%d",
              encFL, encFR, encBL, encBR);
}

void MecanumDriveController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto clamp = [](double w) -> int8_t
  {
    if (w > 127)
      return 127;
    if (w < -127)
      return -127;
    return static_cast<int8_t>(w);
  };

  double y = msg->linear.x;   // forward
  double x = msg->linear.y;   // strafe
  double rx = msg->angular.z; // rotate

  // FTC normalizes so max magnitude = 1.0
  double fl = y + x + rx;
  double bl = y - x - rx;
  double fr = y - x + rx;
  double br = y + x - rx;

  // optional: normalize power so no wheel exceeds 1.0
  double maxVal =
      std::max({std::abs(fl), std::abs(bl), std::abs(fr), std::abs(br), 1.0});

  fl /= maxVal;
  bl /= -maxVal;
  fr /= -maxVal;
  br /= maxVal;

  double speed_gain = 80.0; // scale to int8_t (-127..127)

  int8_t speeds[4] = {
      clamp(fl * speed_gain),
      clamp(fr * speed_gain),
      clamp(bl * speed_gain),
      clamp(br * speed_gain)};

  if (!i2cWriteArray(MOTOR_FIXED_SPEED_ADDR, speeds, 4))
  {
    RCLCPP_ERROR(get_logger(), "I2C motor write failed");
  }
}

} // namespace mecanum_drive_controller

RCLCPP_COMPONENTS_REGISTER_NODE(mecanum_drive_controller::MecanumDriveController);
