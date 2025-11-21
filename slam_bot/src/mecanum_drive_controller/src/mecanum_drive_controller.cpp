#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

#define MOTOR_I2C_ADDR 0x34
#define MOTOR_FIXED_SPEED_ADDR 51 // fixed speed closed-loop control

class MecanumDriveController : public rclcpp::Node
{
public:
  MecanumDriveController()
      : Node("mecanum_drive_controller")
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

    // ROS interfaces
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&MecanumDriveController::cmdVelCallback, this, std::placeholders::_1));
  }

  ~MecanumDriveController()
  {
    if (i2c_file_ >= 0)
      close(i2c_file_);
  }

private:
  // Send array of 4 speeds to motor driver
  bool i2cWriteArray(uint8_t reg, int8_t *data, size_t len)
  {
    uint8_t buffer[len + 1];
    buffer[0] = reg;
    memcpy(&buffer[1], data, len);

    if (write(i2c_file_, buffer, len + 1) != (ssize_t)(len + 1))
      return false;

    return true;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto clamp = [](double w) -> int8_t {
    if (w > 127) return 127;
    if (w < -127) return -127;
    return static_cast<int8_t>(w);
  };

  double y  = msg->linear.x;   // forward
  double x  = msg->linear.y;   // strafe
  double rx = msg->angular.z;  // rotate

  // FTC normalizes so max magnitude = 1.0
  double fl = y + x + rx;
  double bl = y - x - rx;
  double fr = y - x + rx;
  double br = y + x - rx;

  // optional: normalize power so no wheel exceeds 1.0
  double maxVal =
      std::max({ std::abs(fl), std::abs(bl), std::abs(fr), std::abs(br), 1.0 });

  fl /= maxVal;
  bl /= -maxVal;
  fr /= -maxVal;
  br /= maxVal;

  double speed_gain = 80.0;  // scale to int8_t (-127..127)

  int8_t speeds[4] = {
    clamp(fl * speed_gain),
    clamp(fr * speed_gain),
    clamp(bl * speed_gain),
    clamp(br * speed_gain)
  };

  if (!i2cWriteArray(MOTOR_FIXED_SPEED_ADDR, speeds, 4)) {
    RCLCPP_ERROR(get_logger(), "I2C motor write failed");
  }
}

  // Members
  int i2c_file_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  double wheel_radius_, base_length_, base_width_;
  std::vector<double> wheel_signs_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumDriveController>());
  rclcpp::shutdown();
  return 0;
}