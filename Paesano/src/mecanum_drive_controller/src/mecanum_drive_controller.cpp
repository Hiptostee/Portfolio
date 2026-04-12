#include "mecanum_drive_controller/mecanum_drive_controller.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <chrono>
#include <vector>

#define REG_MOTOR_SPEEDS 51 // write 4x int8: FL,FR,BL,BR
#define REG_ENCODERS 60     // read 16 bytes: 4x int32 LE: FL,FR,BL,BR
#define REG_PID_GAINS 70    // write 12 bytes: kp,ki,kd,kp_hold,ki_hold,kd_hold (LE int16)

namespace mecanum_drive_controller
{
  // Clamp an integer to the range of a signed 16-bit value before packing.
  static inline int16_t clamp_i16(int v)
  {
    if (v > 32767)
      return 32767;
    if (v < -32768)
      return -32768;
    return (int16_t)v;
  }

  // Decode a little-endian 32-bit integer read back from the Pico.
  static inline int32_t le_i32(const uint8_t *p)
  {
    return (int32_t)((uint32_t)p[0] |
                     ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) |
                     ((uint32_t)p[3] << 24));
  }

  MecanumDriveController::MecanumDriveController(const rclcpp::NodeOptions &options)
      : rclcpp::Node("mecanum_drive_controller", options)
  {
    // Hardware and ROS interface parameters.
    i2c_device_ = declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
    i2c_address_ = declare_parameter<int>("i2c_address", 0x12);
    encoder_publish_period_ms_ = declare_parameter<int>("encoder_publish_period_ms", 50);
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    encoder_topic_ = declare_parameter<std::string>("encoder_topic", "/wheel_encoders");

    // Velocity PID gains used by the Pico motor controller.
    kp_ = declare_parameter<double>("kp", 0.5);
    ki_ = declare_parameter<double>("ki", 0.00001);
    kd_ = declare_parameter<double>("kd", 0.0);

    // Hold PID gains used when the commanded target is zero.
    kp_hold_ = declare_parameter<double>("kp_hold", 5.0);
    ki_hold_ = declare_parameter<double>("ki_hold", 0.0);
    kd_hold_ = declare_parameter<double>("kd_hold", 0.0);

    // Physical parameters used to interpret cmd_vel in SI units.
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0485);
    base_length_ = declare_parameter<double>("base_length", 0.212);
    base_width_ = declare_parameter<double>("base_width", 0.194);
    ticks_per_rev_ = declare_parameter<double>("ticks_per_rev", 2882.0);
    max_ticks_per_sec_ = declare_parameter<double>("max_ticks_per_sec", 3500.0);
    max_linear_speed_mps_ = declare_parameter<double>("max_linear_speed_mps", 0.35);
    max_angular_speed_radps_ = declare_parameter<double>("max_angular_speed_radps", 1.5);

    distance_per_tick_ =
      (2.0 * M_PI * wheel_radius_) / std::max(ticks_per_rev_, 1e-6);
    mecanum_radius_ = 0.5 * (base_length_ + base_width_);

    // Open the I2C bus and bind it to the Pico address.
    i2c_file_ = open(i2c_device_.c_str(), O_RDWR);
    if (i2c_file_ < 0)
      RCLCPP_FATAL(get_logger(), "Failed to open %s", i2c_device_.c_str());

    if (ioctl(i2c_file_, I2C_SLAVE, i2c_address_) < 0)
      RCLCPP_FATAL(get_logger(), "Failed to set I2C addr 0x%02x", i2c_address_);

    // Push the configured gains once at startup.
    sendPidToPico();

    // Periodically poll the wheel encoder registers from the Pico.
    encoder_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(encoder_publish_period_ms_),
        std::bind(&MecanumDriveController::readEncoders, this));

    // Subscribe to ROS2 velocity commands and publish encoder feedback.
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10,
        std::bind(&MecanumDriveController::cmdVelCallback, this, std::placeholders::_1));

    pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(encoder_topic_, 10);
  }

  // Close the I2C file descriptor when the node shuts down.
  MecanumDriveController::~MecanumDriveController()
  {
    if (i2c_file_ >= 0)
      close(i2c_file_);
  }

  // Read a raw byte block from a Pico register over I2C.
  bool MecanumDriveController::i2cReadArray(uint8_t reg, uint8_t *data, size_t len)
  {
    if (write(i2c_file_, &reg, 1) != 1)
      return false;
    if (read(i2c_file_, data, len) != (ssize_t)len)
      return false;
    return true;
  }

  // Write a signed byte block to a Pico register over I2C.
  bool MecanumDriveController::i2cWriteI8(uint8_t reg, const int8_t *data, size_t len)
  {
    std::vector<uint8_t> buf(len + 1);
    buf[0] = reg;
    std::memcpy(&buf[1], data, len);
    return (write(i2c_file_, buf.data(), buf.size()) == (ssize_t)buf.size());
  }

  // Write an unsigned byte block to a Pico register over I2C.
  bool MecanumDriveController::i2cWriteU8(uint8_t reg, const uint8_t *data, size_t len)
  {
    std::vector<uint8_t> buf(len + 1);
    buf[0] = reg;
    std::memcpy(&buf[1], data, len);
    return (write(i2c_file_, buf.data(), buf.size()) == (ssize_t)buf.size());
  }

  // Pack the configured PID gains and send them to the Pico controller.
  void MecanumDriveController::sendPidToPico()
  {
    int16_t kp_q = clamp_i16((int)std::lround(kp_ * 1000.0));
    int16_t ki_q = clamp_i16((int)std::lround(ki_ * 100000.0));
    int16_t kd_q = clamp_i16((int)std::lround(kd_ * 10.0));
    int16_t kp_q_hold = clamp_i16((int)std::lround(kp_hold_ * 1000.0));
    int16_t ki_q_hold = clamp_i16((int)std::lround(ki_hold_ * 100000.0));
    int16_t kd_q_hold = clamp_i16((int)std::lround(kd_hold_ * 10.0));

    uint8_t gains[12];
    auto pack16 = [&](int idx, int16_t v)
    {
      gains[idx + 0] = (uint8_t)(v & 0xFF);
      gains[idx + 1] = (uint8_t)((v >> 8) & 0xFF);
    };

    pack16(0, kp_q);
    pack16(2, ki_q);
    pack16(4, kd_q);
    pack16(6, kp_q_hold);
    pack16(8, ki_q_hold);
    pack16(10, kd_q_hold);

    if (!i2cWriteU8(REG_PID_GAINS, gains, 12))
      RCLCPP_ERROR(get_logger(), "Failed to write PID gains");

    RCLCPP_WARN(get_logger(),
                "Sent PID to Pico: kp=%.6f ki=%.8f kd=%.6f | kp_hold=%.6f ki_hold=%.8f kd_hold=%.6f",
                kp_, ki_, kd_, kp_hold_, ki_hold_, kd_hold_);
  }

  // Read all four wheel encoders from the Pico and publish them to ROS.
  void MecanumDriveController::readEncoders()
  {
    uint8_t raw[16];
    if (!i2cReadArray(REG_ENCODERS, raw, sizeof(raw)))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Encoder read failed");
      return;
    }

    encFL = le_i32(&raw[0]);
    encFR = le_i32(&raw[4]);
    encBL = le_i32(&raw[8]);
    encBR = le_i32(&raw[12]);

    std_msgs::msg::Int32MultiArray msg;
    msg.data = {encFL, encFR, encBL, encBR};
    pub_->publish(msg);
  }

  // Convert a ROS cmd_vel command in body-frame SI units into wheel tick-rate commands
  // for the Pico controller.
  void MecanumDriveController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double omega = msg->angular.z;

    const double planar_speed = std::hypot(vx, vy);
    if (planar_speed > max_linear_speed_mps_ && planar_speed > 1e-9) {
      const double scale = max_linear_speed_mps_ / planar_speed;
      vx *= scale;
      vy *= scale;
    }
    omega = std::clamp(omega, -max_angular_speed_radps_, max_angular_speed_radps_);

    // Wheel rim linear velocities in m/s.
    double fl_mps = vx - vy - mecanum_radius_ * omega;
    double fr_mps = vx + vy + mecanum_radius_ * omega;
    double bl_mps = vx + vy - mecanum_radius_ * omega;
    double br_mps = vx - vy + mecanum_radius_ * omega;

    // Convert wheel rim linear velocities into wheel encoder tick rates.
    const double ticks_per_meter = 1.0 / std::max(distance_per_tick_, 1e-9);
    double fl_ticks = fl_mps * ticks_per_meter;
    double fr_ticks = fr_mps * ticks_per_meter;
    double bl_ticks = bl_mps * ticks_per_meter;
    double br_ticks = br_mps * ticks_per_meter;

    // Preserve the requested motion direction while respecting the Pico's maximum
    // representable wheel target.
    const double max_abs_ticks = std::max({
      std::abs(fl_ticks), std::abs(fr_ticks), std::abs(bl_ticks), std::abs(br_ticks), 1.0
    });
    const double scale =
      (max_abs_ticks > max_ticks_per_sec_) ? (max_ticks_per_sec_ / max_abs_ticks) : 1.0;
    fl_ticks *= scale;
    fr_ticks *= scale;
    bl_ticks *= scale;
    br_ticks *= scale;

    auto to_i8 = [](double u) -> int8_t
    {
      u = std::max(-1.0, std::min(1.0, u));
      int v = (int)std::lround(u * 127.0);
      v = std::max(-127, std::min(127, v));
      return (int8_t)v;
    };

    const double inv_max_ticks =
      1.0 / std::max(max_ticks_per_sec_, 1e-6);
    int8_t speeds[4] = {
      to_i8(fl_ticks * inv_max_ticks),
      to_i8(fr_ticks * inv_max_ticks),
      to_i8(bl_ticks * inv_max_ticks),
      to_i8(br_ticks * inv_max_ticks)
    };

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "cmd_vel vx=%.3f vy=%.3f wz=%.3f | wheel_mps FL=%.3f FR=%.3f BL=%.3f BR=%.3f | "
      "wheel_ticks FL=%.1f FR=%.1f BL=%.1f BR=%.1f | i2c_cmd FL=%d FR=%d BL=%d BR=%d",
      vx, vy, omega,
      fl_mps, fr_mps, bl_mps, br_mps,
      fl_ticks, fr_ticks, bl_ticks, br_ticks,
      static_cast<int>(speeds[0]),
      static_cast<int>(speeds[1]),
      static_cast<int>(speeds[2]),
      static_cast<int>(speeds[3]));

    if (!i2cWriteI8(REG_MOTOR_SPEEDS, speeds, 4))
      RCLCPP_ERROR(get_logger(), "I2C motor write failed");
  }

} // namespace mecanum_drive_controller

RCLCPP_COMPONENTS_REGISTER_NODE(mecanum_drive_controller::MecanumDriveController)
