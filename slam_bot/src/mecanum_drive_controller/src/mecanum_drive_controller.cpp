#include "mecanum_drive_controller.hpp"
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

#define MOTOR_I2C_ADDR 0x12

#define REG_MOTOR_SPEEDS 51 // write 4x int8: FL,FR,BL,BR (canonical)
#define REG_ENCODERS 60     // read 16 bytes: 4x int32 LE: FL,FR,BL,BR (physical)
#define REG_PID_GAINS 70    // write 6 bytes: kp_i16,ki_i16,kd_i16 (LE)
#define REG_TARGET_POS 72
#define REG_TELEM_FL 73

namespace mecanum_drive_controller
{
  static inline int16_t clamp_i16(int v)
  {
    if (v > 32767)
      return 32767;
    if (v < -32768)
      return -32768;
    return (int16_t)v;
  }

  static inline int16_t le_i16(const uint8_t *p)
  {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
  }

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
    // wheel_radius_ = declare_parameter<double>("wheel_radius", 0.0485);
    // base_length_ = declare_parameter<double>("base_length", 0.297);
    // base_width_ = declare_parameter<double>("base_width", 0.256);

    kp_ = declare_parameter<double>("kp", 0.20);
    ki_ = declare_parameter<double>("ki", 0.00005);
    kd_ = declare_parameter<double>("kd", 0.0);

    i2c_file_ = open("/dev/i2c-1", O_RDWR);
    if (i2c_file_ < 0)
      RCLCPP_FATAL(get_logger(), "Failed to open /dev/i2c-1");

    if (ioctl(i2c_file_, I2C_SLAVE, MOTOR_I2C_ADDR) < 0)
      RCLCPP_FATAL(get_logger(), "Failed to set I2C addr 0x12");

    sendPidToPico();

    telem_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MecanumDriveController::readTelemFL, this));

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&MecanumDriveController::cmdVelCallback, this, std::placeholders::_1));

    pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/wheel_encoders", 10);
  }

  MecanumDriveController::~MecanumDriveController()
  {
    if (i2c_file_ >= 0)
      close(i2c_file_);
  }

  bool MecanumDriveController::i2cReadArray(uint8_t reg, uint8_t *data, size_t len)
  {
    if (write(i2c_file_, &reg, 1) != 1)
      return false;
    if (read(i2c_file_, data, len) != (ssize_t)len)
      return false;
    return true;
  }

  void MecanumDriveController::readTelemFL()
  {
    uint8_t raw[16];
    if (!i2cReadArray(REG_TELEM_FL, raw, sizeof(raw)))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "FL telemetry read failed");
      return;
    }

    int32_t v_tgt = le_i32(&raw[0]);
    int32_t v_meas = le_i32(&raw[4]);
    int32_t pwm_base = le_i32(&raw[8]);
    int32_t pwm_out = le_i32(&raw[12]);

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 200,
        "[FL_TELEM] vtgt=%d vmeas=%d pwm_base=%d pwm_out=%d",
        v_tgt, v_meas, pwm_base, pwm_out);
  }

  bool MecanumDriveController::i2cWriteI8(uint8_t reg, const int8_t *data, size_t len)
  {
    std::vector<uint8_t> buf(len + 1);
    buf[0] = reg;
    std::memcpy(&buf[1], data, len);
    return (write(i2c_file_, buf.data(), buf.size()) == (ssize_t)buf.size());
  }

  bool MecanumDriveController::i2cWriteU8(uint8_t reg, const uint8_t *data, size_t len)
  {
    std::vector<uint8_t> buf(len + 1);
    buf[0] = reg;
    std::memcpy(&buf[1], data, len);
    return (write(i2c_file_, buf.data(), buf.size()) == (ssize_t)buf.size());
  }

  void MecanumDriveController::sendPidToPico()
  {

    int16_t kp_q = clamp_i16((int)std::lround(kp_ * 1000.0));
    int16_t ki_q = clamp_i16((int)std::lround(ki_ * 100000.0));
    int16_t kd_q = clamp_i16((int)std::lround(kd_ * 10.0));

    uint8_t gains[6];
    auto pack16 = [&](int idx, int16_t v)
    {
      gains[idx + 0] = (uint8_t)(v & 0xFF);
      gains[idx + 1] = (uint8_t)((v >> 8) & 0xFF);
    };
    pack16(0, kp_q);
    pack16(2, ki_q);
    pack16(4, kd_q);

    if (!i2cWriteU8(REG_PID_GAINS, gains, 6))
      RCLCPP_ERROR(get_logger(), "Failed to write PID gains");

    RCLCPP_WARN(get_logger(),
                "Sent PID to Pico: kp=%.6f ki=%.8f kd=%.6f (q: %d %d %d)",
                kp_, ki_, kd_, (int)kp_q, (int)ki_q, (int)kd_q);
  }

  void MecanumDriveController::sendTargetPosToPico()
  {
    uint8_t buf[1 + 16];
    buf[0] = REG_TARGET_POS;
    int32_t vals[4] = {encFL, encFR, encBL, encBR};
    for (int i = 0; i < 4; i++)
    {
      buf[1 + i * 4 + 0] = (uint8_t)(vals[i] & 0xFF);
      buf[1 + i * 4 + 1] = (uint8_t)((vals[i] >> 8) & 0xFF);
      buf[1 + i * 4 + 2] = (uint8_t)((vals[i] >> 16) & 0xFF);
      buf[1 + i * 4 + 3] = (uint8_t)((vals[i] >> 24) & 0xFF);
    }
    if (write(i2c_file_, buf, sizeof(buf)) != (ssize_t)sizeof(buf))
      RCLCPP_ERROR(get_logger(), "Failed to write target positions");
  }

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

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "Encoders: FL=%d FR=%d BL=%d BR=%d",
                         encFL, encFR, encBL, encBR);
  }

  void MecanumDriveController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {

    double y = msg->linear.x;
    double x = msg->linear.y;
    double rx = msg->angular.z;

    double fl = y + x + rx;
    double fr = y - x - rx;
    double bl = y - x + rx;
    double br = y + x - rx;

    double maxVal = std::max({std::abs(fl), std::abs(fr), std::abs(bl), std::abs(br), 1.0});
    fl /= maxVal;
    fr /= maxVal;
    bl /= maxVal;
    br /= maxVal;

    auto to_i8 = [](double u) -> int8_t
    {
      u = std::max(-1.0, std::min(1.0, u));
      int v = (int)std::lround(u * 127.0);
      v = std::max(-127, std::min(127, v));
      return (int8_t)v;
    };

    int8_t speeds[4] = {to_i8(fl), to_i8(fr), to_i8(bl), to_i8(br)};

    if (!i2cWriteI8(REG_MOTOR_SPEEDS, speeds, 4))
      RCLCPP_ERROR(get_logger(), "I2C motor write failed");

    if (speeds[0] == 0.0 && speeds[1] == 0.0 && speeds[2] == 0.0 && speeds[3] == 0.0)
    {
      sendTargetPosToPico();
    }
  }

} // namespace mecanum_drive_controller

RCLCPP_COMPONENTS_REGISTER_NODE(mecanum_drive_controller::MecanumDriveController)