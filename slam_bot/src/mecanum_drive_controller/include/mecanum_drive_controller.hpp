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
    bool i2cWriteU8(uint8_t reg, const uint8_t *data, size_t len);
    bool i2cWriteI8(uint8_t reg, const int8_t *data, size_t len);
    void sendPidToPico();
    void readPidDebug();
    void readTelemFL();
    void sendTargetPosToPico();

    double kp_{0.0}, ki_{0.0}, kd_{0.0};
    double kp_hold_{0.0}, ki_hold_{0.0}, kd_hold_{0.0};

    int32_t encFL = 0, encFR = 0, encBL = 0, encBR = 0;

    // Members
    int i2c_file_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr telem_timer_;
    rclcpp::TimerBase::SharedPtr encoder_timer_;

    double wheel_radius_{};
    double base_length_{};
    double base_width_{};
    std::vector<double> wheel_signs_;
  };
} // namespace mecanum_drive_controller