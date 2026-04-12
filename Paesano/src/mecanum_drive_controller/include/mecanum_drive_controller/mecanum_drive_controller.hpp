// Header for mecanum_drive_controller node
#pragma once

#include <cstdint>
#include <cstddef>

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
    bool i2cWriteU8(uint8_t reg, const uint8_t *data, size_t len);
    bool i2cWriteI8(uint8_t reg, const int8_t *data, size_t len);
    void sendPidToPico();

    double kp_{0.0}, ki_{0.0}, kd_{0.0};
    double kp_hold_{0.0}, ki_hold_{0.0}, kd_hold_{0.0};
    double wheel_radius_{0.0485};
    double base_length_{0.212};
    double base_width_{0.194};
    double ticks_per_rev_{2882.0};
    double max_ticks_per_sec_{3500.0};
    double max_linear_speed_mps_{0.35};
    double max_angular_speed_radps_{1.5};
    double distance_per_tick_{0.0};
    double mecanum_radius_{0.0};

    int32_t encFL = 0, encFR = 0, encBL = 0, encBR = 0;

    int i2c_file_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr encoder_timer_;

    std::string i2c_device_{"/dev/i2c-1"};
    int i2c_address_{0x12};
    int encoder_publish_period_ms_{50};
    std::string cmd_vel_topic_{"/cmd_vel"};
    std::string encoder_topic_{"/wheel_encoders"};
  };
} // namespace mecanum_drive_controller
