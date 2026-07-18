#ifndef COVARIANCES_ON_IMU_HPP
#define COVARIANCES_ON_IMU_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>

namespace covariances_on_imu
{
class CovariancesOnImu : public rclcpp::Node
{
public:
  explicit CovariancesOnImu(const rclcpp::NodeOptions &options);

private:
  void handleImu(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  double ignore_variance_{1e6};
  double yaw_stddev_{0.005};
  double gyro_z_stddev_{0.0025};
};
} // namespace covariances_on_imu

#endif // COVARIANCES_ON_IMU_HPP
