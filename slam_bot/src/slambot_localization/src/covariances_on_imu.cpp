#include "covariances_on_imu.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include "rclcpp_components/register_node_macro.hpp"

namespace covariances_on_imu
{

CovariancesOnImu::CovariancesOnImu(const rclcpp::NodeOptions &options)
    : rclcpp::Node("covariances_on_imu", options)
{
  const auto imu_input_topic  = declare_parameter<std::string>("imu_input_topic",  "/imu");
  const auto imu_output_topic = declare_parameter<std::string>("imu_output_topic", "/imu_with_covariances");

  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      imu_output_topic,
      rclcpp::SystemDefaultsQoS());

  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input_topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&CovariancesOnImu::handleImu, this, std::placeholders::_1));
}

static inline void fill_cov(double cov[9], double value)
{
  for (int i = 0; i < 9; ++i) cov[i] = value;
}

void CovariancesOnImu::handleImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!imu_publisher_) return;

  sensor_msgs::msg::Imu out = *msg;

  // -----------------------------
  // Tune these as needed
  // -----------------------------
  // "Ignore" variance (very large => EKF effectively ignores that axis)
  constexpr double VAR_IGNORE = 1e6;

  // Yaw: don't make it too tiny if you don't have a magnetometer (drift is real).
  // Start around ~3-5 degrees stddev.
  constexpr double yaw_std = 0.07;                // rad (~4 degrees)
  constexpr double var_yaw = yaw_std * yaw_std;   // ~0.0049

  // Gyro z: typical indoor bot; start around 0.02 rad/s stddev
  constexpr double gyroz_std = 0.02;                 // rad/s
  constexpr double var_gyroz = gyroz_std * gyroz_std; // 0.0004

  // -----------------------------
  // ORIENTATION covariance (roll, pitch ignored; yaw used)
  // -----------------------------
  fill_cov(out.orientation_covariance.data(), 0.0);
  out.orientation_covariance[0] = VAR_IGNORE; // roll
  out.orientation_covariance[4] = VAR_IGNORE; // pitch
  out.orientation_covariance[8] = var_yaw;    // yaw

  // -----------------------------
  // ANGULAR VELOCITY covariance (wx, wy ignored; wz used)
  // -----------------------------
  fill_cov(out.angular_velocity_covariance.data(), 0.0);
  out.angular_velocity_covariance[0] = VAR_IGNORE; // wx
  out.angular_velocity_covariance[4] = VAR_IGNORE; // wy
  out.angular_velocity_covariance[8] = var_gyroz;  // wz

  // -----------------------------
  // LINEAR ACCELERATION covariance (ignore all)
  // -----------------------------
  fill_cov(out.linear_acceleration_covariance.data(), 0.0);
  out.linear_acceleration_covariance[0] = VAR_IGNORE; // ax
  out.linear_acceleration_covariance[4] = VAR_IGNORE; // ay
  out.linear_acceleration_covariance[8] = VAR_IGNORE; // az (ignore for now)

  // Optional: sanity check against NaN/Inf (prevents EKF blowups)
  auto finite9 = [](const std::array<double, 9> &c) {
    for (double v : c) if (!std::isfinite(v)) return false;
    return true;
  };
  if (!finite9(out.orientation_covariance) ||
      !finite9(out.angular_velocity_covariance) ||
      !finite9(out.linear_acceleration_covariance))
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "IMU covariance contained non-finite values; resetting to safe ignore defaults.");
    for (auto &c : out.orientation_covariance) c = 0.0;
    for (auto &c : out.angular_velocity_covariance) c = 0.0;
    for (auto &c : out.linear_acceleration_covariance) c = 0.0;

    out.orientation_covariance[0] = VAR_IGNORE;
    out.orientation_covariance[4] = VAR_IGNORE;
    out.orientation_covariance[8] = var_yaw;

    out.angular_velocity_covariance[0] = VAR_IGNORE;
    out.angular_velocity_covariance[4] = VAR_IGNORE;
    out.angular_velocity_covariance[8] = var_gyroz;

    out.linear_acceleration_covariance[0] = VAR_IGNORE;
    out.linear_acceleration_covariance[4] = VAR_IGNORE;
    out.linear_acceleration_covariance[8] = VAR_IGNORE;
  }

  imu_publisher_->publish(out);
}

} // namespace covariances_on_imu

RCLCPP_COMPONENTS_REGISTER_NODE(covariances_on_imu::CovariancesOnImu)