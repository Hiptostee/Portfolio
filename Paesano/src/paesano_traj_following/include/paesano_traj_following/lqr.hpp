#ifndef PAESANO_TRAJ_FOLLOWING__LQR_HPP_
#define PAESANO_TRAJ_FOLLOWING__LQR_HPP_

#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "paesano_traj_following/lqr_helpers.hpp"

namespace lqr
{

class LQR : public rclcpp::Node
{
public:
  explicit LQR(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  Matrix3d solveDare(Matrix3d A, Matrix3d B, Matrix3d Q, Matrix3d R);
  Vector3d calculateError(const Pose & current_pose, const Pose & target_pose);
  double wrapAngle(double a) const;
  std::size_t findClosestIndex(
    const Pose & robot_pose,
    const std::vector<Pose> & path,
    std::size_t last_index) const;

private:
  void lqrLoop();
  void estimatedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void handleStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  Pose poseStampedToPose(const geometry_msgs::msg::PoseStamped & pose_stamped) const;
  void publishZeroVelocity();
  void stopTracking(const char * reason);
  Eigen::Matrix3d K;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped current_estimated_pose_msg_;
  std::vector<Pose> current_path_;
  std::size_t current_path_index_{0};
  bool have_estimated_pose_{false};
  bool have_path_{false};
  bool initialized_{false};
  int control_period_ms_{20};
  int lookahead_points_{1};
  double max_linear_velocity_{0.6};
  double max_angular_velocity_{0.6};
  double path_complete_tolerance_{0.08};
  double q_x_{50.0};
  double q_y_{75.0};
  double q_theta_{5.0};
  double r_weight_{0.5};

};

}  // namespace lqr

#endif  // PAESANO_TRAJ_FOLLOWING__LQR_HPP_
