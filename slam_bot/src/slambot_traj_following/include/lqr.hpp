#ifndef SLAMBOT_TRAJ_FOLLOWING__LQR_HPP_
#define SLAMBOT_TRAJ_FOLLOWING__LQR_HPP_

#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "lqr_helpers.hpp"

namespace lqr
{

class LQR : public rclcpp::Node
{
public:
  explicit LQR(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  Matrix3d solveDare(Matrix3d A, Matrix3d B, Matrix3d Q, Matrix3d R);
  Vector3d calculateError(const Pose & current_pose, const Pose & target_pose);
  double wrapAngle(double a) const;
  int findClosestIndex(const Pose & robot_pose, const std::vector<Pose> & path, int last_index) const;
  Pose getCurrentEstimatedPose() const;
  Pose getNextPathPose() const;

private:
  void lqrLoop();
  void estimatedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  Pose poseStampedToPose(const geometry_msgs::msg::PoseStamped & pose_stamped) const;
  void publishZeroVelocity();
  Eigen::Matrix3d K;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped current_estimated_pose_msg_;
  nav_msgs::msg::Path current_path_msg_;
  std::vector<Pose> current_path_;
  std::size_t current_path_index_{0};
  bool have_estimated_pose_{false};
  bool have_path_{false};
  bool initialized_{false};

};

}  // namespace lqr

#endif  // SLAMBOT_TRAJ_FOLLOWING__LQR_HPP_
