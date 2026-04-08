#include "slambot_traj_following/lqr.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace lqr
{

// Method to solve the Algebraic Riccati Equation.
Matrix3d LQR::solveDare(Matrix3d A, Matrix3d B, Matrix3d Q, Matrix3d R)
{
  Matrix3d P = Q;
  Matrix3d P_next;
  Matrix3d K;

  for (int i = 0; i < 1000; i++) {
    K = (R + B.transpose() * P * B).inverse() * (B.transpose() * P * A);
    P_next = A.transpose() * P * (A - B * K) + Q;
    if ((P_next - P).norm() < 1e-6) break;
    P = P_next;
  }
  return K;
}

// The error matrix between the current and target poses.
Vector3d LQR::calculateError(const Pose & current_pose, const Pose & target_pose)
{
  const double dx = target_pose.x - current_pose.x;
  const double dy = target_pose.y - current_pose.y;
  const double dtheta = wrapAngle(target_pose.theta - current_pose.theta);

  const double cos_th = std::cos(current_pose.theta);
  const double sin_th = std::sin(current_pose.theta);

  Vector3d local_error;
  local_error << dx * cos_th + dy * sin_th,
                 -dx * sin_th + dy * cos_th,
                 dtheta;
  return local_error;
}

// Helper method to make sure the angle stays within (0, 360).
double LQR::wrapAngle(double a) const
{
  return std::atan2(std::sin(a), std::cos(a));
}

// Method to find the closest index to the robot pose.
std::size_t LQR::findClosestIndex(
  const Pose & robot_pose,
  const std::vector<Pose> & path,
  std::size_t last_index) const
{
  double min_dist = std::numeric_limits<double>::max();
  std::size_t closest_idx = last_index;

  // Search window logic: If last_index is 0, we search the WHOLE path once.
  // Otherwise, we search 50 points ahead to stay efficient on the RPi 5.
  const std::size_t search_start = last_index;
  const std::size_t search_end =
    (last_index == 0) ? path.size() : std::min(path.size(), last_index + static_cast<std::size_t>(50));

  for (std::size_t i = search_start; i < search_end; ++i) {
    const double dist = std::pow(path[i].x - robot_pose.x, 2) + std::pow(path[i].y - robot_pose.y, 2);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}

// Convert a pose stamped to a pose.
Pose LQR::poseStampedToPose(const geometry_msgs::msg::PoseStamped & pose_stamped) const
{
  const auto & position = pose_stamped.pose.position;
  const double theta = tf2::getYaw(pose_stamped.pose.orientation);
  return Pose{position.x, position.y, theta};
}

} // namespace lqr
