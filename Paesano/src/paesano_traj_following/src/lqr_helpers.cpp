#include "paesano_traj_following/lqr.hpp"
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

  for (int i = 0; i < dare_max_iterations_; i++) {
    K = (R + B.transpose() * P * B).inverse() * (B.transpose() * P * A);
    P_next = A.transpose() * P * (A - B * K) + Q;
    if ((P_next - P).norm() < dare_convergence_tolerance_) break;
    P = P_next;
  }
  return K;
}

// The error matrix between the current and target poses.
Vector3d LQR::calculateError(const Pose & current_pose, const Pose & target_pose)
{
  const double cos_th = std::cos(current_pose.theta);
  const double sin_th = std::sin(current_pose.theta);
  return calculateError(current_pose, target_pose, cos_th, sin_th);
}

Vector3d LQR::calculateError(
  const Pose & current_pose,
  const Pose & target_pose,
  double cos_th,
  double sin_th)
{
  const double dx = target_pose.x - current_pose.x;
  const double dy = target_pose.y - current_pose.y;
  const double dtheta = wrapAngle(target_pose.theta - current_pose.theta);

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
    const double dx = path[i].x - robot_pose.x;
    const double dy = path[i].y - robot_pose.y;
    const double dist = dx * dx + dy * dy;
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

bool LQR::worldToLocalMapCell(double world_x, double world_y, int & cell_x, int & cell_y) const
{
  if (!have_local_map_ || latest_local_map_.data.empty()) {
    return false;
  }

  const auto & origin = latest_local_map_.info.origin;
  const double resolution = latest_local_map_.info.resolution;
  if (!(resolution > 0.0)) {
    return false;
  }

  const double origin_yaw = tf2::getYaw(origin.orientation);
  const double cos_yaw = std::cos(origin_yaw);
  const double sin_yaw = std::sin(origin_yaw);
  const double dx = world_x - origin.position.x;
  const double dy = world_y - origin.position.y;

  const double map_x = dx * cos_yaw + dy * sin_yaw;
  const double map_y = -dx * sin_yaw + dy * cos_yaw;
  cell_x = static_cast<int>(std::floor(map_x / resolution));
  cell_y = static_cast<int>(std::floor(map_y / resolution));

  return cell_x >= 0 &&
    cell_y >= 0 &&
    cell_x < static_cast<int>(latest_local_map_.info.width) &&
    cell_y < static_cast<int>(latest_local_map_.info.height);
}

bool LQR::hasOccupiedCellNear(double world_x, double world_y) const
{
  int center_x;
  int center_y;
  if (!worldToLocalMapCell(world_x, world_y, center_x, center_y)) {
    return false;
  }

  const double resolution = latest_local_map_.info.resolution;
  const int radius_cells =
    std::max(0, static_cast<int>(std::ceil(local_map_path_corridor_radius_ / resolution)));
  const int width = static_cast<int>(latest_local_map_.info.width);
  const int height = static_cast<int>(latest_local_map_.info.height);

  for (int y = center_y - radius_cells; y <= center_y + radius_cells; ++y) {
    if (y < 0 || y >= height) {
      continue;
    }
    for (int x = center_x - radius_cells; x <= center_x + radius_cells; ++x) {
      if (x < 0 || x >= width) {
        continue;
      }

      const double dx = static_cast<double>(x - center_x) * resolution;
      const double dy = static_cast<double>(y - center_y) * resolution;
      if (std::hypot(dx, dy) > local_map_path_corridor_radius_) {
        continue;
      }

      const std::size_t index = static_cast<std::size_t>(y * width + x);
      if (latest_local_map_.data[index] >= local_map_occupied_threshold_) {
        return true;
      }
    }
  }

  return false;
}

bool LQR::isPathBlockedByLocalMap(const Pose & current_pose, std::size_t target_idx) const
{
  if (!have_local_map_ || latest_local_map_.data.empty() || current_path_.empty()) {
    return false;
  }

  double remaining_distance = local_map_obstacle_check_distance_;
  Pose sample_start = current_pose;
  std::size_t path_idx = std::min(target_idx, current_path_.size() - 1);

  while (remaining_distance > 0.0 && path_idx < current_path_.size()) {
    const Pose & sample_end = current_path_[path_idx];
    const double dx = sample_end.x - sample_start.x;
    const double dy = sample_end.y - sample_start.y;
    const double segment_length = std::hypot(dx, dy);

    if (segment_length <= 1e-6) {
      sample_start = sample_end;
      ++path_idx;
      continue;
    }

    const double segment_to_check = std::min(segment_length, remaining_distance);
    const int steps =
      std::max(1, static_cast<int>(std::ceil(segment_to_check / local_map_path_sample_step_)));
    for (int step = 1; step <= steps; ++step) {
      const double distance_along =
        std::min(segment_to_check, static_cast<double>(step) * local_map_path_sample_step_);
      const double t = distance_along / segment_length;
      const double sample_x = sample_start.x + dx * t;
      const double sample_y = sample_start.y + dy * t;

      if (hasOccupiedCellNear(sample_x, sample_y)) {
        return true;
      }
    }

    remaining_distance -= segment_to_check;
    sample_start = sample_end;
    ++path_idx;
  }

  return false;
}

} // namespace lqr
