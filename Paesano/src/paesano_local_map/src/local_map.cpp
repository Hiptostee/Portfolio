#include "paesano_local_map/local_map.hpp"

#include <algorithm>
#include <cmath>

namespace paesano_local_map
{

LocalMap::LocalMap(double size_m, double resolution_m, double obstacle_radius_m)
: size_m_(size_m),
  resolution_m_(resolution_m),
  inflate_cells_(static_cast<int>(std::ceil(obstacle_radius_m / resolution_m))),
  side_cells_(static_cast<int>(std::ceil(size_m / resolution_m))),
  grid_(static_cast<std::size_t>(side_cells_ * side_cells_), false)
{
}

void LocalMap::updateFromScan(
  const sensor_msgs::msg::LaserScan & scan,
  const geometry_msgs::msg::TransformStamped & laser_to_map)
{
  // Extract 2D rotation from quaternion — valid for a planar robot
  const auto & q = laser_to_map.transform.rotation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double tx = laser_to_map.transform.translation.x;
  const double ty = laser_to_map.transform.translation.y;

  std::lock_guard<std::mutex> lock(mutex_);

  // Recentre the grid on the robot's current map position each scan
  origin_x_ = tx - size_m_ * 0.5;
  origin_y_ = ty - size_m_ * 0.5;

  std::fill(grid_.begin(), grid_.end(), false);

  const float angle_min = scan.angle_min;
  const float angle_inc = scan.angle_increment;

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float r = scan.ranges[i];
    if (r < scan.range_min || r > scan.range_max || !std::isfinite(r)) {
      continue;
    }

    const float angle = angle_min + static_cast<float>(i) * angle_inc;
    const double lx = static_cast<double>(r) * std::cos(angle);
    const double ly = static_cast<double>(r) * std::sin(angle);

    // Transform endpoint from laser frame to map frame
    const double wx = cos_yaw * lx - sin_yaw * ly + tx;
    const double wy = sin_yaw * lx + cos_yaw * ly + ty;

    int cx, cy;
    worldToCell(wx, wy, cx, cy);

    // Circular inflation around the hit cell
    for (int dy = -inflate_cells_; dy <= inflate_cells_; ++dy) {
      for (int dx = -inflate_cells_; dx <= inflate_cells_; ++dx) {
        if (dx * dx + dy * dy > inflate_cells_ * inflate_cells_) {
          continue;
        }
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (inBounds(nx, ny)) {
          grid_[static_cast<std::size_t>(ny * side_cells_ + nx)] = true;
        }
      }
    }
  }
}

bool LocalMap::isPathBlocked(
  const nav_msgs::msg::Path & path,
  std::size_t from_index,
  int lookahead_n) const
{
  if (path.poses.empty() || lookahead_n <= 0) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  const std::size_t end = std::min(
    path.poses.size(),
    from_index + static_cast<std::size_t>(lookahead_n));

  for (std::size_t i = from_index; i < end; ++i) {
    int cx, cy;
    worldToCell(
      path.poses[i].pose.position.x,
      path.poses[i].pose.position.y,
      cx, cy);
    if (!inBounds(cx, cy)) {
      return true;  // waypoint outside local map window — treat as unknown/blocked
    }
    if (grid_[static_cast<std::size_t>(cy * side_cells_ + cx)]) {
      return true;
    }
  }
  return false;
}

void LocalMap::worldToCell(double wx, double wy, int & cx, int & cy) const
{
  cx = static_cast<int>(std::floor((wx - origin_x_) / resolution_m_));
  cy = static_cast<int>(std::floor((wy - origin_y_) / resolution_m_));
}

bool LocalMap::inBounds(int cx, int cy) const
{
  return cx >= 0 && cy >= 0 && cx < side_cells_ && cy < side_cells_;
}

}  // namespace paesano_local_map
