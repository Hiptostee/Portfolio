#pragma once

#include <mutex>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace paesano_local_map
{

// Lightweight dynamic local costmap built from live LiDAR scans.
// Thread-safe: updateFromScan() and isPathBlocked() may be called concurrently.
class LocalMap
{
public:
  LocalMap(double size_m, double resolution_m, double obstacle_radius_m);

  // Rebuild the grid from one scan. laser_to_map is the transform from the
  // laser frame to the map frame at the time of the scan.
  void updateFromScan(
    const sensor_msgs::msg::LaserScan & scan,
    const geometry_msgs::msg::TransformStamped & laser_to_map);

  // Return true if any of the next lookahead_n waypoints (starting at from_index)
  // falls inside an occupied (or inflated) cell.
  bool isPathBlocked(
    const nav_msgs::msg::Path & path,
    std::size_t from_index,
    int lookahead_n) const;

private:
  const double size_m_;
  const double resolution_m_;
  const int inflate_cells_;
  const int side_cells_;

  std::vector<bool> grid_;  // row-major; true = occupied

  double origin_x_ = 0.0;  // map-frame bottom-left corner of the grid
  double origin_y_ = 0.0;

  mutable std::mutex mutex_;

  void worldToCell(double wx, double wy, int & cx, int & cy) const;
  bool inBounds(int cx, int cy) const;
};

}  // namespace paesano_local_map
