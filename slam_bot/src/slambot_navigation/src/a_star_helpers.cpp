#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "slambot_navigation/a_star_helpers.hpp"
#include "slambot_navigation/a_star.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace slambot_navigation
{

namespace
{

struct InternalPoint
{
  double x;
  double y;
  double dist;
};

}  // namespace

// Check that the cached occupancy grid has valid dimensions and data size.
bool AStarPlanner::isMapValid() const
{
  return map_.info.width > 0 && map_.info.height > 0 && map_.info.resolution > 0 &&
         map_.data.size() == map_.info.width * map_.info.height;
}

// Return whether a grid cell lies inside the map bounds.
bool AStarPlanner::isInBounds(const Coordinate & cell) const
{
  const int width = map_.info.width;
  const int height = map_.info.height;
  return cell.x >= 0 && cell.y >= 0 && cell.x < width && cell.y < height;
}

// Convert a valid grid cell into the flat index used by OccupancyGrid::data.
size_t AStarPlanner::toIndex(const Coordinate & cell) const
{
  if (!isInBounds(cell)) {
    throw std::out_of_range("Grid cell is out of bounds");
  }
  return cell.y * map_.info.width + cell.x;
}

// Reject cells that are occupied, unknown, or too close to nearby obstacles.
bool AStarPlanner::isCellTraversable(const Coordinate & cell) const
{
  if (!isInBounds(cell)) {
    throw std::out_of_range("Grid cell is out of bounds");
  }

  const double resolution = map_.info.resolution;
  const int buffer_cells = static_cast<int>(std::ceil(obstacle_buffer_m_ / resolution));

  for (int dy = -buffer_cells; dy <= buffer_cells; ++dy) {
    for (int dx = -buffer_cells; dx <= buffer_cells; ++dx) {
      const Coordinate neighbor{cell.x + dx, cell.y + dy};
      if (!isInBounds(neighbor)) {
        return false;
      }

      const double distance_m = std::sqrt(
        std::pow(static_cast<double>(dx) * resolution, 2) +
        std::pow(static_cast<double>(dy) * resolution, 2));
      if (distance_m > obstacle_buffer_m_) {
        continue;
      }

      const int value = map_.data[toIndex(neighbor)];
      if (value >= 50 || value == -1) {
        return false;
      }
    }
  }

  return true;
}

// String pulling to create a path of the longest possible straight lines
nav_msgs::msg::Path AStarPlanner::stringPull(const nav_msgs::msg::Path& path) const {
  if (path.poses.size() <= 2) {
    return path;
  }

  nav_msgs::msg::Path smoothed_path;
  smoothed_path.header = path.header;

  int current_idx = 0;
  smoothed_path.poses.push_back(path.poses[current_idx]);

  while (current_idx < static_cast<int>(path.poses.size()) - 1) {
    bool found_shortcut = false;

    for (int i = path.poses.size() - 1; i > current_idx; i--) {
      if (is_line_clear(path.poses[current_idx].pose, path.poses[i].pose, map_)) {
        smoothed_path.poses.push_back(path.poses[i]);
        current_idx = i; 
        found_shortcut = true;
        break; 
      }
    }

    if (!found_shortcut) {
      current_idx++;
      smoothed_path.poses.push_back(path.poses[current_idx]);
    }
  }

  return smoothed_path;
}

// Bresenham Line Algorithm + line search.
bool AStarPlanner::is_line_clear(const geometry_msgs::msg::Pose& start, 
                                 const geometry_msgs::msg::Pose& end, 
                                 const nav_msgs::msg::OccupancyGrid& map) const 
{
    int x0 = static_cast<int>((start.position.x - map.info.origin.position.x) / map.info.resolution);
    int y0 = static_cast<int>((start.position.y - map.info.origin.position.y) / map.info.resolution);
    int x1 = static_cast<int>((end.position.x - map.info.origin.position.x) / map.info.resolution);
    int y1 = static_cast<int>((end.position.y - map.info.origin.position.y) / map.info.resolution);

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1; 
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int radius = 4;

    while (true) {
        for (int i = -radius; i <= radius; ++i) {
            for (int j = -radius; j <= radius; ++j) {
                int nx = x0 + i;
                int ny = y0 + j;

                if (nx < 0 || nx >= (int)map.info.width || ny < 0 || ny >= (int)map.info.height) {
                    return false;
                }

                int index = ny * map.info.width + nx;
                if (map.data[index] > 50) {
                    return false;
                }
            }
        }

        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }

    return true;
}

// The equation for converting a point into a point as part of a Catmull-Rom spline.
double AStarPlanner::catmullRom(double p0, double p1, double p2, double p3, double t) const {
    double t2 = t * t;
    double t3 = t2 * t;

    // Standard Catmull-Rom matrix coefficients
    return 0.5 * (
        (2.0 * p1) +
        (-p0 + p2) * t +
        (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
        (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
    );
}

// Apply the spline to the path
nav_msgs::msg::Path AStarPlanner::applySpline(const nav_msgs::msg::Path& path) const {
    if (path.poses.size() < 2) return path;

    // --- STEP 1: Dense Sampling ---
    // We sample the spline very heavily to "measure" its true shape.
    std::vector<InternalPoint> dense_points;
    double total_dist = 0.0;

    for (size_t i = 0; i < path.poses.size() - 1; ++i) {
        auto p1 = path.poses[i].pose.position;
        auto p2 = path.poses[i+1].pose.position;
        auto p0 = (i == 0) ? p1 : path.poses[i-1].pose.position;
        auto p3 = (i + 2 < path.poses.size()) ? path.poses[i+2].pose.position : p2;

        const int samples_per_segment = 100; 
        for (int s = 0; s < samples_per_segment; ++s) {
            double t = static_cast<double>(s) / samples_per_segment;
            double cur_x = catmullRom(p0.x, p1.x, p2.x, p3.x, t);
            double cur_y = catmullRom(p0.y, p1.y, p2.y, p3.y, t);

            if (!dense_points.empty()) {
                double dx = cur_x - dense_points.back().x;
                double dy = cur_y - dense_points.back().y;
                total_dist += std::hypot(dx, dy);
            }
            dense_points.push_back({cur_x, cur_y, total_dist});
        }
    }
    // Add the final goal point explicitly
    double dx_final = path.poses.back().pose.position.x - dense_points.back().x;
    double dy_final = path.poses.back().pose.position.y - dense_points.back().y;
    total_dist += std::hypot(dx_final, dy_final);
    dense_points.push_back({path.poses.back().pose.position.x, path.poses.back().pose.position.y, total_dist});

    // --- STEP 2: Uniform Re-sampling ---
    // Now we walk along that "measured" path at exactly 5cm intervals.
    nav_msgs::msg::Path smooth_path;
    smooth_path.header = path.header;
    const double target_spacing = 0.05;
    size_t dense_idx = 0;

    for (double s = 0; s <= total_dist; s += target_spacing) {
        // Find the two points in dense_points that straddle distance 's'
        while (dense_idx < dense_points.size() - 2 && dense_points[dense_idx + 1].dist < s) {
            dense_idx++;
        }

        InternalPoint p_a = dense_points[dense_idx];
        InternalPoint p_b = dense_points[dense_idx + 1];

        // Linear interpolation between the two dense points
        double segment_dist = p_b.dist - p_a.dist;
        double interp_t = (segment_dist > 1e-6) ? (s - p_a.dist) / segment_dist : 0.0;

        geometry_msgs::msg::PoseStamped ps;
        ps.header = smooth_path.header;
        ps.pose.position.x = p_a.x + interp_t * (p_b.x - p_a.x);
        ps.pose.position.y = p_a.y + interp_t * (p_b.y - p_a.y);
        ps.pose.orientation.w = 1.0;
        
        smooth_path.poses.push_back(ps);
    }

    // Always ensure the very last pose is the exact goal
    smooth_path.poses.push_back(path.poses.back());

    for (size_t i = 0; i < smooth_path.poses.size(); ++i) {
      double yaw;
      if (i < smooth_path.poses.size() - 1) {
          double dx = smooth_path.poses[i+1].pose.position.x - smooth_path.poses[i].pose.position.x;
          double dy = smooth_path.poses[i+1].pose.position.y - smooth_path.poses[i].pose.position.y;
          yaw = std::atan2(dy, dx);
      } else {
          yaw = tf2::getYaw(smooth_path.poses[i-1].pose.orientation);
      }

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      smooth_path.poses[i].pose.orientation = tf2::toMsg(q);
  }

    return smooth_path;
}

// Convert a world-space position in meters into a map grid cell.
bool AStarPlanner::worldToGrid(double wx, double wy, Coordinate & cell) const
{
  if (!isMapValid()) {
    return false;
  }

  const double resolution = map_.info.resolution;
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;
  const double rel_x = wx - origin_x;
  const double rel_y = wy - origin_y;

  cell.x = static_cast<int>(std::floor(rel_x / resolution));
  cell.y = static_cast<int>(std::floor(rel_y / resolution));

  return isInBounds(cell);
}

// Convert a grid cell back to the center of that cell as a ROS pose.
geometry_msgs::msg::PoseStamped AStarPlanner::gridToWorldPose(
  const Coordinate & cell,
  const std_msgs::msg::Header & header) const
{
  const double resolution = map_.info.resolution;
  const double origin_x = map_.info.origin.position.x;
  const double origin_y = map_.info.origin.position.y;

  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  pose.pose.position.x = origin_x + (cell.x + 0.5) * resolution;
  pose.pose.position.y = origin_y + (cell.y + 0.5) * resolution;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

// Estimate straight-line distance between two grid cells for A*.
double AStarPlanner::heuristic(const Coordinate & a, const Coordinate & b) const
{
  return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

// Return the legal neighbor cells, including diagonals that do not cut corners.
std::vector<Coordinate> AStarPlanner::getNeighbors(const Coordinate & cell) const
{
  std::vector<Coordinate> neighbors;
  const Coordinate up{cell.x, cell.y + 1};
  const Coordinate down{cell.x, cell.y - 1};
  const Coordinate right{cell.x + 1, cell.y};
  const Coordinate left{cell.x - 1, cell.y};

  if (isInBounds(up) && isCellTraversable(up)) {
    neighbors.push_back(up);
  }
  if (isInBounds(down) && isCellTraversable(down)) {
    neighbors.push_back(down);
  }
  if (isInBounds(left) && isCellTraversable(left)) {
    neighbors.push_back(left);
  }
  if (isInBounds(right) && isCellTraversable(right)) {
    neighbors.push_back(right);
  }

  const Coordinate top_left{cell.x - 1, cell.y + 1};
  if (isInBounds(top_left) && isCellTraversable(top_left) &&
      isInBounds(left) && isCellTraversable(left) &&
      isInBounds(up) && isCellTraversable(up)) {
    neighbors.push_back(top_left);
  }

  const Coordinate top_right{cell.x + 1, cell.y + 1};
  if (isInBounds(top_right) && isCellTraversable(top_right) &&
      isInBounds(right) && isCellTraversable(right) &&
      isInBounds(up) && isCellTraversable(up)) {
    neighbors.push_back(top_right);
  }

  const Coordinate bottom_left{cell.x - 1, cell.y - 1};
  if (isInBounds(bottom_left) && isCellTraversable(bottom_left) &&
      isInBounds(left) && isCellTraversable(left) &&
      isInBounds(down) && isCellTraversable(down)) {
    neighbors.push_back(bottom_left);
  }

  const Coordinate bottom_right{cell.x + 1, cell.y - 1};
  if (isInBounds(bottom_right) && isCellTraversable(bottom_right) &&
      isInBounds(right) && isCellTraversable(right) &&
      isInBounds(down) && isCellTraversable(down)) {
    neighbors.push_back(bottom_right);
  }

  return neighbors;
}

// Walk backward through the parent links and rebuild the final cell path.
std::vector<Coordinate> AStarPlanner::reconstructPath(
  const Coordinate & start,
  const Coordinate & goal,
  const std::vector<int> & came_from) const
{
  if (!isMapValid() || !isInBounds(start) || !isInBounds(goal)) {
    return {};
  }

  const size_t width = map_.info.width;
  const size_t cell_count = static_cast<size_t>(map_.info.width) * static_cast<size_t>(map_.info.height);
  if (came_from.size() != cell_count) {
    return {};
  }

  std::vector<Coordinate> path;
  const int start_index = static_cast<int>(toIndex(start));
  int current_index = static_cast<int>(toIndex(goal));

  while (current_index != -1) {
    const size_t current = static_cast<size_t>(current_index);
    path.push_back(Coordinate{
      static_cast<int>(current % width),
      static_cast<int>(current / width)
    });

    if (current_index == start_index) {
      break;
    }

    current_index = came_from[current];
  }

  if (path.empty() || path.back().x != start.x || path.back().y != start.y) {
    return {};
  }

  std::reverse(path.begin(), path.end());
  return path;
}

// Convert the reconstructed grid path into a nav_msgs/Path in world coordinates.
nav_msgs::msg::Path AStarPlanner::buildPathMessage(
  const std_msgs::msg::Header & header,
  const Coordinate & start,
  const Coordinate & goal,
  const std::vector<int> & came_from) const
{
  const std::vector<Coordinate> coordinates = reconstructPath(start, goal, came_from);

  nav_msgs::msg::Path path;
  path.header = header;

  for (const Coordinate & coord : coordinates) {
    path.poses.push_back(gridToWorldPose(coord, header));
  }

  return applySpline(stringPull(path));
}

// Store the latest occupancy grid used for planning.
void AStarPlanner::setMap(const nav_msgs::msg::OccupancyGrid & map)
{
  map_ = map;
}

// Update the obstacle inflation radius used during traversability checks.
void AStarPlanner::setObstacleBufferMeters(double obstacle_buffer_m)
{
  obstacle_buffer_m_ = obstacle_buffer_m;
}

}  // namespace slambot_navigation
