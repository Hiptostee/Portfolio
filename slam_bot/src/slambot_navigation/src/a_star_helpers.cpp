#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "slambot_navigation/a_star_helpers.hpp"
#include "slambot_navigation/a_star.hpp"

namespace slambot_navigation
{

bool AStarPlanner::isMapValid() const
{
  return map_.info.width > 0 && map_.info.height > 0 && map_.info.resolution > 0 &&
         map_.data.size() == map_.info.width * map_.info.height;
}

bool AStarPlanner::isInBounds(const Coordinate & cell) const
{
  const int width = map_.info.width;
  const int height = map_.info.height;
  return cell.x >= 0 && cell.y >= 0 && cell.x < width && cell.y < height;
}

size_t AStarPlanner::toIndex(const Coordinate & cell) const
{
  if (!isInBounds(cell)) {
    throw std::out_of_range("Grid cell is out of bounds");
  }
  return cell.y * map_.info.width + cell.x;
}

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

double AStarPlanner::heuristic(const Coordinate & a, const Coordinate & b) const
{
  return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

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

  return path;
}

}  // namespace slambot_navigation
