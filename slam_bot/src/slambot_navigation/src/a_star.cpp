#include <cmath>
#include <limits>
#include <queue>

#include "slambot_navigation/a_star.hpp"

namespace slambot_navigation
{
namespace
{

struct OpenSetEntry
{
  size_t index;
  double f_score;
};

struct OpenSetCompare
{
  bool operator()(const OpenSetEntry & lhs, const OpenSetEntry & rhs) const
  {
    return lhs.f_score > rhs.f_score;
  }
};

}  // namespace


nav_msgs::msg::Path AStarPlanner::plan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path path;
  path.header = goal.header;

  if (path.header.frame_id.empty()) {
    path.header = start.header;
  }

  if (path.header.frame_id.empty()) {
    path.header.frame_id = "map";
  }

  if (!isMapValid()) {
    return path;
  }

  Coordinate start_cell;
  Coordinate goal_cell;
  if (!worldToGrid(start.pose.position.x, start.pose.position.y, start_cell)) {
    return path;
  }
  if (!worldToGrid(goal.pose.position.x, goal.pose.position.y, goal_cell)) {
    return path;
  }
  if (!isCellTraversable(start_cell) || !isCellTraversable(goal_cell)) {
    return path;
  }

  const size_t width = map_.info.width;
  const size_t cell_count =
    static_cast<size_t>(map_.info.width) * static_cast<size_t>(map_.info.height);
  const size_t start_index = toIndex(start_cell);
  const size_t goal_index = toIndex(goal_cell);
  const double inf = std::numeric_limits<double>::infinity();

  std::priority_queue<OpenSetEntry, std::vector<OpenSetEntry>, OpenSetCompare> open_set;
  std::vector<bool> closed(cell_count, false);
  std::vector<int> came_from(cell_count, -1);
  std::vector<double> g_score(cell_count, inf);
  std::vector<double> h_score(cell_count, inf);
  std::vector<double> f_score(cell_count, inf);

  g_score[start_index] = 0.0;
  h_score[start_index] = heuristic(start_cell, goal_cell);
  f_score[start_index] = h_score[start_index];
  open_set.push(OpenSetEntry{start_index, f_score[start_index]});

  while (!open_set.empty()) {
    const OpenSetEntry current_entry = open_set.top();
    open_set.pop();

    const size_t current_index = current_entry.index;
    if (current_entry.f_score > f_score[current_index] || closed[current_index]) {
      continue;
    }

    if (current_index == goal_index) {
      return buildPathMessage(path.header, start_cell, goal_cell, came_from);
    }

    closed[current_index] = true;

    const Coordinate current{
      static_cast<int>(current_index % width),
      static_cast<int>(current_index / width)
    };

    for (const Coordinate & neighbor : getNeighbors(current)) {
      const size_t neighbor_index = toIndex(neighbor);
      if (closed[neighbor_index]) {
        continue;
      }

      const int dx = neighbor.x - current.x;
      const int dy = neighbor.y - current.y;
      const double step_cost = (std::abs(dx) == 1 && std::abs(dy) == 1) ? std::sqrt(2.0) : 1.0;
      const double tentative_g = g_score[current_index] + step_cost;

      if (tentative_g < g_score[neighbor_index]) {
        came_from[neighbor_index] = static_cast<int>(current_index);
        g_score[neighbor_index] = tentative_g;
        h_score[neighbor_index] = heuristic(neighbor, goal_cell);
        f_score[neighbor_index] = g_score[neighbor_index] + h_score[neighbor_index];
        open_set.push(OpenSetEntry{neighbor_index, f_score[neighbor_index]});
      }
    }
  }

  return path;
}

void AStarPlanner::setMap(const nav_msgs::msg::OccupancyGrid & map)
{
  map_ = map;
}

void AStarPlanner::setObstacleBufferMeters(double obstacle_buffer_m)
{
  obstacle_buffer_m_ = obstacle_buffer_m;
}

}  // namespace slambot_navigation
