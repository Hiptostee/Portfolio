#include <cmath>
#include <limits>
#include <set>
#include <utility>

#include "paesano_navigation/a_star.hpp"

namespace paesano_navigation
{

// This is the main function where A* is ran.
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

  // First, check if the map is even valid to run A* on.
  if (!isMapValid()) {
    return path;
  }

  // Convert start and goal poses to grid coordinates, and if they are out of bounds or not traversable then return an empty path.
  Coordinate start_cell;
  Coordinate goal_cell;
  if (!worldToGrid(start.pose.position.x, start.pose.position.y, start_cell)) {
    return path;
  }
  if (!worldToGrid(goal.pose.position.x, goal.pose.position.y, goal_cell)) {
    return path;
  }
  // Allow the robot to start inside the inflated obstacle buffer, but still require the goal to be free.
  if (!isCellTraversable(goal_cell)) {
    return path;
  }

  // Define relevant constants.
  const size_t width = map_.info.width;
  const size_t cell_count =
    static_cast<size_t>(map_.info.width) * static_cast<size_t>(map_.info.height);
  const size_t start_index = toIndex(start_cell);
  const size_t goal_index = toIndex(goal_cell);
  const double inf = std::numeric_limits<double>::infinity();

  // Set up the open_set ordered by lowest f-score. Existing entries are erased
  // before score updates, so the set does not accumulate stale duplicates.
  std::set<std::pair<double, size_t>> open_set;

  // Set up the closed_set which is the nodes we have already finalized and don't want to check again.
  closed_.assign(cell_count, 0);
  
  // Set up the came_from vector which maps the current index with the neighbor it came from.
  came_from_.assign(cell_count, kNoParent);

  // Scores (f = g (cost to get to where it is) + h (heuristic cost))
  g_score_.assign(cell_count, inf);
  h_score_.resize(cell_count);

  for (size_t index = 0; index < cell_count; ++index) {
    const Coordinate cell{
      static_cast<int>(index % width),
      static_cast<int>(index / width)
    };
    h_score_[index] = heuristic(cell, goal_cell);
  }

  // Initialize the scores of the start index.
  g_score_[start_index] = 0.0;
  open_set.emplace(h_score_[start_index], start_index);
  neighbors_.reserve(8);

  /* This is the main while loop of the algorithm. While there are more nodes to explore (or within the loop the goal 
     hasn't been reached), continue.
  */
  while (!open_set.empty()) {

    // Pop the lowest cost node from the open set.
    const size_t current_index = open_set.begin()->second;
    open_set.erase(open_set.begin());

    // If we have already finalized the node, skip it.
    if (closed_[current_index]) {
      continue;
    }

    // If the node is the goal, return the path to get to it.
    if (current_index == goal_index) {
      return buildPathMessage(path.header, start_cell, goal_cell, goal.pose, came_from_);
    }


    // Add the recently popped lowest cost node to the closed set.
    closed_[current_index] = 1;

    // Convert the index into a coordinate in the grid we can run A* on.
    const Coordinate current{
      static_cast<int>(current_index % width),
      static_cast<int>(current_index / width)
    };


    // Loop through each of the neighbors of the most recently popped lowest cost node.
    getNeighbors(current, neighbors_);
    for (const Coordinate & neighbor : neighbors_) {
      const size_t neighbor_index = toIndex(neighbor);
      if (closed_[neighbor_index]) {
        continue;
      }

      // Calculate the tentative g cost of the neighbor.
      const int dx = neighbor.x - current.x;
      const int dy = neighbor.y - current.y;
      const double step_cost = (std::abs(dx) == 1 && std::abs(dy) == 1) ? std::sqrt(2.0) : 1.0;
      const double obstacle_penalty = getCellTraversalPenalty(neighbor);
      if (!std::isfinite(obstacle_penalty)) {
        continue;
      }
      const double tentative_g = g_score_[current_index] + step_cost + obstacle_penalty;

      /* If the tentative g cost is lower than what we have previously calculated for this node, then update its total score,
         add it to the open set, and update the came_from vector. 
      */
      if (tentative_g < g_score_[neighbor_index]) {
        if (std::isfinite(g_score_[neighbor_index])) {
          open_set.erase({g_score_[neighbor_index] + h_score_[neighbor_index], neighbor_index});
        }
        came_from_[neighbor_index] = current_index;
        g_score_[neighbor_index] = tentative_g;
        open_set.emplace(g_score_[neighbor_index] + h_score_[neighbor_index], neighbor_index);
      }
    }
  }

  return path;
}

}  // namespace paesano_navigation
