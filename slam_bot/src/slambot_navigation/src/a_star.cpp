#include <cmath>
#include <limits>
#include <queue>

#include "slambot_navigation/a_star.hpp"

namespace slambot_navigation
{

// Define properties for the open_set.
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
  if (!isCellTraversable(start_cell) || !isCellTraversable(goal_cell)) {
    return path;
  }

  // Define relevant constants.
  const size_t width = map_.info.width;
  const size_t cell_count =
    static_cast<size_t>(map_.info.width) * static_cast<size_t>(map_.info.height);
  const size_t start_index = toIndex(start_cell);
  const size_t goal_index = toIndex(goal_cell);
  const double inf = std::numeric_limits<double>::infinity();

  // Set up the open_set which is a min heap of all the nodes we can possibly visit.
  std::priority_queue<OpenSetEntry, std::vector<OpenSetEntry>, OpenSetCompare> open_set;

  // Set up the closed_set which is the nodes we have already finalized and don't want to check again.
  std::vector<bool> closed(cell_count, false);
  
  // Set up the came_from vector which maps the current index with the neighbor it came from.
  std::vector<int> came_from(cell_count, -1);

  // Scores (f = g (cost to get to where it is) + h (heuristic cost))
  std::vector<double> g_score(cell_count, inf);
  std::vector<double> h_score(cell_count, inf);
  std::vector<double> f_score(cell_count, inf);

  // Initialize the scores of the start index.
  g_score[start_index] = 0.0;
  h_score[start_index] = heuristic(start_cell, goal_cell);
  f_score[start_index] = h_score[start_index];
  open_set.push(OpenSetEntry{start_index, f_score[start_index]});

  /* This is the main while loop of the algorithm. While there are more nodes to explore (or within the loop the goal 
     hasn't been reached), continue.
  */
  while (!open_set.empty()) {

    // Pop the lowest cost node from the open set.
    const OpenSetEntry current_entry = open_set.top();
    open_set.pop();

    // If the node has a worse score than what we've already recorded, or if we have already finalized the node, skip it.
    const size_t current_index = current_entry.index;
    if (current_entry.f_score > f_score[current_index] || closed[current_index]) {
      continue;
    }

    // If the node is the goal, return the path to get to it.
    if (current_index == goal_index) {
      return buildPathMessage(path.header, start_cell, goal_cell, came_from);
    }


    // Add the recently popped lowest cost node to the closed set.
    closed[current_index] = true;

    // Convert the index into a coordinate in the grid we can run A* on.
    const Coordinate current{
      static_cast<int>(current_index % width),
      static_cast<int>(current_index / width)
    };


    // Loop through each of the neighbors of the most recently popped lowest cost node.
    for (const Coordinate & neighbor : getNeighbors(current)) {
      const size_t neighbor_index = toIndex(neighbor);
      if (closed[neighbor_index]) {
        continue;
      }

      // Calculate the tentative g cost of the neighbor.
      const int dx = neighbor.x - current.x;
      const int dy = neighbor.y - current.y;
      const double step_cost = (std::abs(dx) == 1 && std::abs(dy) == 1) ? std::sqrt(2.0) : 1.0;
      const double tentative_g = g_score[current_index] + step_cost;

      /* If the tentative g cost is lower than what we have previously calculated for this node, then update its total score,
         add it to the open set, and update the came_from vector. 
      */
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

}  // namespace slambot_navigation
