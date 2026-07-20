#include "paesano_explorer/frontier_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace paesano_explorer
{

std::optional<FrontierGoal> FrontierSelector::select(
  const nav_msgs::msg::OccupancyGrid & map,
  const nav_msgs::msg::OccupancyGrid & inflated_map,
  const std::vector<FrontierCluster> & clusters,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const std::vector<FailedGoal> & failed_goals,
  const SelectionParameters & parameters) const
{
  const int map_width = static_cast<int>(map.info.width);
  const int map_height = static_cast<int>(map.info.height);
  const double resolution = static_cast<double>(map.info.resolution);
  const std::size_t expected_size =
    static_cast<std::size_t>(map_width) * static_cast<std::size_t>(map_height);
  if (
    map_width <= 0 || map_height <= 0 || resolution <= 0.0 ||
    map.data.size() != expected_size)
  {
    return std::nullopt;
  }

  constexpr double geometry_tolerance = 1e-6;
  const auto & map_origin = map.info.origin;
  const auto & inflated_origin = inflated_map.info.origin;
  const bool geometry_matches =
    inflated_map.header.frame_id == map.header.frame_id &&
    inflated_map.info.width == map.info.width &&
    inflated_map.info.height == map.info.height &&
    inflated_map.data.size() == expected_size &&
    std::abs(static_cast<double>(inflated_map.info.resolution) - resolution) <=
    geometry_tolerance &&
    std::abs(inflated_origin.position.x - map_origin.position.x) <= geometry_tolerance &&
    std::abs(inflated_origin.position.y - map_origin.position.y) <= geometry_tolerance &&
    std::abs(inflated_origin.orientation.x - map_origin.orientation.x) <= geometry_tolerance &&
    std::abs(inflated_origin.orientation.y - map_origin.orientation.y) <= geometry_tolerance &&
    std::abs(inflated_origin.orientation.z - map_origin.orientation.z) <= geometry_tolerance &&
    std::abs(inflated_origin.orientation.w - map_origin.orientation.w) <= geometry_tolerance;
  if (!geometry_matches) {
    return std::nullopt;
  }

  const auto cell_to_world = [&map, resolution](const GridCell & cell) {
      return std::pair<double, double>{
        map.info.origin.position.x + (static_cast<double>(cell.x) + 0.5) * resolution,
        map.info.origin.position.y + (static_cast<double>(cell.y) + 0.5) * resolution};
    };
  const auto world_to_cell = [&map, resolution](double world_x, double world_y) {
      return GridCell{
        static_cast<int>(std::floor((world_x - map.info.origin.position.x) / resolution)),
        static_cast<int>(std::floor((world_y - map.info.origin.position.y) / resolution))};
    };
  const auto is_known_free = [&map, map_width, map_height](
    const GridCell & cell,
    int occupied_threshold)
    {
      if (cell.x < 0 || cell.x >= map_width || cell.y < 0 || cell.y >= map_height) {
        return false;
      }

      const std::size_t index =
        static_cast<std::size_t>(cell.y) * static_cast<std::size_t>(map_width) +
        static_cast<std::size_t>(cell.x);
      const int occupancy = static_cast<int>(map.data[index]);
      return occupancy >= 0 && occupancy < occupied_threshold;
    };

  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  const double standoff_m = std::max(0.0, parameters.goal_standoff_m);
  const double blacklist_radius_m = std::max(0.0, parameters.blacklist_radius_m);

  std::optional<FrontierGoal> best_goal;
  double best_score = std::numeric_limits<double>::lowest();

  constexpr int neighbor_offsets[4][2] = {
    {0, 1},
    {0, -1},
    {1, 0},
    {-1, 0},
  };
  const std::size_t max_approach_points = static_cast<std::size_t>(
    std::max(1, parameters.max_approach_points_per_cluster));

  for (const FrontierCluster & cluster : clusters) {
    if (cluster.cells.empty()) {
      continue;
    }

    std::vector<GridCell> approach_cells;
    approach_cells.reserve(std::min(max_approach_points, cluster.cells.size()));
    approach_cells.push_back(cluster.representative);

    while (
      approach_cells.size() < max_approach_points &&
      approach_cells.size() < cluster.cells.size())
    {
      bool found_cell = false;
      GridCell farthest_cell;
      double farthest_minimum_distance_squared = -1.0;

      for (const GridCell & cell : cluster.cells) {
        double minimum_distance_squared = std::numeric_limits<double>::max();
        for (const GridCell & selected : approach_cells) {
          const double dx = static_cast<double>(cell.x - selected.x);
          const double dy = static_cast<double>(cell.y - selected.y);
          minimum_distance_squared =
            std::min(minimum_distance_squared, dx * dx + dy * dy);
        }

        if (minimum_distance_squared > farthest_minimum_distance_squared) {
          farthest_minimum_distance_squared = minimum_distance_squared;
          farthest_cell = cell;
          found_cell = true;
        }
      }

      if (!found_cell || farthest_minimum_distance_squared <= 0.0) {
        break;
      }
      approach_cells.push_back(farthest_cell);
    }

    for (const GridCell & approach_cell : approach_cells) {
      const auto [frontier_x, frontier_y] = cell_to_world(approach_cell);

      double unknown_direction_x = 0.0;
      double unknown_direction_y = 0.0;
      for (const auto & offset : neighbor_offsets) {
        const GridCell neighbor{
          approach_cell.x + offset[0],
          approach_cell.y + offset[1]};
        if (
          neighbor.x < 0 || neighbor.x >= map_width ||
          neighbor.y < 0 || neighbor.y >= map_height)
        {
          continue;
        }

        const std::size_t neighbor_index =
          static_cast<std::size_t>(neighbor.y) * static_cast<std::size_t>(map_width) +
          static_cast<std::size_t>(neighbor.x);
        if (map.data[neighbor_index] == -1) {
          unknown_direction_x += static_cast<double>(offset[0]);
          unknown_direction_y += static_cast<double>(offset[1]);
        }
      }

      const double unknown_direction_length =
        std::hypot(unknown_direction_x, unknown_direction_y);
      if (unknown_direction_length <= std::numeric_limits<double>::epsilon()) {
        continue;
      }

      const double goal_x =
        frontier_x - unknown_direction_x / unknown_direction_length * standoff_m;
      const double goal_y =
        frontier_y - unknown_direction_y / unknown_direction_length * standoff_m;

      const GridCell goal_cell = world_to_cell(goal_x, goal_y);
      if (!is_known_free(goal_cell, parameters.occupied_threshold)) {
        continue;
      }

      const std::size_t goal_index =
        static_cast<std::size_t>(goal_cell.y) * static_cast<std::size_t>(map_width) +
        static_cast<std::size_t>(goal_cell.x);
      if (inflated_map.data[goal_index] >= 100) {
        continue;
      }

      bool is_blacklisted = false;
      for (const FailedGoal & failed_goal : failed_goals) {
        if (std::hypot(goal_x - failed_goal.x, goal_y - failed_goal.y) < blacklist_radius_m) {
          is_blacklisted = true;
          break;
        }
      }
      if (is_blacklisted) {
        continue;
      }

      const double estimated_distance_m = std::hypot(goal_x - robot_x, goal_y - robot_y);
      const double frontier_value_m = cluster.information_gain * resolution;
      const double score =
        parameters.information_gain_weight * frontier_value_m -
        parameters.distance_weight * estimated_distance_m;
      if (score <= best_score) {
        continue;
      }

      const double yaw = std::atan2(frontier_y - goal_y, frontier_x - goal_x);
      FrontierGoal candidate;
      candidate.pose.header = map.header;
      candidate.pose.pose.position.x = goal_x;
      candidate.pose.pose.position.y = goal_y;
      candidate.pose.pose.orientation.z = std::sin(yaw * 0.5);
      candidate.pose.pose.orientation.w = std::cos(yaw * 0.5);
      candidate.cluster_size = cluster.cells.size();
      candidate.score = score;

      best_score = score;
      best_goal = candidate;
    }
  }

  return best_goal;
}

}  // namespace paesano_explorer
