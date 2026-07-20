#include "paesano_explorer/frontier_clusterer.hpp"

#include <cstdint>
#include <limits>
#include <queue>
#include <utility>

namespace paesano_explorer
{

std::vector<FrontierCluster> FrontierClusterer::cluster(
  const std::vector<GridCell> & frontier_cells,
  int map_width,
  int map_height,
  std::size_t minimum_cluster_size) const
{
  if (map_width <= 0 || map_height <= 0 || frontier_cells.empty()) {
    return {};
  }

  const std::size_t cell_count =
    static_cast<std::size_t>(map_width) * static_cast<std::size_t>(map_height);
  std::vector<std::uint8_t> frontier_mask(cell_count, 0U);
  std::vector<std::uint8_t> visited(cell_count, 0U);

  const auto is_in_bounds = [map_width, map_height](const GridCell & cell) {
      return cell.x >= 0 && cell.x < map_width &&
             cell.y >= 0 && cell.y < map_height;
    };
  const auto cell_index = [map_width](const GridCell & cell) {
      return static_cast<std::size_t>(cell.y) * static_cast<std::size_t>(map_width) +
             static_cast<std::size_t>(cell.x);
    };

  for (const GridCell & cell : frontier_cells) {
    if (is_in_bounds(cell)) {
      frontier_mask[cell_index(cell)] = 1U;
    }
  }

  constexpr int neighbor_offsets[8][2] = {
    {-1, -1}, {0, -1}, {1, -1},
    {-1, 0},           {1, 0},
    {-1, 1},  {0, 1},  {1, 1},
  };

  std::vector<FrontierCluster> clusters;
  for (const GridCell & start : frontier_cells) {
    if (!is_in_bounds(start)) {
      continue;
    }

    const std::size_t start_index = cell_index(start);
    if (visited[start_index] != 0U) {
      continue;
    }

    std::queue<GridCell> pending;
    std::vector<GridCell> cluster_cells;
    double sum_x = 0.0;
    double sum_y = 0.0;

    visited[start_index] = 1U;
    pending.push(start);

    while (!pending.empty()) {
      const GridCell current = pending.front();
      pending.pop();

      cluster_cells.push_back(current);
      sum_x += static_cast<double>(current.x);
      sum_y += static_cast<double>(current.y);

      for (const auto & offset : neighbor_offsets) {
        const GridCell neighbor{current.x + offset[0], current.y + offset[1]};
        if (!is_in_bounds(neighbor)) {
          continue;
        }

        const std::size_t neighbor_index = cell_index(neighbor);
        if (frontier_mask[neighbor_index] == 0U || visited[neighbor_index] != 0U) {
          continue;
        }

        visited[neighbor_index] = 1U;
        pending.push(neighbor);
      }
    }

    if (cluster_cells.size() < minimum_cluster_size) {
      continue;
    }

    FrontierCluster cluster;
    cluster.centroid_x_cells = sum_x / static_cast<double>(cluster_cells.size());
    cluster.centroid_y_cells = sum_y / static_cast<double>(cluster_cells.size());

    double closest_distance_squared = std::numeric_limits<double>::max();
    for (const GridCell & cell : cluster_cells) {
      const double dx = static_cast<double>(cell.x) - cluster.centroid_x_cells;
      const double dy = static_cast<double>(cell.y) - cluster.centroid_y_cells;
      const double distance_squared = dx * dx + dy * dy;
      if (distance_squared < closest_distance_squared) {
        closest_distance_squared = distance_squared;
        cluster.representative = cell;
      }
    }

    cluster.information_gain = static_cast<double>(cluster_cells.size());
    cluster.cells = std::move(cluster_cells);
    clusters.push_back(std::move(cluster));
  }

  return clusters;
}

}  // namespace paesano_explorer
