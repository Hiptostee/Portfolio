#include "paesano_explorer/frontier_detector.hpp"

namespace paesano_explorer
{

std::vector<GridCell> FrontierDetector::detect(
  const nav_msgs::msg::OccupancyGrid & map,
  int free_threshold) const
{
  int height = static_cast<int>(map.info.height);
  int width = static_cast<int>(map.info.width);

  const std::size_t expected_size = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);

  if (width <= 0 || height <= 0 || expected_size != map.data.size()) {
    return {};
  }

  std::vector<GridCell> frontier_cells;

  constexpr int neighbor_offsets[4][2] = {
    {0, 1},
    {0, -1},
    {1, 0},
    {-1, 0},
  };

  for (int j = 0; j < height; j++) {
    for (int i = 0; i < width; i++) {
      const std::size_t index = static_cast<std::size_t>(j) * static_cast<std::size_t>(width) + static_cast<std::size_t>(i);
      const int value = static_cast<int>(map.data[index]);
      if (value < 0 || value >= free_threshold) {
        continue;
      }
      bool touches_unknown = false;
      for (const auto & offset : neighbor_offsets) {
        const int neighbor_x = i + offset[0];
        const int neighbor_y = j + offset[1];
        if (neighbor_x < 0 || neighbor_x >= width || neighbor_y < 0 || neighbor_y >= height) {
          continue;
        }
        const std::size_t neighbor_index = static_cast<std::size_t>(neighbor_y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(neighbor_x);

        if (map.data[neighbor_index] == -1) {
          touches_unknown = true;
          break;
        }
      }

      if (touches_unknown) {
        frontier_cells.push_back(GridCell{i, j});
      }
    }
  }

  return frontier_cells;
}

}  // namespace paesano_explorer
