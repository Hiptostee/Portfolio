#include "paesano_explorer/frontier_detector.hpp"

namespace paesano_explorer
{

std::vector<GridCell> FrontierDetector::detect(
  const nav_msgs::msg::OccupancyGrid & map,
  int free_threshold) const
{
  (void)map;
  (void)free_threshold;

  // TODO(joseph): Implement the O(width * height) frontier detection pass.
  return {};
}

}  // namespace paesano_explorer
