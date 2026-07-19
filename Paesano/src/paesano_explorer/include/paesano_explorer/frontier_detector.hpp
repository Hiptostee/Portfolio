#ifndef PAESANO_EXPLORER__FRONTIER_DETECTOR_HPP_
#define PAESANO_EXPLORER__FRONTIER_DETECTOR_HPP_

#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "paesano_explorer/types.hpp"

namespace paesano_explorer
{

class FrontierDetector
{
public:
  std::vector<GridCell> detect(
    const nav_msgs::msg::OccupancyGrid & map,
    int free_threshold) const;
};

}  // namespace paesano_explorer

#endif  // PAESANO_EXPLORER__FRONTIER_DETECTOR_HPP_
