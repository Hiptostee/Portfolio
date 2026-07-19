#ifndef PAESANO_EXPLORER__FRONTIER_SELECTOR_HPP_
#define PAESANO_EXPLORER__FRONTIER_SELECTOR_HPP_

#include <optional>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "paesano_explorer/types.hpp"

namespace paesano_explorer
{

class FrontierSelector
{
public:
  std::optional<FrontierGoal> select(
    const nav_msgs::msg::OccupancyGrid & map,
    const std::vector<FrontierCluster> & clusters,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const std::vector<FailedGoal> & failed_goals,
    const SelectionParameters & parameters) const;
};

}  // namespace paesano_explorer

#endif  // PAESANO_EXPLORER__FRONTIER_SELECTOR_HPP_
