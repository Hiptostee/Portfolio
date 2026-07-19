#include "paesano_explorer/frontier_selector.hpp"

namespace paesano_explorer
{

std::optional<FrontierGoal> FrontierSelector::select(
  const nav_msgs::msg::OccupancyGrid & map,
  const std::vector<FrontierCluster> & clusters,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const std::vector<FailedGoal> & failed_goals,
  const SelectionParameters & parameters) const
{
  (void)map;
  (void)clusters;
  (void)robot_pose;
  (void)failed_goals;
  (void)parameters;

  // TODO(joseph): Choose and orient a safe known-free frontier goal.
  return std::nullopt;
}

}  // namespace paesano_explorer
