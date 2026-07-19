#ifndef PAESANO_EXPLORER__TYPES_HPP_
#define PAESANO_EXPLORER__TYPES_HPP_

#include <cstddef>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace paesano_explorer
{

struct GridCell
{
  int x{0};
  int y{0};
};

struct FrontierCluster
{
  std::vector<GridCell> cells;
  double centroid_x_cells{0.0};
  double centroid_y_cells{0.0};
  GridCell representative;
  double information_gain{0.0};
  double estimated_distance_m{0.0};
  double score{0.0};
};

struct FrontierGoal
{
  geometry_msgs::msg::PoseStamped pose;
  std::size_t cluster_size{0};
  double score{0.0};
};

struct FailedGoal
{
  double x{0.0};
  double y{0.0};
};

struct SelectionParameters
{
  double goal_standoff_m{0.30};
  double blacklist_radius_m{0.50};
  double information_gain_weight{1.0};
  double distance_weight{1.0};
  int occupied_threshold{50};
};

}  // namespace paesano_explorer

#endif  // PAESANO_EXPLORER__TYPES_HPP_
