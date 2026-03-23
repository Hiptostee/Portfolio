#ifndef SLAMBOT_NAVIGATION__A_STAR_HPP_
#define SLAMBOT_NAVIGATION__A_STAR_HPP_

#include "slambot_navigation/a_star_helpers.hpp"


namespace slambot_navigation
{
class AStarPlanner
{
public:
  nav_msgs::msg::Path plan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;
  void setMap(const nav_msgs::msg::OccupancyGrid & map);
  void setObstacleBufferMeters(double obstacle_buffer_m);
  bool isMapValid() const;
  bool isInBounds(const Coordinate & cell) const;
  size_t toIndex(const Coordinate & cell) const;
  bool isCellTraversable(const Coordinate & cell) const;
  bool worldToGrid(double wx, double wy, Coordinate & cell) const;
  geometry_msgs::msg::PoseStamped gridToWorldPose(const Coordinate & cell,
    const std_msgs::msg::Header & header) const;
  double heuristic(const Coordinate & a, const Coordinate & b) const;
  std::vector<Coordinate> getNeighbors(const Coordinate & cell) const;
  std::vector<Coordinate> reconstructPath(
    const Coordinate & start,
    const Coordinate & goal,
    const std::vector<int> & came_from) const;
  nav_msgs::msg::Path buildPathMessage(const std_msgs::msg::Header & header, const Coordinate & start,
    const Coordinate & goal,
    const std::vector<int> & came_from) const;

private:
  nav_msgs::msg::OccupancyGrid map_;
  double obstacle_buffer_m_{0.3};
};

}  // namespace slambot_navigation

#endif  // SLAMBOT_NAVIGATION__A_STAR_HPP_
