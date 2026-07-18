#ifndef PAESANO_NAVIGATION__A_STAR_HPP_
#define PAESANO_NAVIGATION__A_STAR_HPP_

#include "paesano_navigation/a_star_helpers.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>


namespace paesano_navigation
{
class AStarPlanner
{
public:
  virtual ~AStarPlanner() = default;

  nav_msgs::msg::Path plan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;
  void setMap(const nav_msgs::msg::OccupancyGrid & map);
  void setObstacleBufferMeters(double obstacle_buffer_m);
  void setClearanceDecayLengthMeters(double clearance_decay_length_m);
  void setClearanceCostScale(double clearance_cost_scale);
  const nav_msgs::msg::OccupancyGrid & getInflatedMap() const;
  bool isMapValid() const;
  bool isInBounds(const Coordinate & cell) const;
  size_t toIndex(const Coordinate & cell) const;
  bool isCellTraversable(const Coordinate & cell) const;
  double getCellTraversalPenalty(const Coordinate & cell) const;
  bool worldToGrid(double wx, double wy, Coordinate & cell) const;
  geometry_msgs::msg::PoseStamped gridToWorldPose(const Coordinate & cell,
    const std_msgs::msg::Header & header) const;
  double heuristic(const Coordinate & a, const Coordinate & b) const;
  std::vector<Coordinate> getNeighbors(const Coordinate & cell) const;
  void getNeighbors(const Coordinate & cell, std::vector<Coordinate> & neighbors) const;
  std::vector<Coordinate> reconstructPath(
    const Coordinate & start,
    const Coordinate & goal,
    const std::vector<std::size_t> & came_from) const;
  nav_msgs::msg::Path buildPathMessage(
    const std_msgs::msg::Header & header,
    const Coordinate & start,
    const Coordinate & goal,
    const geometry_msgs::msg::Pose & goal_pose,
    const std::vector<std::size_t> & came_from) const;
  bool is_line_clear(const geometry_msgs::msg::Pose& start, 
                                 const geometry_msgs::msg::Pose& end, 
                                 const nav_msgs::msg::OccupancyGrid& map) const;
  nav_msgs::msg::Path stringPull(const nav_msgs::msg::Path& path) const;
  nav_msgs::msg::Path applySpline(const nav_msgs::msg::Path& sparse_path) const;
  geometry_msgs::msg::Point catmullRom(
    const geometry_msgs::msg::Point & p0,
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    const geometry_msgs::msg::Point & p3,
    double t) const;
  
  

private:
  static constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();

  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::OccupancyGrid map_inflated_;
  std::vector<double> traversal_costs_;
  mutable std::vector<uint8_t> closed_;
  mutable std::vector<std::size_t> came_from_;
  mutable std::vector<double> g_score_;
  mutable std::vector<double> h_score_;
  mutable std::vector<Coordinate> neighbors_;
  double obstacle_buffer_m_{0.3};
  double clearance_decay_length_m_{0.6};
  double clearance_cost_scale_{3.0};
  void buildInflatedMap();
};

}  // namespace paesano_navigation

#endif  // PAESANO_NAVIGATION__A_STAR_HPP_
