#ifndef PAESANO_EXPLORER__EXPLORER_NODE_HPP_
#define PAESANO_EXPLORER__EXPLORER_NODE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "paesano_explorer/frontier_clusterer.hpp"
#include "paesano_explorer/frontier_detector.hpp"
#include "paesano_explorer/frontier_selector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace paesano_explorer
{

class ExplorerNode final : public rclcpp::Node
{
public:
  explicit ExplorerNode(const rclcpp::NodeOptions & options);

private:
  enum class State { WAITING_FOR_DATA, SELECTING, NAVIGATING, COMPLETE };

  void handleMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void handlePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void handleNavigationResult(const std_msgs::msg::String::SharedPtr msg);
  void tick();
  void publishGoal(const FrontierGoal & goal);
  void publishMarkers(
    const std::vector<GridCell> & cells,
    const std::optional<FrontierGoal> & selected_goal);
  std::string stateName() const;

  FrontierDetector detector_;
  FrontierClusterer clusterer_;
  FrontierSelector selector_;

  State state_{State::WAITING_FOR_DATA};
  nav_msgs::msg::OccupancyGrid map_;
  geometry_msgs::msg::PoseStamped robot_pose_;
  std::optional<FrontierGoal> active_goal_;
  std::vector<FailedGoal> failed_goals_;
  bool have_map_{false};
  bool have_pose_{false};
  bool exploration_started_{false};
  int free_threshold_{50};
  int minimum_cluster_size_{8};
  int completion_confirmation_updates_{3};
  int empty_frontier_updates_{0};
  std::uint64_t map_revision_{0};
  std::uint64_t last_processed_map_revision_{0};
  SelectionParameters selection_parameters_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_result_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace paesano_explorer

#endif  // PAESANO_EXPLORER__EXPLORER_NODE_HPP_
