#ifndef SLAMBOT_NAVIGATION__A_STAR_ACTION_SERVER_HPP_
#define SLAMBOT_NAVIGATION__A_STAR_ACTION_SERVER_HPP_

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "slambot_navigation/a_star.hpp"
#include "slambot_navigation/action/a_star.hpp"

namespace slambot_navigation
{

class AStarActionServer : public rclcpp::Node
{
public:
  using AStarAction = slambot_navigation::action::AStar;
  using GoalHandleAStar = rclcpp_action::ServerGoalHandle<AStarAction>;

  explicit AStarActionServer(const rclcpp::NodeOptions & options);

private:
  AStarPlanner planner_;
  rclcpp_action::Server<AStarAction>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_map_publisher_;
  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool have_map_{false};
  bool have_pose_{false};

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const AStarAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleAStar> goal_handle);

  void handleMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void handlePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void handle_accepted(const std::shared_ptr<GoalHandleAStar> goal_handle);
  void execute(const std::shared_ptr<GoalHandleAStar> goal_handle);
};

}  // namespace slambot_navigation

#endif  // SLAMBOT_NAVIGATION__A_STAR_ACTION_SERVER_HPP_
