#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "paesano_navigation/action/a_star.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace paesano_orchestrator {
class Orchestrator final : public rclcpp::Node {
 public:
  Orchestrator();
 private:
  using AStar = paesano_navigation::action::AStar;
  enum class State { IDLE, PLANNING, NAVIGATING, WAITING, REPLANNING };
  void tick();
  void requestPlan();
  void publishPlanningMap();
  void publishState();
  std::string stateName() const;
  void handleGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void handlePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void handleMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void handleLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void handleBlocked(const std_msgs::msg::Bool::SharedPtr msg);
  double delay_{5.0}, goal_tolerance_{0.15}; int occupied_threshold_{50};
  State state_{State::IDLE}; bool have_goal_{false}, have_pose_{false}, have_map_{false}, have_local_map_{false}, blocked_{false}, plan_in_flight_{false};
  geometry_msgs::msg::PoseStamped goal_, pose_; nav_msgs::msg::OccupancyGrid map_, local_map_; rclcpp::Time blocked_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_, pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_, local_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr blocked_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp_action::Client<AStar>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace paesano_orchestrator
