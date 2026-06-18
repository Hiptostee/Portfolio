#pragma once

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "paesano_local_map/local_map.hpp"
#include "paesano_navigation/action/a_star.hpp"

namespace paesano_orchestrator
{

class Orchestrator : public rclcpp::Node
{
public:
  explicit Orchestrator(const rclcpp::NodeOptions & options);

private:
  enum class State { IDLE, PLANNING, NAVIGATING, REPLANNING, GOAL_REACHED };

  // ── Policy (see orchestrator.cpp) ─────────────────────────────────────────
  void tick();
  bool goalReached() const;
  void transition(State next);
  void lqr_stop();
  bool send_astar_goal(const geometry_msgs::msg::PoseStamped & goal);
  void advancePathIndex();

  // ── Topic callbacks ────────────────────────────────────────────────────────
  void goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg);
  void isNavigatingCallback(std_msgs::msg::Bool::SharedPtr msg);

  // ── A* action callbacks ────────────────────────────────────────────────────
  using AStar = paesano_navigation::action::AStar;
  using GoalHandleAStar = rclcpp_action::ClientGoalHandle<AStar>;

  void goalResponseCallback(GoalHandleAStar::SharedPtr goal_handle);
  void goalResultCallback(const GoalHandleAStar::WrappedResult & result);

  // ── ROS interfaces ─────────────────────────────────────────────────────────
  rclcpp_action::Client<AStar>::SharedPtr            astar_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr  lqr_stop_client_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  state_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr     scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             nav_sub_;

  rclcpp::TimerBase::SharedPtr tick_timer_;

  // ── TF2 ───────────────────────────────────────────────────────────────────
  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ── Navigation state ──────────────────────────────────────────────────────
  State       state_          = State::IDLE;
  bool        is_navigating_  = false;
  bool        lqr_started_    = false;  // true once LQR confirms it's tracking
  std::size_t path_index_     = 0;

  geometry_msgs::msg::PoseStamped current_goal_;
  geometry_msgs::msg::PoseStamped current_pose_;
  nav_msgs::msg::Path             current_path_;

  paesano_local_map::LocalMap local_map_;

  // ── Parameters ────────────────────────────────────────────────────────────
  double goal_tolerance_m_;
  int    lookahead_n_;
};

}  // namespace paesano_orchestrator
