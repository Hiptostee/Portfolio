#include "paesano_orchestrator/orchestrator.hpp"

#include <chrono>
#include <cmath>
#include <limits>

namespace paesano_orchestrator
{

using namespace std::chrono_literals;

Orchestrator::Orchestrator(const rclcpp::NodeOptions & options)
: rclcpp::Node("orchestrator", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  local_map_(
    declare_parameter<double>("local_map_size_m",       6.0),
    declare_parameter<double>("local_map_resolution_m", 0.05),
    declare_parameter<double>("obstacle_radius_m",      0.25))
{
  goal_tolerance_m_ = declare_parameter<double>("goal_tolerance_m",       0.15);
  lookahead_n_      = declare_parameter<int>   ("path_blocked_lookahead", 5);
  const int tick_ms = declare_parameter<int>   ("tick_period_ms",         100);

  state_pub_ = create_publisher<std_msgs::msg::String>("/orchestrator/state", 10);

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/navigation/goal", 1,
    std::bind(&Orchestrator::goalCallback, this, std::placeholders::_1));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/estimated_pose", 10,
    std::bind(&Orchestrator::poseCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&Orchestrator::scanCallback, this, std::placeholders::_1));

  nav_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/is_navigating", 10,
    std::bind(&Orchestrator::isNavigatingCallback, this, std::placeholders::_1));

  astar_client_    = rclcpp_action::create_client<AStar>(this, "a_star");
  lqr_stop_client_ = create_client<std_srvs::srv::Trigger>("/lqr/stop");

  tick_timer_ = create_wall_timer(
    std::chrono::milliseconds(tick_ms),
    std::bind(&Orchestrator::tick, this));

  RCLCPP_INFO(get_logger(), "Orchestrator ready (tick %d ms).", tick_ms);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  tick() — the navigation policy
// ═══════════════════════════════════════════════════════════════════════════════

void Orchestrator::tick()
{
  switch (state_) {
    case State::IDLE:
      break;

    case State::PLANNING:
    case State::REPLANNING:
      // Async — result handled in goalResultCallback()
      break;

    case State::NAVIGATING:
      // Wait for LQR to confirm it's tracking before watching for it to stop.
      // Without this, the first tick after entering NAVIGATING would see
      // is_navigating_=false (LQR hasn't started yet) and bail to IDLE.
      if (is_navigating_) {
        lqr_started_ = true;
      }
      if (lqr_started_ && !is_navigating_) {
        // LQR stopped externally (user cancel / pause)
        transition(State::IDLE);
      } else if (local_map_.isPathBlocked(current_path_, path_index_, lookahead_n_)) {
        lqr_stop();
        if (!send_astar_goal(current_goal_)) {
          transition(State::IDLE);
        } else {
          transition(State::REPLANNING);
        }
      } else if (goalReached()) {
        lqr_stop();
        transition(State::GOAL_REACHED);
      }
      break;

    case State::GOAL_REACHED:
      RCLCPP_INFO(get_logger(), "Goal reached.");
      transition(State::IDLE);
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  State helpers
// ═══════════════════════════════════════════════════════════════════════════════

void Orchestrator::transition(State next)
{
  static const auto name = [](State s) -> const char * {
    switch (s) {
      case State::IDLE:         return "IDLE";
      case State::PLANNING:     return "PLANNING";
      case State::NAVIGATING:   return "NAVIGATING";
      case State::REPLANNING:   return "REPLANNING";
      case State::GOAL_REACHED: return "GOAL_REACHED";
    }
    return "UNKNOWN";
  };

  RCLCPP_INFO(get_logger(), "State: %s → %s", name(state_), name(next));
  state_ = next;

  std_msgs::msg::String msg;
  msg.data = name(next);
  state_pub_->publish(msg);
}

bool Orchestrator::goalReached() const
{
  const double dx = current_pose_.pose.position.x - current_goal_.pose.position.x;
  const double dy = current_pose_.pose.position.y - current_goal_.pose.position.y;
  return std::hypot(dx, dy) < goal_tolerance_m_;
}

void Orchestrator::lqr_stop()
{
  if (!lqr_stop_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "LQR stop service not ready");
    return;
  }
  lqr_stop_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
}

bool Orchestrator::send_astar_goal(const geometry_msgs::msg::PoseStamped & goal)
{
  if (!astar_client_->wait_for_action_server(std::chrono::nanoseconds(0))) {
    RCLCPP_WARN(get_logger(), "A* action server not ready");
    return false;
  }

  AStar::Goal goal_msg;
  goal_msg.goal = goal;

  rclcpp_action::Client<AStar>::SendGoalOptions opts;
  opts.goal_response_callback =
    std::bind(&Orchestrator::goalResponseCallback, this, std::placeholders::_1);
  opts.result_callback =
    std::bind(&Orchestrator::goalResultCallback, this, std::placeholders::_1);

  astar_client_->async_send_goal(goal_msg, opts);
  return true;
}

void Orchestrator::advancePathIndex()
{
  if (current_path_.poses.empty()) {
    return;
  }
  const std::size_t search_end =
    std::min(current_path_.poses.size(), path_index_ + 50);
  double min_dist = std::numeric_limits<double>::max();
  for (std::size_t i = path_index_; i < search_end; ++i) {
    const double dx =
      current_pose_.pose.position.x - current_path_.poses[i].pose.position.x;
    const double dy =
      current_pose_.pose.position.y - current_path_.poses[i].pose.position.y;
    const double d = dx * dx + dy * dy;
    if (d < min_dist) {
      min_dist = d;
      path_index_ = i;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Callbacks
// ═══════════════════════════════════════════════════════════════════════════════

void Orchestrator::goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_goal_ = *msg;
  path_index_ = 0;
  if (!send_astar_goal(current_goal_)) {
    RCLCPP_WARN(get_logger(), "Dropping goal — A* not available");
    return;
  }
  transition(State::PLANNING);
}

void Orchestrator::poseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  if (state_ == State::NAVIGATING) {
    advancePathIndex();
  }
}

void Orchestrator::scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(
      "map",
      msg->header.frame_id,
      msg->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Could not get laser→map transform: %s", ex.what());
    return;
  }
  local_map_.updateFromScan(*msg, tf);
}

void Orchestrator::isNavigatingCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  is_navigating_ = msg->data;
}

void Orchestrator::goalResponseCallback(GoalHandleAStar::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "A* goal rejected — returning to IDLE");
    transition(State::IDLE);
  }
}

void Orchestrator::goalResultCallback(const GoalHandleAStar::WrappedResult & result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      !result.result->success ||
      result.result->path.poses.empty())
  {
    RCLCPP_WARN(get_logger(), "A* planning failed — returning to IDLE");
    transition(State::IDLE);
    return;
  }

  current_path_ = result.result->path;
  path_index_ = 0;
  lqr_started_ = false;
  transition(State::NAVIGATING);
}

}  // namespace paesano_orchestrator
