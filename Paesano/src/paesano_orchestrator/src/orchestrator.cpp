#include "paesano_orchestrator/orchestrator.hpp"

#include <chrono>
#include <cmath>

namespace paesano_orchestrator
{

Orchestrator::Orchestrator()
: Node("orchestrator_node")
{
  delay_ = declare_parameter<double>("blocked_replan_delay_sec", delay_);
  occupied_threshold_ = declare_parameter<int>("occupied_threshold", occupied_threshold_);
  goal_tolerance_ = declare_parameter<double>("goal_tolerance_m", goal_tolerance_);

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/navigation/goal",
    10,
    std::bind(&Orchestrator::handleGoal, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/estimated_pose",
    10,
    std::bind(&Orchestrator::handlePose, this, std::placeholders::_1));
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&Orchestrator::handleMap, this, std::placeholders::_1));
  local_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_map",
    10,
    std::bind(&Orchestrator::handleLocalMap, this, std::placeholders::_1));
  blocked_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dynamic_obstacle_blocked",
    10,
    std::bind(&Orchestrator::handleBlocked, this, std::placeholders::_1));

  planning_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/planning_map",
    rclcpp::QoS(1).transient_local().reliable());
  state_pub_ = create_publisher<std_msgs::msg::String>("/orchestrator/state", 10);
  client_ = rclcpp_action::create_client<AStar>(this, "/a_star");
  stop_client_ = create_client<std_srvs::srv::Trigger>("/lqr/stop");
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&Orchestrator::tick, this));
}

void Orchestrator::handleGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_ = *msg;
  have_goal_ = true;
  plan_in_flight_ = false;
  state_ = State::PLANNING;
  publishState();
}

void Orchestrator::handlePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  pose_ = *msg;
  have_pose_ = true;
}

void Orchestrator::handleMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = *msg;
  have_map_ = true;
  publishPlanningMap();
}

void Orchestrator::handleLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  local_map_ = *msg;
  have_local_map_ = true;
  publishPlanningMap();
}

void Orchestrator::handleBlocked(const std_msgs::msg::Bool::SharedPtr msg)
{
  blocked_ = msg->data;
}

void Orchestrator::publishPlanningMap()
{
  if (!have_map_) {
    return;
  }

  auto merged = map_;
  if (have_local_map_ && local_map_.header.frame_id == map_.header.frame_id) {
    for (uint32_t y = 0; y < local_map_.info.height; ++y) {
      for (uint32_t x = 0; x < local_map_.info.width; ++x) {
        const std::size_t local_index = y * local_map_.info.width + x;
        if (local_map_.data[local_index] < occupied_threshold_) {
          continue;
        }

        const auto & origin = local_map_.info.origin;
        const double world_x =
          origin.position.x + (static_cast<double>(x) + 0.5) * local_map_.info.resolution;
        const double world_y =
          origin.position.y + (static_cast<double>(y) + 0.5) * local_map_.info.resolution;
        const int map_x = static_cast<int>(std::floor(
          (world_x - map_.info.origin.position.x) / map_.info.resolution));
        const int map_y = static_cast<int>(std::floor(
          (world_y - map_.info.origin.position.y) / map_.info.resolution));

        if (
          map_x >= 0 && map_y >= 0 &&
          map_x < static_cast<int>(map_.info.width) &&
          map_y < static_cast<int>(map_.info.height))
        {
          const std::size_t map_index = map_y * map_.info.width + map_x;
          merged.data[map_index] = 100;
        }
      }
    }
  }

  merged.header.stamp = now();
  planning_pub_->publish(merged);
}

void Orchestrator::requestPlan()
{
  if (
    plan_in_flight_ || !have_goal_ || !have_pose_ || !have_map_ ||
    !client_->wait_for_action_server(std::chrono::seconds(1)))
  {
    return;
  }

  publishPlanningMap();

  auto goal = std::make_shared<AStar::Goal>();
  goal->goal = goal_;
  plan_in_flight_ = true;

  rclcpp_action::Client<AStar>::SendGoalOptions options;
  options.result_callback = [this](const auto & result) {
    plan_in_flight_ = false;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
      state_ = State::NAVIGATING;
    } else {
      state_ = blocked_ ? State::WAITING : State::IDLE;
    }
    publishState();
  };
  client_->async_send_goal(*goal, options);
}

void Orchestrator::publishState()
{
  std_msgs::msg::String msg;
  msg.data = stateName();
  state_pub_->publish(msg);
}

std::string Orchestrator::stateName() const
{
  switch (state_) {
    case State::IDLE:
      return "IDLE";
    case State::PLANNING:
      return "PLANNING";
    case State::NAVIGATING:
      return "NAVIGATING";
    case State::WAITING:
      return "WAITING";
    case State::REPLANNING:
      return "REPLANNING";
  }
  return "UNKNOWN";
}

}  // namespace paesano_orchestrator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<paesano_orchestrator::Orchestrator>());
  rclcpp::shutdown();
  return 0;
}
