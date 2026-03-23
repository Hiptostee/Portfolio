#include <functional>
#include <thread>

#include "rclcpp_components/register_node_macro.hpp"
#include "slambot_navigation/a_star_action_server.hpp"

namespace slambot_navigation
{

AStarActionServer::AStarActionServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("a_star_action_server", options)
{
  const double obstacle_buffer_m =
    declare_parameter<double>("obstacle_buffer_m", 0.3);
  const std::string path_topic =
    declare_parameter<std::string>("path_topic", "/planned_path");
  const std::string pose_topic =
    declare_parameter<std::string>("pose_topic", "/estimated_pose");
  planner_.setObstacleBufferMeters(obstacle_buffer_m);

  action_server_ = rclcpp_action::create_server<AStarAction>(
    this,
    "a_star",
    std::bind(&AStarActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&AStarActionServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&AStarActionServer::handle_accepted, this, std::placeholders::_1));
  map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&AStarActionServer::handleMap, this, std::placeholders::_1));
  pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic,
    10,
    std::bind(&AStarActionServer::handlePose, this, std::placeholders::_1));
  path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic, 10);
}

rclcpp_action::GoalResponse AStarActionServer::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const AStarAction::Goal> goal)
{
  RCLCPP_INFO(
    get_logger(),
    "Received a_star goal: goal=(%.3f, %.3f), obstacle_buffer_m=%.3f",
    goal->goal.pose.position.x,
    goal->goal.pose.position.y,
    get_parameter("obstacle_buffer_m").as_double());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AStarActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleAStar>)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel a_star goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AStarActionServer::handleMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  have_map_ = true;
  RCLCPP_INFO(
    get_logger(),
    "Received map for a_star planner: %u x %u at %.3f m/cell",
    current_map_.info.width,
    current_map_.info.height,
    current_map_.info.resolution);
}

void AStarActionServer::handlePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  have_pose_ = true;
}

void AStarActionServer::handle_accepted(const std::shared_ptr<GoalHandleAStar> goal_handle)
{
  std::thread(
    std::bind(&AStarActionServer::execute, this, std::placeholders::_1),
    goal_handle).detach();
}

void AStarActionServer::execute(const std::shared_ptr<GoalHandleAStar> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<AStarAction::Feedback>();
  feedback->status = "Planning request received";
  goal_handle->publish_feedback(feedback);

  if (!have_map_) {
    feedback->status = "No map received yet";
    goal_handle->publish_feedback(feedback);

    auto result = std::make_shared<AStarAction::Result>();
    result->success = false;
    goal_handle->abort(result);
    RCLCPP_WARN(get_logger(), "Rejecting a_star execution because no map has been received yet");
    return;
  }

  if (!have_pose_) {
    feedback->status = "No current pose received yet";
    goal_handle->publish_feedback(feedback);

    auto result = std::make_shared<AStarAction::Result>();
    result->success = false;
    goal_handle->abort(result);
    RCLCPP_WARN(get_logger(), "Rejecting a_star execution because no current pose has been received yet");
    return;
  }

  planner_.setMap(current_map_);

  auto result = std::make_shared<AStarAction::Result>();
  result->path = planner_.plan(current_pose_, goal->goal);
  result->success = !result->path.poses.empty();

  if (result->success) {
    path_publisher_->publish(result->path);
  }

  goal_handle->succeed(result);

  RCLCPP_INFO(
    get_logger(),
    "Completed a_star goal with %zu poses in frame '%s' and published to '%s'",
    result->path.poses.size(),
    result->path.header.frame_id.c_str(),
    get_parameter("path_topic").as_string().c_str());
}

}  // namespace slambot_navigation

RCLCPP_COMPONENTS_REGISTER_NODE(slambot_navigation::AStarActionServer)
