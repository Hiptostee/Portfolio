#include "paesano_explorer/explorer_node.hpp"

#include <chrono>
#include <functional>
#include <utility>

#include "rclcpp_components/register_node_macro.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace paesano_explorer
{

ExplorerNode::ExplorerNode(const rclcpp::NodeOptions & options)
: Node("explorer_node", options)
{
  const std::string map_topic = declare_parameter<std::string>("map_topic", "/map");
  const std::string pose_topic = declare_parameter<std::string>("pose_topic", "/estimated_pose");
  const std::string navigation_goal_topic =
    declare_parameter<std::string>("navigation_goal_topic", "/navigation/goal");
  const std::string navigation_result_topic =
    declare_parameter<std::string>("navigation_result_topic", "/navigation/result");
  const std::string state_topic =
    declare_parameter<std::string>("state_topic", "/exploration/state");
  const std::string marker_topic =
    declare_parameter<std::string>("marker_topic", "/exploration/frontiers");
  free_threshold_ = declare_parameter<int>("free_threshold", free_threshold_);
  minimum_cluster_size_ =
    declare_parameter<int>("minimum_cluster_size", minimum_cluster_size_);
  completion_confirmation_updates_ = declare_parameter<int>(
    "completion_confirmation_updates", completion_confirmation_updates_);
  selection_parameters_.goal_standoff_m = declare_parameter<double>(
    "goal_standoff_m", selection_parameters_.goal_standoff_m);
  selection_parameters_.blacklist_radius_m = declare_parameter<double>(
    "blacklist_radius_m", selection_parameters_.blacklist_radius_m);
  selection_parameters_.information_gain_weight = declare_parameter<double>(
    "information_gain_weight", selection_parameters_.information_gain_weight);
  selection_parameters_.distance_weight = declare_parameter<double>(
    "distance_weight", selection_parameters_.distance_weight);
  selection_parameters_.occupied_threshold = declare_parameter<int>(
    "occupied_threshold", selection_parameters_.occupied_threshold);
  const int tick_period_ms = declare_parameter<int>("tick_period_ms", 500);

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic,
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&ExplorerNode::handleMap, this, std::placeholders::_1));
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic,
    10,
    std::bind(&ExplorerNode::handlePose, this, std::placeholders::_1));
  navigation_result_sub_ = create_subscription<std_msgs::msg::String>(
    navigation_result_topic,
    10,
    std::bind(&ExplorerNode::handleNavigationResult, this, std::placeholders::_1));

  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(navigation_goal_topic, 10);
  state_pub_ = create_publisher<std_msgs::msg::String>(state_topic, 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(tick_period_ms),
    std::bind(&ExplorerNode::tick, this));

  RCLCPP_INFO(get_logger(), "Explorer component ready");
}

void ExplorerNode::handleMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = *msg;
  have_map_ = true;
  ++map_revision_;
}

void ExplorerNode::handlePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  robot_pose_ = *msg;
  have_pose_ = true;
}

void ExplorerNode::handleNavigationResult(const std_msgs::msg::String::SharedPtr msg)
{
  if (state_ != State::NAVIGATING || !active_goal_) {
    return;
  }

  if (msg->data == "SUCCEEDED") {
    active_goal_.reset();
    state_ = State::SELECTING;
    empty_frontier_updates_ = 0;
    return;
  }

  if (msg->data == "PLANNING_FAILED") {
    failed_goals_.push_back({
      active_goal_->pose.pose.position.x,
      active_goal_->pose.pose.position.y});
    active_goal_.reset();
    state_ = State::SELECTING;
  }
}

void ExplorerNode::tick()
{
  std_msgs::msg::String state_message;
  state_message.data = stateName();
  state_pub_->publish(state_message);

  if (!have_map_ || !have_pose_) {
    state_ = State::WAITING_FOR_DATA;
    return;
  }

  if (state_ == State::WAITING_FOR_DATA) {
    state_ = State::SELECTING;
  }

  if (state_ != State::SELECTING || map_revision_ == last_processed_map_revision_) {
    return;
  }
  last_processed_map_revision_ = map_revision_;

  const auto frontier_cells = detector_.detect(map_, free_threshold_);
  const auto clusters = clusterer_.cluster(
    frontier_cells,
    static_cast<int>(map_.info.width),
    static_cast<int>(map_.info.height),
    static_cast<std::size_t>(minimum_cluster_size_));
  const auto goal = selector_.select(
    map_, clusters, robot_pose_, failed_goals_, selection_parameters_);
  publishMarkers(frontier_cells, goal);

  if (!goal) {
    if (!exploration_started_) {
      return;
    }
    ++empty_frontier_updates_;
    if (empty_frontier_updates_ >= completion_confirmation_updates_) {
      state_ = State::COMPLETE;
      RCLCPP_INFO(get_logger(), "Exploration complete: no valid frontier goals remain");
    }
    return;
  }

  empty_frontier_updates_ = 0;
  publishGoal(*goal);
}

void ExplorerNode::publishGoal(const FrontierGoal & goal)
{
  exploration_started_ = true;
  active_goal_ = goal;
  goal_pub_->publish(goal.pose);
  state_ = State::NAVIGATING;
  RCLCPP_INFO(
    get_logger(),
    "Selected frontier goal (%.3f, %.3f), cluster_size=%zu, score=%.3f",
    goal.pose.pose.position.x,
    goal.pose.pose.position.y,
    goal.cluster_size,
    goal.score);
}

void ExplorerNode::publishMarkers(
  const std::vector<GridCell> & cells,
  const std::optional<FrontierGoal> & selected_goal)
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker frontier_marker;
  frontier_marker.header = map_.header;
  frontier_marker.header.stamp = now();
  frontier_marker.ns = "frontier_cells";
  frontier_marker.id = 0;
  frontier_marker.type = visualization_msgs::msg::Marker::POINTS;
  frontier_marker.action = visualization_msgs::msg::Marker::ADD;
  frontier_marker.scale.x = map_.info.resolution;
  frontier_marker.scale.y = map_.info.resolution;
  frontier_marker.color.r = 0.0F;
  frontier_marker.color.g = 0.8F;
  frontier_marker.color.b = 1.0F;
  frontier_marker.color.a = 1.0F;

  for (const auto & cell : cells) {
    geometry_msgs::msg::Point point;
    point.x = map_.info.origin.position.x +
      (static_cast<double>(cell.x) + 0.5) * map_.info.resolution;
    point.y = map_.info.origin.position.y +
      (static_cast<double>(cell.y) + 0.5) * map_.info.resolution;
    frontier_marker.points.push_back(point);
  }
  markers.markers.push_back(std::move(frontier_marker));

  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header = map_.header;
  goal_marker.header.stamp = now();
  goal_marker.ns = "selected_frontier";
  goal_marker.id = 1;
  goal_marker.type = visualization_msgs::msg::Marker::ARROW;
  goal_marker.action = selected_goal ?
    visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  goal_marker.scale.x = 0.35;
  goal_marker.scale.y = 0.08;
  goal_marker.scale.z = 0.08;
  goal_marker.color.r = 1.0F;
  goal_marker.color.g = 0.2F;
  goal_marker.color.b = 0.0F;
  goal_marker.color.a = 1.0F;
  if (selected_goal) {
    goal_marker.pose = selected_goal->pose.pose;
  }
  markers.markers.push_back(std::move(goal_marker));

  marker_pub_->publish(markers);
}

std::string ExplorerNode::stateName() const
{
  switch (state_) {
    case State::WAITING_FOR_DATA:
      return "WAITING_FOR_DATA";
    case State::SELECTING:
      return "SELECTING";
    case State::NAVIGATING:
      return "NAVIGATING";
    case State::COMPLETE:
      return "COMPLETE";
  }
  return "UNKNOWN";
}

}  // namespace paesano_explorer

RCLCPP_COMPONENTS_REGISTER_NODE(paesano_explorer::ExplorerNode)
