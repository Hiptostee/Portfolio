#include "slambot_traj_following/lqr.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

#include "rclcpp_components/register_node_macro.hpp"

namespace lqr
{

LQR::LQR(const rclcpp::NodeOptions & options)
    : rclcpp::Node("lqr", options)
{
  const std::string estimated_pose_topic =
    declare_parameter<std::string>("estimated_pose_topic", "/estimated_pose");
  const std::string path_topic =
    declare_parameter<std::string>("path_topic", "/path");
  const std::string cmd_vel_topic =
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  const std::string navigation_state_topic =
    declare_parameter<std::string>("navigation_state_topic", "/is_navigating");

  control_period_ms_ = declare_parameter<int>("control_period_ms", 20);
  lookahead_points_ = declare_parameter<int>("lookahead_points", 1);
  max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 0.6);
  max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 0.6);
  path_complete_tolerance_ = declare_parameter<double>("path_complete_tolerance", 0.08);
  q_x_ = declare_parameter<double>("q_x", 50.0);
  q_y_ = declare_parameter<double>("q_y", 75.0);
  q_theta_ = declare_parameter<double>("q_theta", 5.0);
  r_weight_ = declare_parameter<double>("r_weight", 0.5);

  estimated_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    estimated_pose_topic, 10, std::bind(&LQR::estimatedPoseCallback, this, std::placeholders::_1));

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    path_topic,
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&LQR::pathCallback, this, std::placeholders::_1));

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

  nav_status_pub_ = create_publisher<std_msgs::msg::Bool>(navigation_state_topic, 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(control_period_ms_),
    std::bind(&LQR::lqrLoop, this));

  RCLCPP_INFO(
    get_logger(),
    "LQR node ready: estimated_pose_topic='%s', path_topic='%s', cmd_vel_topic='%s'",
    estimated_pose_topic.c_str(),
    path_topic.c_str(),
    cmd_vel_topic.c_str());
}

void LQR::lqrLoop()
{
  // --- Gating Logic with Debug Info ---
  if (!have_estimated_pose_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Stalled: Waiting for /estimated_pose");
    return;
  }
  if (!have_path_ || current_path_.empty()) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Stalled: Waiting for /path");
    return;
  }

  // Calculate Gain K once
  if (!initialized_) {
    Matrix3d A = Matrix3d::Identity();
    Matrix3d B = Matrix3d::Identity() * (static_cast<double>(control_period_ms_) / 1000.0);
    Matrix3d Q = Matrix3d::Zero();
    Q(0, 0) = q_x_;
    Q(1, 1) = q_y_;
    Q(2, 2) = q_theta_;
    Matrix3d R = Matrix3d::Identity() * r_weight_;

    K = solveDare(A, B, Q, R);
    initialized_ = true;
    RCLCPP_INFO(get_logger(), "LQR Gain Matrix K calculated successfully.");
  }

  auto nav_msg = std_msgs::msg::Bool();
  nav_msg.data = have_path_ && !current_path_.empty();
  nav_status_pub_->publish(nav_msg);

  const Pose current_pose = poseStampedToPose(current_estimated_pose_msg_);
  
  // Update Path Tracking
  current_path_index_ = findClosestIndex(current_pose, current_path_, current_path_index_);
  
  const std::size_t target_idx = std::min(
    current_path_index_ + static_cast<std::size_t>(std::max(1, lookahead_points_)),
    current_path_.size() - 1);
  const Pose target_pose = current_path_[target_idx];

  const Vector3d error = calculateError(current_pose, target_pose);
  const Vector3d control = K * error; // Command = Gain * Error

  // Publish velocity targets to the Pico controller [cite: 30, 87]
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::clamp(control(0), -max_linear_velocity_, max_linear_velocity_);
  cmd.linear.y = std::clamp(control(1), -max_linear_velocity_, max_linear_velocity_);
  cmd.angular.z = std::clamp(control(2), -max_angular_velocity_, max_angular_velocity_);

  // Stop if at the end of the path
  const double dist_to_end = std::hypot(
    current_path_.back().x - current_pose.x,
    current_path_.back().y - current_pose.y);
  if (current_path_index_ >= current_path_.size() - 1 && dist_to_end < path_complete_tolerance_) {
    RCLCPP_INFO(get_logger(), "Path Complete. Stopping.");
    publishZeroVelocity();
    have_path_ = false;
  } else {
    cmd_pub_->publish(cmd);
  }
}

void LQR::publishZeroVelocity()
{
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
}

void LQR::estimatedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_estimated_pose_msg_ = *msg;
  have_estimated_pose_ = true;
}

void LQR::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_.clear();
  for (const auto & p : msg->poses) {
    current_path_.push_back(poseStampedToPose(p));
  }
  // Reset index and force a global search in the next loop iteration
  current_path_index_ = 0; 
  have_path_ = !current_path_.empty();
  RCLCPP_INFO(get_logger(), "New Path Received: %zu points. Starting tracking.", current_path_.size());
}

} // namespace lqr
RCLCPP_COMPONENTS_REGISTER_NODE(lqr::LQR)
