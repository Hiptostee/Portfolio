#include "lqr.hpp"
#include <chrono>
#include <functional>
#include "rclcpp_components/register_node_macro.hpp"

namespace lqr
{

LQR::LQR(const rclcpp::NodeOptions &options)
    : rclcpp::Node("lqr_node", options)
{
  estimated_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/estimated_pose", 10, std::bind(&LQR::estimatedPoseCallback, this, std::placeholders::_1));

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
  "/path",
  rclcpp::QoS(1).transient_local().reliable(), 
  std::bind(&LQR::pathCallback, this, std::placeholders::_1));

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  nav_status_pub_ = create_publisher<std_msgs::msg::Bool>("/is_navigating", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&LQR::lqrLoop, this));
  
  RCLCPP_INFO(get_logger(), "LQR Trajectory Node Initialized. Ensure /estimated_pose and /path are active.");
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
    Matrix3d B = Matrix3d::Identity() * 0.02; // Matches your 50Hz motor control loop [cite: 86, 145]
    Matrix3d Q = Matrix3d::Zero();
    Q(0,0)=50.0; Q(1,1)=75.0; Q(2,2)=5.0; // Heavy penalty on X/Y error
    Matrix3d R = Matrix3d::Identity() * 0.5; 

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
  
  // Look-ahead of 8 points (~160ms ahead) to ensure smooth mecanum movement [cite: 33, 230]
  size_t target_idx = std::min(current_path_index_ + 1, current_path_.size() - 1);
  const Pose target_pose = current_path_[target_idx];

  const Vector3d error = calculateError(current_pose, target_pose);
  const Vector3d control = K * error; // Command = Gain * Error

  // Publish velocity targets to the Pico controller [cite: 30, 87]
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = std::clamp(control(0), -0.6, 0.6);
  cmd.linear.y = std::clamp(control(1), -0.6, 0.6);
  cmd.angular.z = std::clamp(control(2), -0.6, 0.6);

  // Stop if at the end of the path
  double dist_to_end = std::hypot(current_path_.back().x - current_pose.x, current_path_.back().y - current_pose.y);
  if (current_path_index_ >= current_path_.size() - 1 && dist_to_end < 0.08) {
    RCLCPP_INFO(get_logger(), "Path Complete. Stopping.");
    publishZeroVelocity();
    have_path_ = false;
  } else {
    cmd_pub_->publish(cmd);
  }
}

void LQR::publishZeroVelocity() {
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
}

void LQR::estimatedPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  current_estimated_pose_msg_ = *msg;
  have_estimated_pose_ = true;
}

void LQR::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
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