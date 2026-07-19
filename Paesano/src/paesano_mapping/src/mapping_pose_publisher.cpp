#include "paesano_mapping/mapping_pose_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"

namespace paesano_mapping
{

MappingPosePublisher::MappingPosePublisher(const rclcpp::NodeOptions & options)
: Node("mapping_pose_publisher", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  const std::string pose_topic =
    declare_parameter<std::string>("pose_topic", "/estimated_pose");
  const double publish_rate_hz = std::max(
    1.0, declare_parameter<double>("publish_rate_hz", 20.0));

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz),
    std::bind(&MappingPosePublisher::publishPose, this));
}

void MappingPosePublisher::publishPose()
{
  try {
    const auto transform = tf_buffer_.lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    pose_pub_->publish(pose);
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Waiting for %s -> %s transform: %s",
      map_frame_.c_str(), base_frame_.c_str(), error.what());
  }
}

}  // namespace paesano_mapping

RCLCPP_COMPONENTS_REGISTER_NODE(paesano_mapping::MappingPosePublisher)
