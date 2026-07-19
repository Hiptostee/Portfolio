#ifndef PAESANO_MAPPING__MAPPING_POSE_PUBLISHER_HPP_
#define PAESANO_MAPPING__MAPPING_POSE_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace paesano_mapping
{

class MappingPosePublisher final : public rclcpp::Node
{
public:
  explicit MappingPosePublisher(const rclcpp::NodeOptions & options);

private:
  void publishPose();

  std::string map_frame_;
  std::string base_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace paesano_mapping

#endif  // PAESANO_MAPPING__MAPPING_POSE_PUBLISHER_HPP_
