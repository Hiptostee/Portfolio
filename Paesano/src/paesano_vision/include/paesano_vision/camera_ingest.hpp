#ifndef PAESANO_VISION__CAMERA_INGEST_HPP_
#define PAESANO_VISION__CAMERA_INGEST_HPP_

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace paesano_vision
{

class CameraIngest final : public rclcpp::Node
{
public:
  explicit CameraIngest(const rclcpp::NodeOptions & options);

private:
  void handleColor(const sensor_msgs::msg::Image::SharedPtr msg);
  void handleDepth(const sensor_msgs::msg::Image::SharedPtr msg);
  void handleCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void reportStreams();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::uint64_t color_frames_{0};
  std::uint64_t depth_frames_{0};
  std::uint64_t previous_color_frames_{0};
  std::uint64_t previous_depth_frames_{0};
  bool have_camera_info_{false};
  std::uint32_t color_width_{0};
  std::uint32_t color_height_{0};
  std::uint32_t depth_width_{0};
  std::uint32_t depth_height_{0};
  std::string color_encoding_;
  std::string depth_encoding_;
};

}  // namespace paesano_vision

#endif  // PAESANO_VISION__CAMERA_INGEST_HPP_
