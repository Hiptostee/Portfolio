#include "paesano_vision/camera_ingest.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

#include "rclcpp_components/register_node_macro.hpp"

namespace paesano_vision
{

CameraIngest::CameraIngest(const rclcpp::NodeOptions & options)
: Node("camera_ingest", options)
{
  const auto color_topic = declare_parameter<std::string>(
    "color_topic", "/camera/camera/color/image_raw");
  const auto depth_topic = declare_parameter<std::string>(
    "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
  const auto camera_info_topic = declare_parameter<std::string>(
    "camera_info_topic", "/camera/camera/color/camera_info");
  const std::int64_t report_period_ms = std::max<std::int64_t>(
    1, declare_parameter<int>("report_period_ms", 5000));

  color_sub_ = create_subscription<sensor_msgs::msg::Image>(
    color_topic, rclcpp::SensorDataQoS(),
    std::bind(&CameraIngest::handleColor, this, std::placeholders::_1));
  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    depth_topic, rclcpp::SensorDataQoS(),
    std::bind(&CameraIngest::handleDepth, this, std::placeholders::_1));
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic, rclcpp::SensorDataQoS(),
    std::bind(&CameraIngest::handleCameraInfo, this, std::placeholders::_1));
  report_timer_ = create_wall_timer(
    std::chrono::milliseconds(report_period_ms),
    std::bind(&CameraIngest::reportStreams, this));

  RCLCPP_INFO(
    get_logger(), "Listening for color='%s', aligned depth='%s', camera info='%s'",
    color_topic.c_str(), depth_topic.c_str(), camera_info_topic.c_str());
}

void CameraIngest::handleColor(const sensor_msgs::msg::Image::SharedPtr msg)
{
  ++color_frames_;
  color_width_ = msg->width;
  color_height_ = msg->height;
  color_encoding_ = msg->encoding;
}

void CameraIngest::handleDepth(const sensor_msgs::msg::Image::SharedPtr msg)
{
  ++depth_frames_;
  depth_width_ = msg->width;
  depth_height_ = msg->height;
  depth_encoding_ = msg->encoding;
}

void CameraIngest::handleCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  have_camera_info_ = msg->width > 0 && msg->height > 0 && msg->k[0] > 0.0 && msg->k[4] > 0.0;
}

void CameraIngest::reportStreams()
{
  const auto new_color_frames = color_frames_ - previous_color_frames_;
  const auto new_depth_frames = depth_frames_ - previous_depth_frames_;
  previous_color_frames_ = color_frames_;
  previous_depth_frames_ = depth_frames_;

  if (new_color_frames == 0 || new_depth_frames == 0 || !have_camera_info_) {
    RCLCPP_WARN(
      get_logger(), "Camera waiting: color=%lu depth=%lu camera_info=%s",
      static_cast<unsigned long>(new_color_frames),
      static_cast<unsigned long>(new_depth_frames),
      have_camera_info_ ? "yes" : "no");
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Camera receiving: color=%ux%u %s (%lu frames), depth=%ux%u %s (%lu frames), camera_info=yes",
    color_width_, color_height_, color_encoding_.c_str(),
    static_cast<unsigned long>(new_color_frames),
    depth_width_, depth_height_, depth_encoding_.c_str(),
    static_cast<unsigned long>(new_depth_frames));
}

}  // namespace paesano_vision

RCLCPP_COMPONENTS_REGISTER_NODE(paesano_vision::CameraIngest)
