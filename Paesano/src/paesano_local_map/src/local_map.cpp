#include "local_map.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include <rclcpp_components/register_node_macro.hpp>

namespace paesano_local_map
{

LocalMapNode::LocalMapNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("local_map_node", options)
{
  scan_topic_ = declare_parameter<std::string>("scan_topic", scan_topic_);
  pose_topic_ = declare_parameter<std::string>("pose_topic", pose_topic_);
  map_topic_ = declare_parameter<std::string>("map_topic", map_topic_);
  map_size_m_ = std::max(0.1, declare_parameter<double>("map_size_m", map_size_m_));
  map_resolution_ = std::max(0.01, declare_parameter<double>("map_resolution", map_resolution_));
  scan_yaw_offset_ = declare_parameter<double>("scan_yaw_offset", scan_yaw_offset_);
  map_width_ = std::max(1, static_cast<int>(std::round(map_size_m_ / map_resolution_)));
  map_height_ = map_width_;

  lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    10,
    std::bind(&LocalMapNode::lidarCallback, this, std::placeholders::_1));
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, 10);
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic_, 
    10, 
    std::bind(&LocalMapNode::poseCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Local map ready: size=%.2fm resolution=%.3fm cells=%dx%d scan='%s' pose='%s' yaw_offset=%.3f map='%s'",
    map_size_m_,
    map_resolution_,
    map_width_,
    map_height_,
    scan_topic_.c_str(),
    pose_topic_.c_str(),
    scan_yaw_offset_,
    map_topic_.c_str());
}

void LocalMapNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = msg;
}

double LocalMapNode::getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

nav_msgs::msg::OccupancyGrid LocalMapNode::makeEmptyLocalMap(const rclcpp::Time & stamp) const
{
  auto local_map = nav_msgs::msg::OccupancyGrid();
  local_map.header.stamp = stamp;
  local_map.header.frame_id = current_pose_->header.frame_id;

  local_map.info.resolution = map_resolution_;
  local_map.info.width = static_cast<uint32_t>(map_width_);
  local_map.info.height = static_cast<uint32_t>(map_height_);

  const double half_size = map_size_m_ / 2.0;
  local_map.info.origin.orientation.w = 1.0;
  local_map.info.origin.position.x = current_pose_->pose.position.x - half_size;
  local_map.info.origin.position.y = current_pose_->pose.position.y - half_size;
  local_map.info.origin.position.z = current_pose_->pose.position.z;

  local_map.data.assign(static_cast<std::size_t>(map_width_ * map_height_), -1);
  return local_map;
}

bool LocalMapNode::localToCell(double x, double y, int & cell_x, int & cell_y) const
{
  const double half_size = map_size_m_ / 2.0;
  cell_x = static_cast<int>(std::floor((x + half_size) / map_resolution_));
  cell_y = static_cast<int>(std::floor((y + half_size) / map_resolution_));
  return cell_x >= 0 && cell_x < map_width_ && cell_y >= 0 && cell_y < map_height_;
}

void LocalMapNode::markCell(
  nav_msgs::msg::OccupancyGrid & map,
  int cell_x,
  int cell_y,
  int8_t value) const
{
  if (cell_x < 0 || cell_x >= map_width_ || cell_y < 0 || cell_y >= map_height_) {
    return;
  }

  const std::size_t index = static_cast<std::size_t>(cell_y * map_width_ + cell_x);
  map.data[index] = value;
}

void LocalMapNode::markFreeRay(
  nav_msgs::msg::OccupancyGrid & map,
  double end_x,
  double end_y) const
{
  const double ray_length = std::hypot(end_x, end_y);
  if (ray_length <= std::numeric_limits<double>::epsilon()) {
    return;
  }

  const int steps = std::max(1, static_cast<int>(std::ceil(ray_length / map_resolution_)));
  for (int step = 0; step < steps; ++step) {
    const double t = static_cast<double>(step) / static_cast<double>(steps);
    int cell_x;
    int cell_y;
    if (localToCell(end_x * t, end_y * t, cell_x, cell_y)) {
      markCell(map, cell_x, cell_y, 0);
    }
  }
}

void LocalMapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!current_pose_) {
    return;
  }

  auto local_map = makeEmptyLocalMap(msg->header.stamp);
  const double half_diagonal = std::hypot(map_size_m_ / 2.0, map_size_m_ / 2.0);
  const double robot_yaw = getYawFromQuaternion(current_pose_->pose.orientation);

  double angle = msg->angle_min;
  for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
    const double range = msg->ranges[i];

    if (std::isnan(range) || range < msg->range_min) {
      angle += msg->angle_increment;
      continue;
    }

    const bool hit_obstacle = std::isfinite(range) && range <= msg->range_max;
    const double ray_range = hit_obstacle ? std::min(range, half_diagonal) : half_diagonal;
    const double map_relative_angle = angle + scan_yaw_offset_ + robot_yaw;
    const double local_x = ray_range * std::cos(map_relative_angle);
    const double local_y = ray_range * std::sin(map_relative_angle);

    markFreeRay(local_map, local_x, local_y);

    if (hit_obstacle) {
      int cell_x;
      int cell_y;
      if (localToCell(local_x, local_y, cell_x, cell_y)) {
        markCell(local_map, cell_x, cell_y, 100);
      }
    }

    angle += msg->angle_increment;
  }

  map_pub_->publish(local_map);
}

}  // namespace paesano_local_map

RCLCPP_COMPONENTS_REGISTER_NODE(paesano_local_map::LocalMapNode)
