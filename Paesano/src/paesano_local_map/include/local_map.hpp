#ifndef PAESANO_LOCAL_MAP__LOCAL_MAP_HPP_
#define PAESANO_LOCAL_MAP__LOCAL_MAP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <string>

namespace paesano_local_map
{

class LocalMapNode : public rclcpp::Node
{
public:
  explicit LocalMapNode(const rclcpp::NodeOptions & options);

private:
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const;
  nav_msgs::msg::OccupancyGrid makeEmptyLocalMap(const rclcpp::Time & stamp) const;
  bool localToCell(double x, double y, int & cell_x, int & cell_y) const;
  void markCell(nav_msgs::msg::OccupancyGrid & map, int cell_x, int cell_y, int8_t value) const;
  void markFreeRay(
    nav_msgs::msg::OccupancyGrid & map,
    double end_x,
    double end_y) const;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  std::string pose_topic_{"/estimated_pose"};
  std::string scan_topic_{"/scan"};
  std::string map_topic_{"/local_map"};

  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;

  double map_size_m_{2.0};
  double map_resolution_{0.05};
  double scan_yaw_offset_{-1.57079632679};
  int map_width_{40};
  int map_height_{40};
};

}  // namespace paesano_local_map

#endif  // PAESANO_LOCAL_MAP__LOCAL_MAP_HPP_
