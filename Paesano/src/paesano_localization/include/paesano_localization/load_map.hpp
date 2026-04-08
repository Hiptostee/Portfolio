#ifndef PAESANO_LOCALIZATION_LOAD_MAP_HPP
#define PAESANO_LOCALIZATION_LOAD_MAP_HPP

#include <nav2_msgs/srv/load_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace paesano_localization
{

class MapLoaderService : public rclcpp::Node
{
public:
  explicit MapLoaderService(const rclcpp::NodeOptions &options);

private:
  using LoadMap = nav2_msgs::srv::LoadMap;

  void handleLoadMap(
      const std::shared_ptr<LoadMap::Request> request,
      std::shared_ptr<LoadMap::Response> response);

  std::string map_server_service_name_;

  rclcpp::Service<LoadMap>::SharedPtr load_map_service_;
  rclcpp::Service<LoadMap>::SharedPtr load_map_private_service_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
};

} // namespace paesano_localization

#endif // PAESANO_LOCALIZATION_LOAD_MAP_HPP
