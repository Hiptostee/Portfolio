#include "load_map.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>

namespace slambot_localization
{

MapLoaderService::MapLoaderService(const rclcpp::NodeOptions &options)
    : rclcpp::Node("map_loader_service", options)
{
  map_server_service_name_ =
      declare_parameter<std::string>("map_server_load_service", "/map_server/load_map");
  const auto local_service_name =
      declare_parameter<std::string>("load_map_service", "~/load_map");
  const auto map_topic =
      declare_parameter<std::string>("map_topic", "/map");
  const auto autoload_on_startup =
      declare_parameter<bool>("autoload_on_startup", false);
  autoload_map_url_ =
      declare_parameter<std::string>("autoload_map_url", "");
  autoload_retries_left_ =
      declare_parameter<int>("autoload_retries", 10);

  map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(1).transient_local().reliable());
  load_map_service_ = create_service<LoadMap>(
      local_service_name,
      std::bind(
          &MapLoaderService::handleLoadMap,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

  RCLCPP_INFO(
      get_logger(),
      "Map loader service ready. Local service: '%s', Nav2 target: '%s'",
      local_service_name.c_str(),
      map_server_service_name_.c_str());

  if (autoload_on_startup && !autoload_map_url_.empty()) {
    startup_load_timer_ = create_wall_timer(
        std::chrono::seconds(2),
        [this]() {
          auto request = std::make_shared<LoadMap::Request>();
          auto response = std::make_shared<LoadMap::Response>();
          request->map_url = autoload_map_url_;

          handleLoadMap(request, response);

          if (response->result == LoadMap::Response::RESULT_SUCCESS) {
            RCLCPP_INFO(
                get_logger(),
                "Startup map load succeeded for '%s'.",
                autoload_map_url_.c_str());
            startup_load_timer_->cancel();
            return;
          }

          autoload_retries_left_--;
          if (autoload_retries_left_ <= 0) {
            RCLCPP_ERROR(
                get_logger(),
                "Startup map load failed after retries for '%s'.",
                autoload_map_url_.c_str());
            startup_load_timer_->cancel();
          } else {
            RCLCPP_WARN(
                get_logger(),
                "Startup map load retrying (%d retries left) for '%s'.",
                autoload_retries_left_,
                autoload_map_url_.c_str());
          }
        });
  }
}

void MapLoaderService::handleLoadMap(
    const std::shared_ptr<LoadMap::Request> request,
    std::shared_ptr<LoadMap::Response> response)
{
  if (request->map_url.empty()) {
    RCLCPP_ERROR(get_logger(), "Received empty map_url request.");
    response->result = LoadMap::Response::RESULT_INVALID_MAP_METADATA;
    return;
  }

  auto helper_node = std::make_shared<rclcpp::Node>("map_loader_service_helper");
  auto helper_client = helper_node->create_client<LoadMap>(map_server_service_name_);
  rclcpp::executors::SingleThreadedExecutor helper_executor;
  helper_executor.add_node(helper_node);

  if (!helper_client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
        get_logger(),
        "Nav2 map server service '%s' is unavailable.",
        map_server_service_name_.c_str());
    response->result = LoadMap::Response::RESULT_UNDEFINED_FAILURE;
    return;
  }

  auto forward_req = std::make_shared<LoadMap::Request>();
  forward_req->map_url = request->map_url;
  auto future = helper_client->async_send_request(forward_req);

  const auto status = helper_executor.spin_until_future_complete(
      future, std::chrono::seconds(10));
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Timed out calling Nav2 load_map service.");
    response->result = LoadMap::Response::RESULT_UNDEFINED_FAILURE;
    return;
  }

  const auto nav2_response = future.get();
  response->map = nav2_response->map;
  response->result = nav2_response->result;

  if (response->result == LoadMap::Response::RESULT_SUCCESS) {
    if (response->map.header.frame_id.empty()) {
      response->map.header.frame_id = "map";
    }
    map_publisher_->publish(response->map);
    RCLCPP_INFO(
        get_logger(),
        "Loaded and published map from '%s'.",
        request->map_url.c_str());
  } else {
    RCLCPP_WARN(
        get_logger(),
        "Nav2 failed to load map '%s' (result=%u).",
        request->map_url.c_str(),
        static_cast<unsigned>(response->result));
  }
}

} // namespace slambot_localization

RCLCPP_COMPONENTS_REGISTER_NODE(slambot_localization::MapLoaderService)
