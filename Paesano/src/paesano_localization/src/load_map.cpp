#include "paesano_localization/load_map.hpp"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <future>

namespace paesano_localization
{

MapLoaderService::MapLoaderService(const rclcpp::NodeOptions &options)
    : rclcpp::Node("map_loader_service", options)
{
  map_server_service_name_ =
      declare_parameter<std::string>("map_server_load_service", "/map_server/load_map");
  const auto global_service_name =
      declare_parameter<std::string>("global_load_map_service", "/paesano/load_map");
  const auto local_service_name =
      declare_parameter<std::string>("load_map_service", "~/load_map");
  const auto map_topic =
      declare_parameter<std::string>("map_topic", "/map");
  const int map_server_wait_timeout_ms =
      static_cast<int>(declare_parameter<int>("map_server_wait_timeout_ms", 2000));
  const int map_server_response_timeout_ms =
      static_cast<int>(declare_parameter<int>("map_server_response_timeout_ms", 10000));
  service_wait_timeout_ =
      std::chrono::milliseconds(std::max(1, map_server_wait_timeout_ms));
  response_timeout_ =
      std::chrono::milliseconds(std::max(1, map_server_response_timeout_ms));

  map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(1).transient_local().reliable());
  helper_node_ = std::make_shared<rclcpp::Node>("map_loader_service_helper");
  helper_client_ = helper_node_->create_client<LoadMap>(map_server_service_name_);
  helper_executor_.add_node(helper_node_);
  helper_executor_thread_ = std::thread([this]() {
    helper_executor_.spin();
  });

  load_map_service_ = create_service<LoadMap>(
      global_service_name,
      std::bind(
          &MapLoaderService::handleLoadMap,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

  if (local_service_name != global_service_name) {
    load_map_private_service_ = create_service<LoadMap>(
        local_service_name,
        std::bind(
            &MapLoaderService::handleLoadMap,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
  }

  RCLCPP_INFO(
      get_logger(),
      "Map loader service ready. Global service: '%s', Local service: '%s', Nav2 target: '%s'",
      global_service_name.c_str(),
      local_service_name.c_str(),
      map_server_service_name_.c_str());
}

MapLoaderService::~MapLoaderService()
{
  helper_executor_.cancel();
  if (helper_executor_thread_.joinable()) {
    helper_executor_thread_.join();
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

  if (!helper_client_->wait_for_service(service_wait_timeout_)) {
    RCLCPP_ERROR(
        get_logger(),
        "Nav2 map server service '%s' is unavailable.",
        map_server_service_name_.c_str());
    response->result = LoadMap::Response::RESULT_UNDEFINED_FAILURE;
    return;
  }

  auto forward_req = std::make_shared<LoadMap::Request>();
  forward_req->map_url = request->map_url;
  auto future = helper_client_->async_send_request(forward_req);

  if (future.wait_for(response_timeout_) != std::future_status::ready) {
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

} // namespace paesano_localization

RCLCPP_COMPONENTS_REGISTER_NODE(paesano_localization::MapLoaderService)
