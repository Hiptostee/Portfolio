cmake_minimum_required(VERSION 3.8)
project(paesano_orchestrator)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(paesano_navigation REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME}_components SHARED
  src/orchestrator.cpp
  src/orchestrator_tick.cpp
)
target_include_directories(${PROJECT_NAME}_components PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_components
  geometry_msgs
  nav_msgs
  paesano_navigation
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "paesano_orchestrator::Orchestrator"
  EXECUTABLE orchestrator_node
)

install(TARGETS
  ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
