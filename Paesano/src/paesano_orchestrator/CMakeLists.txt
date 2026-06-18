cmake_minimum_required(VERSION 3.8)
project(paesano_orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(paesano_local_map REQUIRED)
find_package(paesano_navigation REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_components SHARED
  src/orchestrator.cpp
  src/orchestrator_node.cpp
)
target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_components
  geometry_msgs
  nav_msgs
  paesano_local_map
  paesano_navigation
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)
target_compile_features(${PROJECT_NAME}_components PUBLIC cxx_std_17)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "paesano_orchestrator::Orchestrator"
  EXECUTABLE orchestrator
)

install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS orchestrator
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
