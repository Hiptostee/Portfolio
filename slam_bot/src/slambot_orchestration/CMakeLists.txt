cmake_minimum_required(VERSION 3.8)
project(slambot_orchestration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(slambot_interfaces REQUIRED)
find_package(slambot_navigation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(orchestration_node
  src/orchestration_node.cpp
)

ament_target_dependencies(orchestration_node
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  slambot_interfaces
  slambot_navigation
  std_msgs
  std_srvs
)

rosidl_get_typesupport_target(slambot_interfaces_typesupport_target
  slambot_interfaces
  "rosidl_typesupport_cpp"
)
rosidl_get_typesupport_target(slambot_navigation_typesupport_target
  slambot_navigation
  "rosidl_typesupport_cpp"
)
target_link_libraries(orchestration_node
  "${slambot_interfaces_typesupport_target}"
  "${slambot_navigation_typesupport_target}"
)

install(TARGETS
  orchestration_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
