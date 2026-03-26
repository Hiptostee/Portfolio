cmake_minimum_required(VERSION 3.8)
project(slambot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/AStar.action"
  DEPENDENCIES
    geometry_msgs
    nav_msgs
)

add_library(${PROJECT_NAME}_components SHARED
  src/a_star.cpp
  src/a_star_helpers.cpp
  src/a_star_action_server.cpp
)
target_include_directories(${PROJECT_NAME}_components PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_components
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(${PROJECT_NAME}_components
  "${cpp_typesupport_target}"
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "slambot_navigation::AStarActionServer"
  EXECUTABLE a_star_action_server
)

install(TARGETS
  ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
