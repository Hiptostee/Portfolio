cmake_minimum_required(VERSION 3.8)
project(mecanum_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_components SHARED
  src/mecanum_drive_controller.cpp
)

target_include_directories(${PROJECT_NAME}_components PUBLIC include)

ament_target_dependencies(${PROJECT_NAME}_components
  geometry_msgs
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "mecanum_drive_controller::MecanumDriveController"
  EXECUTABLE ${PROJECT_NAME}_exec
)

install(TARGETS
  ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
