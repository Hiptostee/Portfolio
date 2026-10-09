cmake_minimum_required(VERSION 3.8)
project(paesano_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(camera_ingest_component SHARED src/camera_ingest.cpp)
target_include_directories(camera_ingest_component PUBLIC include)
ament_target_dependencies(camera_ingest_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(camera_ingest_component
  PLUGIN "paesano_vision::CameraIngest"
  EXECUTABLE camera_ingest_node
)

install(TARGETS camera_ingest_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
