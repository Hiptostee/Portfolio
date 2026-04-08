cmake_minimum_required(VERSION 3.8)
project(paesano_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_components SHARED
  src/covariances_on_imu.cpp
  src/load_map.cpp
  src/particle_filter.cpp
  src/pf_compute_score.cpp
  src/pf_random_helpers.cpp
  src/pf_resample.cpp
  src/pf_tf_helpers.cpp
)

target_include_directories(${PROJECT_NAME}_components PUBLIC include)

ament_target_dependencies(${PROJECT_NAME}_components
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "covariances_on_imu::CovariancesOnImu"
  EXECUTABLE ${PROJECT_NAME}_CovariancesOnImu
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "paesano_localization::ParticleFilter"
  EXECUTABLE ${PROJECT_NAME}_ParticleFilter
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "paesano_localization::MapLoaderService"
  EXECUTABLE ${PROJECT_NAME}_MapLoaderService
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

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
