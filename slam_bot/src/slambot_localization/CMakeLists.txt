cmake_minimum_required(VERSION 3.5)
project(slambot_localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)



add_library(${PROJECT_NAME}_components SHARED src/covariances_on_imu.cpp src/particle_filter.cpp)
target_include_directories(${PROJECT_NAME}_components PUBLIC include)
ament_target_dependencies(
        ${PROJECT_NAME}_components
        "rclcpp"
        "rclcpp_components"
        "std_msgs"
        "sensor_msgs"
        "nav_msgs"
        "geometry_msgs"
        "tf2"
        "tf2_ros"
)
rclcpp_components_register_node(
        ${PROJECT_NAME}_components
        PLUGIN "covariances_on_imu::CovariancesOnImu"
        EXECUTABLE ${PROJECT_NAME}_CovariancesOnImu
)
rclcpp_components_register_node(
        ${PROJECT_NAME}_components
        PLUGIN "slambot_localization::ParticleFilter"
        EXECUTABLE ${PROJECT_NAME}_ParticleFilter
)

install(
        TARGETS ${PROJECT_NAME}_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  launch
  include
  src
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
