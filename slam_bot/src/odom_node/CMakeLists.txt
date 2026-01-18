cmake_minimum_required(VERSION 3.5)
project(odom_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)



add_library(${PROJECT_NAME}_components SHARED src/odom_node.cpp)
target_include_directories(${PROJECT_NAME}_components PUBLIC include)
ament_target_dependencies(
        ${PROJECT_NAME}_components
        "rclcpp"
        "rclcpp_components"
        "nav_msgs"
        "std_msgs"
)
rclcpp_components_register_node(
        ${PROJECT_NAME}_components
        PLUGIN "odom_node::OdomNode"
        EXECUTABLE ${PROJECT_NAME}_exec
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
  DIRECTORY launch include src
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
