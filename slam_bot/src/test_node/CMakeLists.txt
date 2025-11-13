cmake_minimum_required(VERSION 3.5)
project(test_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)


add_library(${PROJECT_NAME}_components SHARED src/test_node.cpp)
ament_target_dependencies(
        ${PROJECT_NAME}_components
        "rclcpp"
        "rclcpp_components"
)
rclcpp_components_register_node(
        ${PROJECT_NAME}_components
        PLUGIN "test_node::TestNode"
        EXECUTABLE ${PROJECT_NAME}_exec
)

install(
        TARGETS ${PROJECT_NAME}_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
