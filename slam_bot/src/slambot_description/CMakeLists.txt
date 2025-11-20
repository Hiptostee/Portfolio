cmake_minimum_required(VERSION 3.8)
project(slambot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Try to locate Gazebo Sim (gz-sim) and friends (version-agnostic)
set(GZ_SIM_CANDIDATES gz-sim8 gz-sim7)
set(GZ_MATH_CANDIDATES gz-math8 gz-math7)
set(GZ_PLUGIN_CANDIDATES gz-plugin2 gz-plugin1)

foreach(pkg ${GZ_SIM_CANDIDATES})
  find_package(${pkg} QUIET)
  if(${pkg}_FOUND)
    set(GZ_SIM_TARGET ${pkg})
    break()
  endif()
endforeach()

foreach(pkg ${GZ_MATH_CANDIDATES})
  find_package(${pkg} QUIET)
  if(${pkg}_FOUND)
    set(GZ_MATH_TARGET ${pkg})
    break()
  endif()
endforeach()

foreach(pkg ${GZ_PLUGIN_CANDIDATES})
  find_package(${pkg} QUIET)
  if(${pkg}_FOUND)
    set(GZ_PLUGIN_TARGET ${pkg})
    break()
  endif()
endforeach()

if(GZ_SIM_TARGET AND GZ_PLUGIN_TARGET AND GZ_MATH_TARGET)
  add_library(slambot_kinematic_base SHARED
    src/kinematic_base_system.cpp)
  target_compile_features(slambot_kinematic_base PUBLIC cxx_std_17)
  ament_target_dependencies(slambot_kinematic_base rclcpp geometry_msgs)
  target_link_libraries(slambot_kinematic_base
    ${GZ_SIM_TARGET}::${GZ_SIM_TARGET}
    ${GZ_PLUGIN_TARGET}::${GZ_PLUGIN_TARGET}
    ${GZ_MATH_TARGET}::${GZ_MATH_TARGET}
  )

  install(TARGETS slambot_kinematic_base
    LIBRARY DESTINATION lib)
endif()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY urdf launch config worlds
  DESTINATION share/${PROJECT_NAME}/
)

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
