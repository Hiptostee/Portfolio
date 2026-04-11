cmake_minimum_required(VERSION 3.8)
project(paesano_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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
  add_library(paesano_kinematic_base SHARED
    src/kinematic_base_system.cpp
  )
  target_compile_features(paesano_kinematic_base PUBLIC cxx_std_17)
  ament_target_dependencies(paesano_kinematic_base
    geometry_msgs
    rclcpp
    sensor_msgs
    std_msgs
  )
  target_link_libraries(paesano_kinematic_base
    ${GZ_SIM_TARGET}::${GZ_SIM_TARGET}
    ${GZ_PLUGIN_TARGET}::${GZ_PLUGIN_TARGET}
    ${GZ_MATH_TARGET}::${GZ_MATH_TARGET}
  )

  install(TARGETS paesano_kinematic_base
    LIBRARY DESTINATION lib
  )
else()
  message(WARNING
    "Skipping paesano_kinematic_base build because Gazebo development packages "
    "were not found. Found gz-sim target: ${GZ_SIM_TARGET}; "
    "found gz-plugin target: ${GZ_PLUGIN_TARGET}; "
    "found gz-math target: ${GZ_MATH_TARGET}.")
endif()

install(DIRECTORY
  config
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
