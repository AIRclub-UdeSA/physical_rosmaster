cmake_minimum_required(VERSION 3.5)
project(yahboomcar_base_node)

# Default to C++14 for ROS 2 Humble.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp  REQUIRED)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(base_node_X3 src/base_node_X3.cpp)
ament_target_dependencies(base_node_X3 rclcpp tf2 tf2_ros diagnostic_msgs nav_msgs sensor_msgs geometry_msgs builtin_interfaces)
install(TARGETS
  base_node_X3
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  ament_add_gtest(test_x3_odometry test/test_x3_odometry.cpp)
  ament_target_dependencies(test_x3_odometry tf2 nav_msgs geometry_msgs builtin_interfaces)
endif()

ament_package()
