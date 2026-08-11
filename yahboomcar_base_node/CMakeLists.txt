cmake_minimum_required(VERSION 3.5)
project(yahboomcar_base_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp  REQUIRED)
#find_package(turtlesim REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(base_node_X3 src/base_node_X3.cpp)
add_executable(base_node_x1 src/base_node_x1.cpp)
add_executable(base_node_R2 src/base_node_R2.cpp)
ament_target_dependencies(base_node_X3 rclcpp tf2 tf2_ros nav_msgs geometry_msgs)
ament_target_dependencies(base_node_x1 rclcpp tf2 tf2_ros nav_msgs geometry_msgs)
ament_target_dependencies(base_node_R2 rclcpp tf2 tf2_ros nav_msgs geometry_msgs)
install(TARGETS
  base_node_X3
  base_node_x1
  base_node_R2
	DESTINATION lib/${PROJECT_NAME}
)
ament_package()
