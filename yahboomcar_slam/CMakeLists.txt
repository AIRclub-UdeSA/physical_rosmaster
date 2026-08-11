cmake_minimum_required(VERSION 3.5)
project(yahboomcar_slam)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io visualization)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(pointcloud_mapping src/point_cloud_main.cpp src/point_cloud.cpp)
target_include_directories(pointcloud_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  pointcloud_mapping
  "rclcpp"
  "std_msgs"
  "pcl_conversions"
  "pcl_ros"
  "cv_bridge"
  "sensor_msgs"
  "image_transport"
  "tf2_ros"
  "message_filters"
  "geometry_msgs"
)


install(TARGETS 
  pointcloud_mapping
  DESTINATION lib/${PROJECT_NAME})

include_directories(include/yahboomcar_slam)
install(DIRECTORY launch params pcl rviz DESTINATION share/${PROJECT_NAME})

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

target_link_libraries(pointcloud_mapping ${PCL_LIBRARIES})
ament_package()
