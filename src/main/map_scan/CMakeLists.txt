cmake_minimum_required(VERSION 3.8)
project(map_scan)
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(radar_msgs REQUIRED)

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

include_directories(include)
include_directories( ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(static_scan src/static_scan.cpp)
target_link_libraries(static_scan
  ${PCL_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(static_scan
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs)
install(TARGETS static_scan DESTINATION lib/${PROJECT_NAME})


add_executable(checking_period_scan src/checking_period_scan.cpp)
target_link_libraries(checking_period_scan
  ${PCL_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(checking_period_scan
  rclcpp
  sensor_msgs
  radar_msgs
  pcl_conversions
  PCL
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs)
install(TARGETS checking_period_scan DESTINATION lib/${PROJECT_NAME})

add_executable(pcd_process src/pcd_process.cpp)
target_link_libraries(pcd_process
  ${PCL_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(pcd_process
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs)
install(TARGETS pcd_process DESTINATION lib/${PROJECT_NAME})

add_executable(filter src/filter.cpp)
target_link_libraries(filter
  ${PCL_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(filter
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  geometry_msgs)
install(TARGETS filter DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
