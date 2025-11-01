cmake_minimum_required(VERSION 3.8)
project(relocalization)

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
find_package(small_gicp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Open3D REQUIRED)
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

add_executable(icp_node src/icp.cpp)
target_link_libraries(icp_node
  ${PCL_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(icp_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  std_msgs
  radar_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs)
install(TARGETS icp_node DESTINATION lib/${PROJECT_NAME})

add_executable(viewer_node src/viewer.cpp)
target_link_libraries(viewer_node
  ${PCL_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(viewer_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  Open3D
  std_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs)
install(TARGETS viewer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
