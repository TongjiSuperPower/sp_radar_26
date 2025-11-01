cmake_minimum_required(VERSION 3.8)
project(pointcloud_locate)
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(FMT_INCLUDE_DIR "/usr/lib/x86_64-linux-gnu/")
set(FMT_LIBRARY "/usr/lib/x86_64-linux-gnu/libfmt.so.8")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(radar_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR})

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

add_subdirectory(tools)

add_executable(locate src/locate.cpp src/tracker.cpp)
target_link_libraries(locate ${OpenCV_LIBS} ${FMT_LIBRARY} yaml-cpp tools) 
ament_target_dependencies(locate
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  PCL
  pcl_conversions
  radar_msgs
)
install(TARGETS
  locate
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(publish_car_bbox 
  src/node.cpp)
ament_target_dependencies(publish_car_bbox
  rclcpp
  radar_msgs
)
install(TARGETS publish_car_bbox
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY configs launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
