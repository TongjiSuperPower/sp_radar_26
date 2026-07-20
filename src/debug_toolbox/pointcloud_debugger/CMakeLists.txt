cmake_minimum_required(VERSION 3.8)
project(pointcloud_debugger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(radar_msgs REQUIRED)

add_executable(pointcloud_debugger
  src/pointcloud_debugger.cpp
)
ament_target_dependencies(pointcloud_debugger
  rclcpp
  sensor_msgs
  radar_msgs
)
install(TARGETS pointcloud_debugger
  DESTINATION lib/${PROJECT_NAME})

ament_package()