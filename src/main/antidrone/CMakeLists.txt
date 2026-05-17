cmake_minimum_required(VERSION 3.8)
project(antidrone)
set(CMAKE_BUILD_TYPE "RELEASE")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} 
  ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(tools)
add_subdirectory(io)

add_executable(antidrone src/antidrone.cpp)
target_link_libraries(antidrone
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS} 
  fmt::fmt
  yaml-cpp
  tools
  io
)
ament_target_dependencies(antidrone
  rclcpp
  pcl_conversions
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  OpenCV
  PCL)
install(TARGETS antidrone DESTINATION lib/${PROJECT_NAME})

ament_package()