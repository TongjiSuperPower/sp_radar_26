cmake_minimum_required(VERSION 3.8)
project(cluster)
set(CMAKE_BUILD_TYPE "RELEASE")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(cluster src/cluster.cpp)
target_link_libraries(cluster
  ${PCL_LIBRARIES}
)
ament_target_dependencies(cluster
  rclcpp
  pcl_conversions
  PCL)
install(TARGETS cluster DESTINATION lib/${PROJECT_NAME})

ament_package()

