cmake_minimum_required(VERSION 3.8)
project(video_player)
# debug mode
set(CMAKE_BUILD_TYPE debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(player
  player.cpp
)
target_link_libraries(player ${OpenCV_LIBS} yaml-cpp) 
ament_target_dependencies(player
  rclcpp
  radar_msgs
)

install(TARGETS
player
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
