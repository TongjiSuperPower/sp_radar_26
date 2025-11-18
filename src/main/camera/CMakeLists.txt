cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(fmt REQUIRED)

# Fix include directories - use absolute paths
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}  # This is key - adds the root as include path
)

# Add both subdirectories
add_subdirectory(camera_io/tools)
add_subdirectory(camera_io/io)

add_executable(image_creator
  src/image_creator.cpp
)

# Use modern target_include_directories instead of old include_directories
target_include_directories(image_creator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(image_creator 
  ${OpenCV_LIBS} 
  fmt::fmt 
  yaml-cpp 
  tools
  io
)

ament_target_dependencies(image_creator
  rclcpp
  radar_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  nlohmann_json
  fmt
)

install(TARGETS
  image_creator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()