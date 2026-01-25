cmake_minimum_required(VERSION 3.8)
project(image_viewer)

# 默认使用C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


find_package(OpenCV REQUIRED)

# 包含目录
include_directories(include)

# 创建可执行文件
add_executable(image_viewer src/image_viewer.cpp)

# 链接库
ament_target_dependencies(image_viewer
  rclcpp
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_viewer
  ${OpenCV_LIBRARIES}
)

# 安装可执行文件
install(TARGETS image_viewer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
)

ament_package()