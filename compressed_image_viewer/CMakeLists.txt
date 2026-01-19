cmake_minimum_required(VERSION 3.8)
project(compressed_image_viewer)

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

# 查找OpenCV - 方法1：使用find_package
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc imgcodecs)

# 如果上述方法失败，尝试方法2：使用pkg-config
if(NOT OpenCV_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(OpenCV REQUIRED opencv4)
endif()

# 包含目录
include_directories(include)

# 创建可执行文件
add_executable(compressed_image_viewer_node src/compressed_image_viewer.cpp)

# 链接库
ament_target_dependencies(compressed_image_viewer_node
  rclcpp
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
)

# 链接OpenCV库
if(OpenCV_FOUND)
  target_include_directories(compressed_image_viewer_node PUBLIC
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(compressed_image_viewer_node
    ${OpenCV_LIBRARIES}
  )
elseif(OpenCV_LIBRARIES)
  target_include_directories(compressed_image_viewer_node PUBLIC
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(compressed_image_viewer_node
    ${OpenCV_LIBRARIES}
  )
endif()

# 安装可执行文件
install(TARGETS compressed_image_viewer_node
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