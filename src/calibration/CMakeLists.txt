cmake_minimum_required(VERSION 3.8)
project(calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(FMT_INCLUDE_DIR "/usr/lib/x86_64-linux-gnu/")
set(FMT_LIBRARY "/usr/lib/x86_64-linux-gnu//libfmt.so.8")
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
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR})


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

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


# camera_calibration
add_subdirectory(camera_io/tools)
add_subdirectory(camera_io/io)

add_executable(camera_calibrate camera_calibration/camera_calibrate.cpp)
target_link_libraries(camera_calibrate ${OpenCV_LIBS} ${FMT_LIBRARY} tools io)
ament_target_dependencies(camera_calibrate
  rclcpp
)
install(TARGETS camera_calibrate
  DESTINATION lib/${PROJECT_NAME})


add_executable(img_shoot camera_calibration/img_shoot.cpp)
target_link_libraries(img_shoot ${OpenCV_LIBS} ${FMT_LIBRARY} tools io)
ament_target_dependencies(img_shoot
  rclcpp
)
install(TARGETS img_shoot
  DESTINATION lib/${PROJECT_NAME})


# lidar_camera_calibration/manual
include_directories(lidar_camera_calibration/manual/include)
add_executable(pointcloud_projection lidar_camera_calibration/manual/src/pointcloud_projection.cpp)
target_link_libraries(pointcloud_projection ${OpenCV_LIBS} ${FMT_LIBRARY} tools io)
ament_target_dependencies(pointcloud_projection
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  PCL
  pcl_conversions
)
install(TARGETS
  pointcloud_projection
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(rosbag_pointcloud_projection lidar_camera_calibration/manual/src/rosbag_pointcloud_projection.cpp)
target_link_libraries(rosbag_pointcloud_projection ${OpenCV_LIBS} ${FMT_LIBRARY} tools io)
ament_target_dependencies(rosbag_pointcloud_projection
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  PCL
  pcl_conversions
)
install(TARGETS
  rosbag_pointcloud_projection
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(tf_broadcast/include)
add_executable(tf_broadcast
  tf_broadcast/src/main.cpp
  tf_broadcast/src/tf_broadcast.cpp
  tf_broadcast/include/tf_broadcast.h
  tf_broadcast/src/tf_broadcast.ui
)
ament_target_dependencies(tf_broadcast
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(tf_broadcast Qt5::Widgets yaml-cpp)
install(TARGETS tf_broadcast
  DESTINATION lib/${PROJECT_NAME})


add_executable(coordinate_transform coordinate_transform/src/main.cpp)
target_link_libraries(coordinate_transform yaml-cpp)
ament_target_dependencies(coordinate_transform
  rclcpp
  PCL
  pcl_conversions
)
install(TARGETS coordinate_transform
  DESTINATION lib/${PROJECT_NAME})

add_executable(scaling coordinate_transform/src/scaling.cpp)
# target_link_libraries(coordinate_transform yaml-cpp)
ament_target_dependencies(scaling
  rclcpp
  PCL
  pcl_conversions
)
install(TARGETS scaling
  DESTINATION lib/${PROJECT_NAME})

add_executable(pcd_pcd pcd_pcd_calibration/pcd_pcd.cpp)
ament_target_dependencies(pcd_pcd
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
)

install(TARGETS pcd_pcd
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
