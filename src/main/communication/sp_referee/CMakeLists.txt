cmake_minimum_required(VERSION 3.8)
project(sp_referee)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(radar_msgs REQUIRED)
set(CMAKE_INSTALL_RPATH /usr/local/lib)
find_package(serial REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})


add_executable(main
  src/main.cpp
  src/sp_referee.cpp
  src/read.cpp
  src/write.cpp
  src/check.cpp
)
ament_target_dependencies(main
  rclcpp
  radar_msgs
  serial
)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
