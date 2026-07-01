cmake_minimum_required(VERSION 3.16)
project(dart_alertion)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)
include_directories(${PROJECT_SOURCE_DIR})

add_executable(dart_alertion src/dart_alertion.cpp)
ament_target_dependencies(dart_alertion
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
)

target_link_libraries(dart_alertion yaml-cpp ${OpenCV_LIBS})

install(TARGETS dart_alertion
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()