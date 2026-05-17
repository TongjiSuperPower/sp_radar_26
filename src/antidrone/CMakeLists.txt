cmake_minimum_required(VERSION 3.8)
project(detection_antidrone LANGUAGES CXX CUDA)
cmake_policy(SET CMP0074 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(MPI)
if(MPI_FOUND)
  if(NOT TARGET MPI::MPI_C)
    add_library(MPI::MPI_C INTERFACE IMPORTED)
  endif()
  if(NOT TARGET MPI::MPI_CXX)
    add_library(MPI::MPI_CXX INTERFACE IMPORTED)
  endif()
endif()

#-------------------------------------------------------------------------------
# 依赖项
#-------------------------------------------------------------------------------
# CUDA
find_package(CUDA REQUIRED)                           
set(CMAKE_CUDA_ARCHITECTURES 89)                    
set(CUDA_PATH ${CUDA_TOOLKIT_ROOT_DIR})                 
set(CUDA_ROOT "/usr/local/cuda")
include_directories(${CUDA_ROOT}/include)
link_directories(${CUDA_ROOT}/lib64)
find_library(CUDA_LIBRARY cudart HINTS ${CUDA_ROOT}/lib64)

# TensorRT
set(TENSORRT_PATH "/home/radar/TensorRT-8.6.1.6" CACHE PATH "Path to TensorRT")
include_directories(${TENSORRT_PATH}/include)
link_directories(${TENSORRT_PATH}/lib)

# Find TensorRT libraries
find_library(NVINFER_LIBRARY nvinfer PATHS ${TENSORRT_PATH}/lib REQUIRED)
find_library(NVINFER_PLUGIN_LIBRARY nvinfer_plugin PATHS ${TENSORRT_PATH}/lib REQUIRED)
find_library(NVONNX_PARSER_LIB nvonnxparser PATHS ${TENSORRT_PATH}/lib REQUIRED)

# fmt
set(FMT_LIBRARY "/usr/lib/x86_64-linux-gnu/libfmt.so.8")  # Fixed double slash

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(radar_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
#-------------------------------------------------------------------------------
# TRTYOLO Configuration
#-------------------------------------------------------------------------------
set(TRTYOLO_DIR "/home/radar/TensorRT-YOLO" CACHE PATH "Path to TRTYOLO source/build directory")

if(NOT EXISTS ${TRTYOLO_DIR})
    message(FATAL_ERROR "TRTYOLO directory not found: ${TRTYOLO_DIR}")
endif()

# TRTYOLO include path (头文件通常在include目录)
set(TRTYOLO_INCLUDE_DIR ${TRTYOLO_DIR}/include)
if(NOT EXISTS ${TRTYOLO_INCLUDE_DIR})
    # 尝试其他可能的include位置
    set(TRTYOLO_INCLUDE_DIR ${TRTYOLO_DIR}/modules/trtyolo/include)
endif()

# TRTYOLO library path (库文件可能在lib或build目录)
set(TRTYOLO_LIBRARY_DIR ${TRTYOLO_DIR}/lib)
if(NOT EXISTS ${TRTYOLO_LIBRARY_DIR})
    # 如果在lib找不到，尝试build目录
    set(TRTYOLO_LIBRARY_DIR ${TRTYOLO_DIR}/build)
endif()

# 查找TRTYOLO库
find_library(TRTYOLO_LIBRARY
    NAMES trtyolo
    PATHS 
        ${TRTYOLO_LIBRARY_DIR}
        ${TRTYOLO_DIR}/build/modules/trtyolo
        ${TRTYOLO_DIR}/modules/trtyolo
    REQUIRED
)

if(NOT TRTYOLO_LIBRARY)
    message(FATAL_ERROR "TRTYOLO library not found. Looked in: ${TRTYOLO_LIBRARY_DIR}")
endif()

message(STATUS "Found TRTYOLO library: ${TRTYOLO_LIBRARY}")
message(STATUS "TRTYOLO include dir: ${TRTYOLO_INCLUDE_DIR}")

# 添加头文件目录
include_directories(${TRTYOLO_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR})

function(set_compile_options target)
    if(MSVC)
        target_compile_options(${target} PUBLIC $<$<CONFIG:Release>:-O2>)
        set_property(TARGET ${target} PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    else()
        target_compile_options(${target} PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-O3 -flto=auto>)
        target_link_options(${target} PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-O3 -flto=auto>)
    endif()
endfunction()

#-------------------------------------------------------------------------------
# 子目录
#-------------------------------------------------------------------------------
add_subdirectory(tools)

add_executable(detection_antidrone
  src/detectionantidrone.cpp
)

# Include directories
target_include_directories(detection_antidrone PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${TENSORRT_PATH}/include  # Fixed from TENSORRT_LIBRARY/common
    ${CMAKE_CURRENT_SOURCE_DIR}/tools
)
target_include_directories(detection_antidrone PRIVATE ${TENSORRT_LIBRARY}/common)
target_link_libraries(detection_antidrone ${TRTYOLO_LIBRARY})
target_link_libraries(detection_antidrone ${NVINFER_LIBRARY} ${NVINFER_PLUGIN_LIBRARY} ${NVONNX_PARSER_LIB} ${CUDA_LIBRARY} yaml-cpp)
target_link_libraries(detection_antidrone ${OpenCV_LIBS} ${FMT_LIBRARY} tools)



set_compile_options(detection_antidrone)

set_target_properties(detection_antidrone PROPERTIES
    CUDA_ARCHITECTURES "61;70;75;86"
)

# ROS2 dependencies
ament_target_dependencies(detection_antidrone
    rclcpp
    radar_msgs
    sensor_msgs
    cv_bridge
    image_transport
    OpenCV
    nlohmann_json
    std_msgs
    message_filters
)

# Installation
install(TARGETS detection_antidrone
    DESTINATION lib/${PROJECT_NAME})

ament_package()