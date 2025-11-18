cmake_minimum_required(VERSION 3.8)
project(camera_detection LANGUAGES CXX CUDA)

# set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 添加依赖项
# CUDA
find_package(CUDA REQUIRED)                           
set(CMAKE_CUDA_ARCHITECTURES 89)                    
set(CUDA_PATH ${CUDA_TOOLKIT_ROOT_DIR})                 
set(CUDA_ROOT "/usr/local/cuda")
include_directories(${CUDA_ROOT}/include)
link_directories(${CUDA_ROOT}/lib64)
find_library(CUDA_LIBRARY cudart HINTS ${CUDA_ROOT}/lib64)

set(TENSORRT_PATH "/home/radar/TensorRT-8.6.1.6") 
include_directories("/home/radar/TensorRT-8.6.1.6/include") 

# find dependencies
set(FMT_INCLUDE_DIR "/usr/lib/x86_64-linux-gnu/")
set(FMT_LIBRARY "/usr/lib/x86_64-linux-gnu//libfmt.so.8")
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR})
find_package(radar_msgs REQUIRED)
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

# 添加公共配置
function(configure_cuda_trt target)
    # 添加CUDA定义、包含目录、链接库
    target_compile_definitions(${target} PUBLIC ${CUDA_DEFINITIONS})
    target_include_directories(${target} PUBLIC ${CUDA_INCLUDE_DIRS})
    target_link_libraries(${target} PUBLIC ${CUDA_cudart_LIBRARY})

    # 添加TensorRT的包含目录、库目录、链接库
    target_include_directories(${target} PUBLIC ${TENSORRT_PATH}/include)
    target_link_directories(${target} PUBLIC ${TENSORRT_PATH}/lib)
    if(MSVC AND EXISTS ${TENSORRT_PATH}/lib/nvinfer_10.dll)
        target_link_libraries(${target} PUBLIC nvinfer_10 nvinfer_plugin_10 nvonnxparser_10)
    else()
        target_link_libraries(${target} PUBLIC nvinfer nvinfer_plugin nvonnxparser)
    endif()
endfunction()

function(add_compile_files target)
    # 添加头文件搜索路径
    include_directories(${PROJECT_SOURCE_DIR}/yolo_detection/include) 
    file(GLOB_RECURSE SOURCES
        ${PROJECT_SOURCE_DIR}/yolo_detection/source/deploy/core/*.cpp
        ${PROJECT_SOURCE_DIR}/yolo_detection/source/deploy/utils/utils.cpp
        ${PROJECT_SOURCE_DIR}/yolo_detection/source/deploy/vision/*.cpp
        ${PROJECT_SOURCE_DIR}/yolo_detection/source/deploy/vision/*.cu
    )
    # 添加源文件
    target_sources(${target} PUBLIC ${SOURCES}) 
endfunction()

function(set_compile_options target)
    if(MSVC)
        target_compile_options(${target} PUBLIC $<$<CONFIG:Release>:-O2>)                                   # 对于MSVC，在Release模式下使用-O2优化
        set_property(TARGET ${target} PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>") # 设置MSVC运行时库
    else()
        target_compile_options(${target} PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-O3 -flto=auto>)                  # 对于非MSVC，使用-O3优化和自动LTO
        target_link_options(${target} PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-O3 -flto=auto>)                     # 链接时同样使用-O3优化和自动LTO
    endif()
endfunction()

add_subdirectory(camera_io/tools)
add_subdirectory(camera_io/io)
add_subdirectory(${PROJECT_SOURCE_DIR}/yolo_detection/plugin)  

add_library(deploy SHARED)                                                                            # 创建共享库
add_compile_files(deploy)                                                                             # 添加源文件
configure_cuda_trt(deploy)                                                                            # 配置CUDA和TensorRT
set_compile_options(deploy)                                                                           # 设置编译选项
set_target_properties(deploy PROPERTIES OUTPUT_NAME deploy)                                           # 设置输出名称

target_link_libraries(deploy PUBLIC ${OpenCV_LIBS} yaml-cpp)
if(MSVC)
    set_target_properties(deploy PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/yolo_detection/lib) # 设置Release模式下输出目录
else()
    set_target_properties(deploy PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/yolo_detection/lib)         # 设置库文件输出目录
endif()

add_executable(runtime_car_armor
  src/run_car_armor.cu
  src/detector_manager.cpp
  src/tracker.cpp
)
target_include_directories(runtime_car_armor PRIVATE ${TENSORRT_LIBRARY}/common)
target_link_libraries(runtime_car_armor  -Wl,--no-as-needed deploy)
target_link_libraries(runtime_car_armor  deploy ${NVINFER_LIBRARY} ${NVINFER_PLUGIN_LIBRARY} ${NVONNX_PARSER_LIB} ${CUDA_LIBRARY} yaml-cpp)
target_link_libraries(runtime_car_armor  ${OpenCV_LIBS} ${FMT_LIBRARY} tools io)
set_target_properties(runtime_car_armor PROPERTIES CUDA_ARCHITECTURES "61;70;75;86")
ament_target_dependencies(runtime_car_armor
  rclcpp
  radar_msgs
)
install(TARGETS runtime_car_armor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
