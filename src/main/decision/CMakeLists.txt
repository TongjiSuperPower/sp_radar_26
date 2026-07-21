cmake_minimum_required(VERSION 3.8)
project(decision)
# debug mode
# set(CMAKE_BUILD_TYPE debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(radar_parse_em_wave REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

include_directories(${EIGEN3_INCLUDE_DIR})
add_subdirectory(tools)

add_executable(decision
  src/decision.cpp src/tracker.cpp
)
target_link_libraries(decision tools yaml-cpp) 
ament_target_dependencies(decision
  rclcpp
  radar_msgs
  radar_parse_em_wave
)

install(TARGETS
  decision
  DESTINATION lib/${PROJECT_NAME}
)

# # test

# add_executable(tracker_test
#   src/test.cpp src/tracker.cpp
# )
# target_link_libraries(tracker_test tools yaml-cpp) 
# ament_target_dependencies(tracker_test
#   rclcpp
#   radar_msgs
# )

# install(TARGETS
# tracker_test
#   DESTINATION lib/${PROJECT_NAME}
# )


ament_package()
