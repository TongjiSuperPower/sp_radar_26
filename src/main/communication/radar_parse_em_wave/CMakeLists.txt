cmake_minimum_required(VERSION 3.8)
project(radar_parse_em_wave)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # rf_wave
  "msg/RadarParseEmWave0A01RobotPosition.msg"
  "msg/RadarParseEmWave0A02RobotHp.msg"
  "msg/RadarParseEmWave0A03RobotAmmo.msg"
  "msg/RadarParseEmWave0A04FieldStatus.msg"
  "msg/RadarParseEmWave0A05RobotBuff.msg"
  "msg/RadarParseEmWave0A06InterferenceKey.msg"
  "msg/RadarParseEmWaveDemodConfig.msg"

  DEPENDENCIES std_msgs
)

ament_package()
