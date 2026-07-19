cmake_minimum_required(VERSION 3.8)
project(radar_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # referee_read
  "msg/GameStatus.msg"
  "msg/GameRobotHP.msg"
  "msg/EventData.msg"
  "msg/RobotStatus.msg"
  "msg/RadarMarkData.msg"
  "msg/RadarInfo.msg"
  
  # referee_write
  "msg/RadarCmd.msg"
  "msg/MapRobotData.msg"
  "msg/CustomInfo.msg"
  
  # detection
  "msg/Bbox.msg"
  "msg/CarBbox.msg"

  "msg/Car.msg"
  "msg/Cars.msg"

  "msg/CarsAndDrones.msg"

  # rf_wave
  "msg/RadarParseEmWave0A01RobotPosition.msg"
  "msg/RadarParseEmWave0A02RobotHp.msg"
  "msg/RadarParseEmWave0A03RobotAmmo.msg"
  "msg/RadarParseEmWave0A04FieldStatus.msg"
  "msg/RadarParseEmWave0A05RobotBuff.msg"
  "msg/RadarParseEmWave0A06InterferenceKey.msg"
  "msg/RadarParseEmWaveDemodConfig.msg"

  DEPENDENCIES std_msgs sensor_msgs
)

ament_package()
