cmake_minimum_required(VERSION 3.8)
project(dib_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LandingTarget6D.msg"
  "msg/LandingTarget6DArray.msg"
  "msg/BoxState.msg"
  "msg/BoxInfo.msg"
  "msg/BoxEnvironment.msg"
  "msg/BoxPower.msg"
  "msg/BatteryStatus.msg"
  "msg/RTKInfo.msg"
  "msg/BoxTelemetry.msg"
  "msg/BoxCmd.msg"
  "msg/LidStatus.msg"
  "msg/ClampStatus.msg"
  "msg/ChargeStatus.msg"
  "msg/CoolingStatus.msg"
  "msg/PowerStatus.msg"
  "msg/State.msg"
  "msg/DroneTelemetry.msg"
  "msg/WindSensor.msg"
  "msg/Temperature.msg"
  "msg/Humidity.msg"
  "msg/Rain.msg"
  "msg/BoxPose.msg"
  "msg/MissionItem.msg"
  "msg/MissionSchedule.msg"
  "msg/MissionScheduleCmd.msg"
  "srv/LidCmd.srv"
  "srv/ClampCmd.srv"
  "srv/ChargeCmd.srv"
  "srv/CoolingCmd.srv"
  "srv/PowerButtonCmd.srv"
  "srv/BoxCmd.srv"
  "srv/MissionUpload.srv"
  "srv/MissionScheduleCmd.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
