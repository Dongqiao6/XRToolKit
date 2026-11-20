cmake_minimum_required(VERSION 3.8)
project(vr_safe_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyObservation.msg"
  "msg/MotionPermit.msg"
  "msg/TeleopHeartbeat.msg"
  "msg/ArmState.msg"
  "msg/ChassisState.msg"
  "srv/ConfirmResume.srv"
  DEPENDENCIES std_msgs
)

ament_package()
