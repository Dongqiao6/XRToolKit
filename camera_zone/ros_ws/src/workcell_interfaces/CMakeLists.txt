cmake_minimum_required(VERSION 3.8)
project(workcell_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ZoneState.msg"
  "msg/MotionPermit.msg"
  "msg/ArmExecutionState.msg"
  "srv/ConfirmResume.srv"
  DEPENDENCIES std_msgs
)

ament_package()
