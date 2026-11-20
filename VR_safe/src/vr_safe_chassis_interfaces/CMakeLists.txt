cmake_minimum_required(VERSION 3.8)
project(vr_safe_chassis_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 只复制底盘状态/电池消息；action、Path、充电、定位都留在 demo，不进入新系统。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SystemState.msg"
  "msg/BatteryState.msg"
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
