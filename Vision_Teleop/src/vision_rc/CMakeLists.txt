cmake_minimum_required(VERSION 3.8)
project(vision_rc)

# Copied from demo/src/jaka_driver (rc_ctrl_node + PGI-140 driver + JAKA SDK).
# Only the real-time teleoperation node is built here: the generic ROS-service
# driver (jaka_driver.cpp) and the AMR packages stay in the demo project.

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(rc_ctrl_node
  src/rc_ctrl_node.cpp
  src/pgi140_gripper.cpp
  src/pgi140_protocol.cpp
)

ament_target_dependencies(rc_ctrl_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

target_include_directories(rc_ctrl_node
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
target_link_libraries(rc_ctrl_node ${LIBRARY_DIR}/libjakaAPI.so)

set_target_properties(rc_ctrl_node PROPERTIES
  BUILD_RPATH "${LIBRARY_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

install(TARGETS rc_ctrl_node DESTINATION lib/${PROJECT_NAME})
install(FILES lib/libjakaAPI.so DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
