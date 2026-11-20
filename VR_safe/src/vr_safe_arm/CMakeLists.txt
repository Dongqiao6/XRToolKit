cmake_minimum_required(VERSION 3.8)
project(vr_safe_arm)

# 双臂执行端（工单 05）：每个进程只连一台 JAKA，launch 起两个实例。
# 源码/头文件/JAKA SDK 从 Vision_Teleop/src/vision_rc 复制，运行期不引用兄弟目录。

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vr_safe_interfaces REQUIRED)

# 纯逻辑安全核心：不依赖 ROS/JAKA，供单元测试和控制节点共用。
add_library(vr_safe_arm_core STATIC
  src/arm_core.cpp
)
target_include_directories(vr_safe_arm_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

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
  vr_safe_interfaces
)

target_include_directories(rc_ctrl_node
  PUBLIC
    include
  PRIVATE
    ${ament_INCLUDE_DIRS}
)

target_link_libraries(rc_ctrl_node vr_safe_arm_core)

# JAKA SDK：必须随包复制并在安装目录内解析，运行期不能引用 Vision_Teleop。
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
target_link_libraries(rc_ctrl_node ${LIBRARY_DIR}/libjakaAPI.so)

set_target_properties(rc_ctrl_node PROPERTIES
  BUILD_RPATH "${LIBRARY_DIR}"
  INSTALL_RPATH "$ORIGIN"
)

install(TARGETS rc_ctrl_node DESTINATION lib/${PROJECT_NAME})
install(FILES lib/libjakaAPI.so DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_arm_core test/test_arm_core.cpp)
  target_link_libraries(test_arm_core vr_safe_arm_core)
  add_test(NAME test_arm_core COMMAND test_arm_core)
endif()

ament_package()
