#!/usr/bin/env bash
# 在 ROS2 环境里跑我们 venv 的 python。
#
# 为什么要这么个壳：rclpy 的 C 扩展、动态库、消息定义都挂在 ROS 的一堆
# 环境变量上（AMENT_PREFIX_PATH / LD_LIBRARY_PATH / PYTHONPATH），光
# 激活 venv 是不够的；而反过来只 source ROS 又会用成系统 python
# （那边 numpy 是 1.23，mediapipe 和 rerun 都跑不了）。
# 两边都要：先 source ROS 拿环境变量，再用 venv 的解释器跑。
#
# ⚠️ 默认把 ROS_DOMAIN_ID 钉到 42，**和局域网里的机器人隔离开**。
# 这台机器所在的网段上，domain 0 里跑着真实机器人的控制接口：
#   /left_arm/motor_*/command   /gripper_control   /chassis_control
#   /head_control               /highlevel_control
# 我们的东西只是感知输出，一旦发到 domain 0 并且话题名撞上，
# 就可能把机械臂或底盘真的动起来。隔离是默认，要跨域得显式指定。
#
#   ./rosrun.sh zone_publisher.py            跑脚本（domain 42）
#   ROS_DOMAIN_ID=0 ./rosrun.sh xxx.py       真要接机器人时才显式改
#   ./rosrun.sh                              进交互 python
#
# 这里不能用 set -u：ROS 的 setup.bash 会读 AMENT_TRACE_SETUP_FILES 之类
# 尚未定义的变量，开了 -u 直接报「未绑定的变量」退出。
set -eo pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
# 只在本机内通信，连网卡都不出去。要跨机器再把这行注释掉。
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
source /opt/ros/humble/setup.bash
# 再叠上本项目的工作空间（workcell_interfaces 在这里），有才 source。
# 顺序不能反：局部工作空间要盖在系统之上。
_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/ros_ws/install/setup.bash"
[ -f "$_WS" ] && source "$_WS"
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.venv/bin/python" "$@"
