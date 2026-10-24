#!/usr/bin/env zsh

setopt ERR_EXIT

if [[ ! -f /opt/ros/humble/setup.zsh ]]; then
  print -u2 "找不到 /opt/ros/humble/setup.zsh，请确认 ROS 2 Humble 已正确安装。"
  exit 1
fi

typeset _xrt_build_file="${(%):-%N}"
typeset _xrt_build_ws="${_xrt_build_file:A:h}"

# Colcon exports this while sourcing nested setup files. A stale exported value
# makes /opt/ros/humble/setup.zsh resolve the wrong workspace setup file.
unset COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.zsh

cd "${_xrt_build_ws}"
colcon build --symlink-install --packages-up-to \
  xrt_teleop jaka_driver sr_amr_control robot_bringup

print ""
print "编译完成。当前终端加载工作区："
print "  source '${_xrt_build_ws}/install/setup.zsh'"
