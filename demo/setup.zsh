#!/usr/bin/env zsh

# 在 zsh 中加载：source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"

if [[ -z "${ZSH_VERSION:-}" ]]; then
  print -u2 "此脚本只能在 zsh 中 source。"
  return 1 2>/dev/null || exit 1
fi

if [[ ! -f /opt/ros/humble/setup.zsh ]]; then
  print -u2 "找不到 /opt/ros/humble/setup.zsh，请确认 ROS 2 Humble 已正确安装。"
  return 1
fi

typeset _xrt_setup_file="${(%):-%N}"
typeset _xrt_setup_ws="${_xrt_setup_file:A:h}"

# 防止旧终端中导出的 COLCON_CURRENT_PREFIX 让 ROS setup.zsh 去找另一个
# 工作区下并不存在的 setup.sh/setup.bash。
unset COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.zsh

if [[ ! -f "${_xrt_setup_ws}/install/setup.zsh" ]]; then
  print -u2 "工作区尚未编译：${_xrt_setup_ws}/install/setup.zsh 不存在。"
  print -u2 "请先运行：cd '${_xrt_setup_ws}' && ./build.zsh"
  unset _xrt_setup_file _xrt_setup_ws
  return 1
fi

source "${_xrt_setup_ws}/install/setup.zsh"
print "ROS 2 Humble 和 XRToolKit demo 工作区已加载。"

unset _xrt_setup_file _xrt_setup_ws
