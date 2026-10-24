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
typeset _xrt_venv="${_xrt_setup_ws}/.venv"

if [[ ! -f "${_xrt_venv}/bin/activate" ]]; then
  print -u2 "工程内 Python 环境不存在：${_xrt_venv}"
  print -u2 "请先运行：'${_xrt_setup_ws}/install_xrt_sdk.zsh'"
  unset _xrt_setup_file _xrt_setup_ws _xrt_venv
  return 1
fi

source "${_xrt_venv}/bin/activate"
export XRT_VENV="${_xrt_venv}"
export PYTHONNOUSERSITE=1
export ROS_HOME="${_xrt_setup_ws}/.ros"
export ROS_LOG_DIR="${_xrt_setup_ws}/log/ros"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

# 防止旧终端中导出的 COLCON_CURRENT_PREFIX 让 ROS setup.zsh 去找另一个
# 工作区下并不存在的 setup.sh/setup.bash。
unset COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.zsh || return 1

if [[ ! -f "${_xrt_setup_ws}/install/setup.zsh" ]]; then
  print -u2 "工作区尚未编译：${_xrt_setup_ws}/install/setup.zsh 不存在。"
  print -u2 "请先运行：cd '${_xrt_setup_ws}' && ./build.zsh"
  unset _xrt_setup_file _xrt_setup_ws
  return 1
fi

if ! "${_xrt_venv}/bin/python" -c \
  "import xrobotoolkit_sdk, sros_sdk_py" >/dev/null 2>&1; then
  print -u2 "工程内 Python 环境无法导入项目依赖。"
  print -u2 "请运行：'${_xrt_setup_ws}/install_xrt_sdk.zsh'"
  unset _xrt_setup_file _xrt_setup_ws _xrt_venv
  return 1
fi

source "${_xrt_setup_ws}/install/setup.zsh" || {
  print -u2 "无法加载 ${_xrt_setup_ws}/install/setup.zsh。"
  unset _xrt_setup_file _xrt_setup_ws _xrt_venv
  return 1
}
print "ROS 2 Humble、工程内 venv 和 XRToolKit demo 工作区已加载。"

unset _xrt_setup_file _xrt_setup_ws _xrt_venv
