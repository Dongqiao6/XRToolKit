#!/usr/bin/env zsh

setopt ERR_EXIT

if [[ ! -f /opt/ros/humble/setup.zsh ]]; then
  print -u2 "找不到 /opt/ros/humble/setup.zsh，请确认 ROS 2 Humble 已正确安装。"
  exit 1
fi

typeset _xrt_build_file="${(%):-%N}"
typeset _xrt_build_ws="${_xrt_build_file:A:h}"
typeset _xrt_build_venv="${_xrt_build_ws}/.venv"
typeset _xrt_build_repo="${_xrt_build_ws:h}"

for _xrt_stray in \
  "${_xrt_build_repo}/build" \
  "${_xrt_build_repo}/install" \
  "${_xrt_build_repo}/log"; do
  if [[ -e "${_xrt_stray}" ]]; then
    print -u2 "发现放错位置的编译目录：${_xrt_stray}"
    print -u2 "请先运行 '${_xrt_build_ws}/clean.zsh'。"
    exit 1
  fi
done

if [[ ! -f "${_xrt_build_venv}/bin/activate" ]]; then
  print -u2 "工程内 Python 环境不存在：${_xrt_build_venv}"
  print -u2 "请先运行：'${_xrt_build_ws}/install_xrt_sdk.zsh'"
  exit 1
fi

source "${_xrt_build_venv}/bin/activate"
export XRT_VENV="${_xrt_build_venv}"
export PYTHONNOUSERSITE=1
export COLCON_HOME="${_xrt_build_ws}/.colcon"
export ROS_HOME="${_xrt_build_ws}/.ros"
export ROS_LOG_DIR="${_xrt_build_ws}/log/ros"
mkdir -p "${COLCON_HOME}" "${ROS_HOME}" "${ROS_LOG_DIR}"

# Colcon exports this while sourcing nested setup files. A stale exported value
# makes /opt/ros/humble/setup.zsh resolve the wrong workspace setup file.
unset COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.zsh

cd "${_xrt_build_ws}"
colcon \
  --log-base "${_xrt_build_ws}/log/build" \
  build \
  --symlink-install \
  --build-base "${_xrt_build_ws}/build" \
  --install-base "${_xrt_build_ws}/install" \
  --packages-up-to \
  xrt_teleop jaka_driver sr_amr_control robot_bringup

print ""
print "编译完成。当前终端加载工作区："
print "  source '${_xrt_build_ws}/setup.zsh'"
