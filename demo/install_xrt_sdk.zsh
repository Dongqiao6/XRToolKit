#!/usr/bin/env zsh

setopt ERR_EXIT PIPE_FAIL

typeset _xrt_install_file="${(%):-%N}"
typeset _xrt_install_root="${_xrt_install_file:A:h}"
typeset _xrt_pybind_repo="${XRT_PYBIND_DIR:-$HOME/.local/src/XRoboToolkit-PC-Service-Pybind}"

if ! command -v git >/dev/null 2>&1; then
  print -u2 "找不到 git，请先安装 git。"
  exit 1
fi

mkdir -p "${_xrt_pybind_repo:h}"
if [[ ! -d "${_xrt_pybind_repo}/.git" ]]; then
  print "克隆 XRoboToolkit PC-Service Pybind 到 ${_xrt_pybind_repo}"
  git clone --depth 1 \
    https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git \
    "${_xrt_pybind_repo}"
else
  print "使用已有 Pybind 源码：${_xrt_pybind_repo}"
fi

cd "${_xrt_pybind_repo}"

# 上游构建脚本目前是 setup_ubuntu.sh。它是一个一次性安装器，不作为 zsh
# 环境文件 source；必须先安装 XRoboToolkit-PC-Service 的 ARM64 版本。
bash setup_ubuntu.sh

if ! /usr/bin/python3 -c "import xrobotoolkit_sdk" >/dev/null 2>&1; then
  print -u2 "安装流程结束，但 /usr/bin/python3 仍无法导入 xrobotoolkit_sdk。"
  print -u2 "请检查 setup_ubuntu.sh 的输出，并确保它使用的是 /usr/bin/python3。"
  print -u2 "PC-service Pybind 源码目录：${_xrt_pybind_repo}"
  exit 1
fi

print "xrobotoolkit_sdk 安装成功，/usr/bin/python3 可以正常导入。"
print "下一步：cd '${_xrt_install_root}' && ./build.zsh"
