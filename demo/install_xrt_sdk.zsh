#!/usr/bin/env zsh

setopt ERR_EXIT PIPE_FAIL

typeset _xrt_install_file="${(%):-%N}"
typeset _xrt_install_root="${_xrt_install_file:A:h}"
typeset _xrt_pybind_repo="${XRT_PYBIND_DIR:-${_xrt_install_root}/.deps/XRoboToolkit-PC-Service-Pybind}"
typeset _xrt_base_python="${XRT_BASE_PYTHON:-/usr/bin/python3}"
typeset _xrt_venv="${_xrt_install_root}/.venv"
typeset _xrt_python="${_xrt_venv}/bin/python"
typeset _xrt_amr_wheel="${_xrt_install_root}/vendor/sros_sdk_py-1.4.1-py3-none-any.whl"
typeset _xrt_pip_cache="${_xrt_install_root}/.cache/pip"

if [[ ! -x "${_xrt_base_python}" ]]; then
  print -u2 "找不到基础 Python：${_xrt_base_python}"
  exit 1
fi

if [[ ! -x "${_xrt_python}" ]]; then
  print "创建工程内 Python 虚拟环境：${_xrt_venv}"
  if ! "${_xrt_base_python}" -m venv --system-site-packages "${_xrt_venv}"; then
    print -u2 "Python venv 创建失败。请安装 python3-venv 后重试。"
    exit 1
  fi
fi

export PYTHONNOUSERSITE=1
export PIP_CACHE_DIR="${_xrt_pip_cache}"
export PIP_REQUIRE_VIRTUALENV=1
mkdir -p "${_xrt_pip_cache}"
if ! "${_xrt_python}" -m pip install --upgrade pip; then
  print -u2 "pip 升级失败，将继续使用 venv 中的现有 pip。"
fi

# 官方发布 xrobotoolkit-sdk 预编译 wheel。依赖只写入 demo/.venv，不污染
# 用户 site-packages；venv 同时能看到 ROS 2 系统包。
print "从 PyPI 安装 xrobotoolkit-sdk 到 ${_xrt_venv}..."
if ! "${_xrt_python}" -m pip install --upgrade --ignore-installed xrobotoolkit-sdk; then
  print -u2 "PyPI 安装失败，尝试从现有源码目录构建..."
  if [[ ! -f "${_xrt_pybind_repo}/pyproject.toml" ]]; then
    if ! command -v git >/dev/null 2>&1; then
      print -u2 "找不到 git，无法下载 Pybind 源码。"
      exit 1
    fi
    print "克隆 XRoboToolkit PC-Service Pybind 到 ${_xrt_pybind_repo}"
    mkdir -p "${_xrt_pybind_repo:h}"
    git clone --depth 1 \
      https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git \
      "${_xrt_pybind_repo}"
  else
    print "使用已有 Pybind 源码：${_xrt_pybind_repo}"
  fi
  "${_xrt_python}" -m pip install --upgrade "${_xrt_pybind_repo}"
fi

print "安装 AMR SDK 到 ${_xrt_venv}..."
"${_xrt_python}" -m pip install --force-reinstall "${_xrt_amr_wheel}"

if ! "${_xrt_python}" -c \
  "import xrobotoolkit_sdk, sros_sdk_py" >/dev/null 2>&1; then
  print -u2 "安装流程结束，但 ${_xrt_python} 无法导入项目依赖。"
  print -u2 "请检查安装输出。"
  print -u2 "PC-service Pybind 源码目录：${_xrt_pybind_repo}"
  exit 1
fi

print "项目 Python 环境安装完成：${_xrt_venv}"
print "xrobotoolkit_sdk 和 sros_sdk_py 均只安装在该环境中。"
print "下一步：cd '${_xrt_install_root}' && ./clean.zsh && ./build.zsh"
