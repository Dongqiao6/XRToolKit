#!/usr/bin/env zsh

# 删除本工程生成物。默认保留 demo/.venv，避免每次清理都重装 Python 环境。
# 用法：
#   ./clean.zsh         清理 colcon、make、ROS 日志和 Python 缓存
#   ./clean.zsh --venv  同时删除 demo/.venv

setopt ERR_EXIT

typeset _clean_file="${(%):-%N}"
typeset _clean_demo="${_clean_file:A:h}"
typeset _clean_repo="${_clean_demo:h}"
typeset -a _clean_targets
_clean_targets=(
  "${_clean_demo}/build"
  "${_clean_demo}/install"
  "${_clean_demo}/log"
  "${_clean_demo}/.deps"
  "${_clean_demo}/.cache"
  "${_clean_demo}/.colcon"
  "${_clean_demo}/.ros"
  "${_clean_repo}/build"
  "${_clean_repo}/install"
  "${_clean_repo}/log"
)

for _clean_target in "${_clean_targets[@]}"; do
  if [[ -e "${_clean_target}" ]]; then
    print "删除 ${_clean_target}"
    rm -rf -- "${_clean_target}"
  fi
done

find "${_clean_demo}" -type d -name __pycache__ -prune -exec rm -rf -- {} +
rm -f -- \
  "${_clean_demo}/xrt_receive_test" \
  "${_clean_demo}/xrt_received.jsonl"

if [[ "${1:-}" == "--venv" ]]; then
  if [[ -e "${_clean_demo}/.venv" ]]; then
    print "删除 ${_clean_demo}/.venv"
    rm -rf -- "${_clean_demo}/.venv"
  fi
fi

print "清理完成。所有后续编译产物都会位于：${_clean_demo}/{build,install,log}"
