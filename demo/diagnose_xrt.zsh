#!/usr/bin/env zsh

setopt ERR_EXIT NO_UNSET

typeset _probe_file="${(%):-%N}"
typeset _probe_demo="${_probe_file:A:h}"
typeset _probe_python="${_probe_demo}/.venv/bin/python"

if [[ ! -x "${_probe_python}" ]]; then
  print -u2 "工程内 Python 环境不存在，请先运行 ${_probe_demo}/install_xrt_sdk.zsh"
  exit 1
fi

export PYTHONNOUSERSITE=1
export PYTHONPATH="${_probe_demo}/src/xrt_teleop${PYTHONPATH:+:${PYTHONPATH}}"
exec "${_probe_python}" \
  -m xrt_teleop.input_probe "$@"
