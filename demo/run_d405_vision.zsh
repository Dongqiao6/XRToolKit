#!/usr/bin/env zsh

setopt ERR_EXIT NO_UNSET

typeset _vision_file="${(%):-%N}"
typeset _vision_demo="${_vision_file:A:h}"
typeset _vision_venv="${_vision_demo}/.venv"

if [[ ! -x "${_vision_venv}/bin/python" ]]; then
  print -u2 "工程内 Python 环境不存在，请先运行 ${_vision_demo}/install_xrt_sdk.zsh"
  exit 1
fi

export PYTHONNOUSERSITE=1
exec "${_vision_venv}/bin/python" \
  "${_vision_demo}/d405_vision/d405_vision_sender.py" "$@"
