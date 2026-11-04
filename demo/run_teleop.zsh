#!/usr/bin/env zsh

setopt ERR_EXIT

typeset _run_file="${(%):-%N}"
typeset _run_demo_dir="${_run_file:A:h}"

source "${_run_demo_dir}/setup.zsh"

if ! "${_run_demo_dir}/launch_quest.zsh"; then
  print -u2 "Quest 3 自动启动或无线连接未就绪，遥操作没有启动。"
  print -u2 "请连接 Quest USB 数据线，或设置 QUEST_IP=<Quest IP> 后重试。"
  exit 1
fi

exec ros2 launch jaka_driver dual_arm_teleop.launch.py "$@"
