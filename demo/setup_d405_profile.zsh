#!/usr/bin/env zsh

setopt ERR_EXIT PIPE_FAIL

typeset _profile_file="${(%):-%N}"
typeset _profile_demo="${_profile_file:A:h}"
typeset _profile_yaml="${_profile_demo}/d405_vision/video_source_d405.yml"
typeset _profile_package=""

if ! command -v adb >/dev/null 2>&1; then
  print -u2 "找不到 adb。请先连接 Quest 并运行 launch_quest.zsh。"
  exit 1
fi

typeset -a _profile_devices
_profile_devices=("${(@f)$(adb devices | awk '$2 == "device" {print $1}')}")
if (( ${#_profile_devices} == 0 )); then
  print -u2 "没有已连接的 Quest adb 设备。"
  exit 1
fi
typeset _profile_serial="${_profile_devices[1]}"

for _profile_package in \
  com.xrobotoolkit.client.quest \
  com.xrobotoolkit.client; do
  if adb -s "${_profile_serial}" shell pm path "${_profile_package}" \
      >/dev/null 2>&1; then
    break
  fi
  _profile_package=""
done

if [[ -z "${_profile_package}" ]]; then
  print -u2 "未找到 XRoboToolkit APK 包（尝试了 com.xrobotoolkit.client.quest 和 com.xrobotoolkit.client）。"
  exit 1
fi

typeset _profile_target="/sdcard/Android/data/${_profile_package}/files/video_source.yml"
adb -s "${_profile_serial}" shell mkdir -p "${_profile_target:h}"
adb -s "${_profile_serial}" push "${_profile_yaml}" "${_profile_target}"
adb -s "${_profile_serial}" shell am force-stop "${_profile_package}"
adb -s "${_profile_serial}" shell monkey \
  -p "${_profile_package}" \
  -c android.intent.category.LAUNCHER 1 >/dev/null

print "D405 视频源配置已写入 ${_profile_package}:"
print "  ${_profile_target}"
print "重新打开 Remote Vision 后，视频源列表应出现 REALSENSE_D405。"
