#!/usr/bin/env zsh

# 自动连接一次 adb 并启动 Quest 3 上的 XRoboToolkit APK。
# 首次建议先用 USB 连接一次，之后脚本会保存无线 IP 并优先复用。

setopt PIPE_FAIL
emulate -L zsh

typeset _quest_file="${(%):-%N}"
typeset _quest_demo_dir="${_quest_file:A:h}"
typeset _quest_repo_dir="${_quest_demo_dir:h}"
typeset _quest_package="com.xrobotoolkit.client.quest"
typeset _quest_port="${QUEST_ADB_PORT:-5555}"
typeset _quest_mac="${QUEST_MAC:-50:99:03:0a:38:78}"
typeset _quest_apk="${_quest_repo_dir}/XRoboToolkit-Quest-1.0.1.apk"
typeset _quest_ip_file="${_quest_demo_dir}/.quest_ip"
typeset _quest_ip="${QUEST_IP:-}"

if ! command -v adb >/dev/null 2>&1; then
  print -u2 "找不到 adb。Quest 仍可手动打开 APK，但无法自动启动。"
  exit 1
fi

if [[ -z "${_quest_ip}" && -f "${_quest_ip_file}" ]]; then
  _quest_ip="$(<"${_quest_ip_file}")"
fi

adb start-server >/dev/null 2>&1 || {
  print -u2 "adb server 启动失败。"
  exit 1
}

typeset -a _quest_devices
_quest_devices=("${(@f)$(adb devices | awk '$2 == "device" {print $1}')}")
typeset _quest_serial=""
typeset _quest_device
typeset _quest_detected_ip=""

# 优先使用已经建立的无线连接，其次使用 USB。
for _quest_device in "${_quest_devices[@]}"; do
  if [[ "${_quest_device}" == *:* ]]; then
    _quest_serial="${_quest_device}"
    break
  fi
done
if [[ -z "${_quest_serial}" ]]; then
  for _quest_device in "${_quest_devices[@]}"; do
    if [[ "${_quest_device}" != *:* ]]; then
      _quest_serial="${_quest_device}"
      break
    fi
  done
fi

# 无线 adb 尚未连接时，尝试从保存的 IP 或 MAC/ARP 表解析地址。
if [[ -z "${_quest_serial}" && -z "${_quest_ip}" && -n "${_quest_mac}" ]]; then
  typeset _quest_norm_mac
  _quest_norm_mac="$(print -r -- "${_quest_mac}" | tr '[:upper:]' '[:lower:]' | tr -d ':-')"
  _quest_ip="$(
    ip neigh show 2>/dev/null | awk -v mac="${_quest_norm_mac}" \
      '{line=tolower($0); gsub(/[:-]/, "", line); if (index(line, mac)) {print $1; exit}}'
  )"
fi

if [[ -z "${_quest_serial}" && -z "${_quest_ip}" ]]; then
  ping -c 1 -W 1 -b 255.255.255.255 >/dev/null 2>&1 || true
  if [[ -n "${_quest_mac}" ]]; then
    typeset _quest_norm_mac
    _quest_norm_mac="$(print -r -- "${_quest_mac}" | tr '[:upper:]' '[:lower:]' | tr -d ':-')"
    _quest_ip="$(
      ip neigh show 2>/dev/null | awk -v mac="${_quest_norm_mac}" \
        '{line=tolower($0); gsub(/[:-]/, "", line); if (index(line, mac)) {print $1; exit}}'
    )"
  fi
fi

if [[ -z "${_quest_serial}" && -n "${_quest_ip}" ]]; then
  print "尝试无线 adb 连接 ${_quest_ip}:${_quest_port}"
  adb connect "${_quest_ip}:${_quest_port}" >/dev/null 2>&1 || true
  _quest_serial="${_quest_ip}:${_quest_port}"
fi

if [[ -z "${_quest_serial}" ]] || \
   ! adb -s "${_quest_serial}" get-state >/dev/null 2>&1; then
  print -u2 "未找到可用的 Quest adb 设备。"
  print -u2 "请先用 USB 连接 Quest，或在运行前设置 QUEST_IP=<Quest IP>。"
  print -u2 "本次仍会启动遥操作；你也可以在头显中手动打开 XRoboToolkit。"
  exit 1
fi

# USB 模式下获取 Quest IP，并保存供后续无 USB 启动使用。
if [[ "${_quest_serial}" != *:* ]]; then
  _quest_detected_ip="$(
    adb -s "${_quest_serial}" shell ip -4 route 2>/dev/null |
      awk '{for (i = 1; i < NF; i++) if ($i == "src") {print $(i + 1); exit}}'
  )"
  if [[ -n "${_quest_detected_ip}" ]]; then
    print -r -- "${_quest_detected_ip}" > "${_quest_ip_file}"
  fi
fi

if ! adb -s "${_quest_serial}" shell pm path "${_quest_package}" >/dev/null 2>&1; then
  if [[ ! -f "${_quest_apk}" ]]; then
    print -u2 "Quest 未安装 ${_quest_package}，且找不到 APK：${_quest_apk}"
    exit 1
  fi
  print "Quest 未安装 XRoboToolkit APK，正在安装..."
  adb -s "${_quest_serial}" install -r "${_quest_apk}" || exit 1
fi

print "通过 adb ${_quest_serial} 启动 XRoboToolkit..."
adb -s "${_quest_serial}" shell monkey \
  -p "${_quest_package}" \
  -c android.intent.category.LAUNCHER 1 >/dev/null || {
    print -u2 "XRoboToolkit APK 启动失败。"
    exit 1
  }

# 首次 USB 运行时建立无线 adb，后续可拔掉 USB。
if [[ "${_quest_serial}" != *:* && -n "${_quest_detected_ip}" ]]; then
  adb -s "${_quest_serial}" tcpip "${_quest_port}" >/dev/null 2>&1 || true
  sleep 1
  adb connect "${_quest_detected_ip}:${_quest_port}" >/dev/null 2>&1 || true
fi

print "XRoboToolkit APK 已启动；请在头显中确认 Tracking 和 Send 已开启。"
exit 0
