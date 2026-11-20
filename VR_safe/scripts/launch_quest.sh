#!/usr/bin/env bash
# 通过 adb 连接 Quest 3 并启动 XRoboToolkit APK。
#
# 背景：XRoboToolkit 的 SDK 只负责读 PC-service 的数据，不会替你把 APK 拉起来。
# 这一步必须单独做；demo 里由 launch_quest.zsh 完成，VR_safe 现在自带。
#
# 用法：
#   ./scripts/launch_quest.sh                 # 自动找 USB/无线，安装并启动 APK
#   QUEST_IP=192.168.1.105 ./scripts/launch_quest.sh
#   QUEST_WAIT_UNPLUG=1 ./scripts/launch_quest.sh   # 有 USB 时等拔线（首次建议）
#
# 环境变量：QUEST_IP / QUEST_MAC / QUEST_ADB_PORT / QUEST_APK /
#           QUEST_WAIT_UNPLUG / QUEST_PACKAGE
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PACKAGE="${QUEST_PACKAGE:-com.xrobotoolkit.client.quest}"
PORT="${QUEST_ADB_PORT:-5555}"
MAC="${QUEST_MAC:-50:99:03:0a:38:78}"
APK="${QUEST_APK:-$ROOT/vendor/XRoboToolkit-Quest-1.0.1.apk}"
IP_FILE="${QUEST_IP_FILE:-$ROOT/.quest_ip}"
IP="${QUEST_IP:-}"
WAIT_UNPLUG="${QUEST_WAIT_UNPLUG:-0}"

if ! command -v adb >/dev/null 2>&1; then
  echo "✗ 找不到 adb；Quest 仍可手动打开 APK，但无法自动启动。" >&2
  exit 1
fi

if [ -z "$IP" ] && [ -f "$IP_FILE" ]; then
  IP="$(tr -d '[:space:]' < "$IP_FILE")"
fi

adb start-server >/dev/null 2>&1 || { echo "✗ adb server 启动失败" >&2; exit 1; }

mapfile -t DEVICES < <(adb devices | awk '$2 == "device" {print $1}')
USB_SERIAL=""
WIRELESS_SERIAL=""
for d in "${DEVICES[@]}"; do
  case "$d" in
    *:*) [ -z "$WIRELESS_SERIAL" ] && WIRELESS_SERIAL="$d" ;;
    *)   [ -z "$USB_SERIAL" ] && USB_SERIAL="$d" ;;
  esac
done
SERIAL="${WIRELESS_SERIAL:-$USB_SERIAL}"

norm_mac() { printf '%s' "$1" | tr '[:upper:]' '[:lower:]' | tr -d ':-'; }

if [ -z "$SERIAL" ] && [ -z "$IP" ] && [ -n "$MAC" ]; then
  NORM="$(norm_mac "$MAC")"
  IP="$(ip neigh show 2>/dev/null | awk -v mac="$NORM" \
    '{line=tolower($0); gsub(/[:-]/, "", line); if (index(line, mac)) {print $1; exit}}')"
fi
if [ -z "$SERIAL" ] && [ -z "$IP" ] && [ -n "$MAC" ]; then
  ping -c 1 -W 1 -b 255.255.255.255 >/dev/null 2>&1 || true
  NORM="$(norm_mac "$MAC")"
  IP="$(ip neigh show 2>/dev/null | awk -v mac="$NORM" \
    '{line=tolower($0); gsub(/[:-]/, "", line); if (index(line, mac)) {print $1; exit}}' || true)"
fi

if [ -z "$SERIAL" ] && [ -n "$IP" ]; then
  echo "尝试无线 adb 连接 ${IP}:${PORT}"
  adb connect "${IP}:${PORT}" >/dev/null 2>&1 || true
  SERIAL="${IP}:${PORT}"
fi

ready() { adb -s "$1" get-state 2>/dev/null | grep -qx "device"; }
usb_present() { adb devices 2>/dev/null | awk -v s="$1" '$1 == s && $2 == "device" {found=1} END {exit !found}'; }
app_running() { adb -s "$1" shell pidof "$PACKAGE" >/dev/null 2>&1; }

# 无线连接可能只是缓存的离线条目；有 USB 时退回 USB，不要让旧 IP 挡住。
if [ -n "$WIRELESS_SERIAL" ] && ! ready "$WIRELESS_SERIAL" && [ -n "$USB_SERIAL" ]; then
  echo "缓存无线 adb 不在线，改用 USB $USB_SERIAL" >&2
  SERIAL="$USB_SERIAL"
fi

if [ -z "$SERIAL" ] || ! ready "$SERIAL"; then
  echo "✗ 未找到可用的 Quest adb 设备。" >&2
  echo "  请先用 USB 连接 Quest，或运行：QUEST_IP=<Quest IP> $0" >&2
  echo "  如果缓存 IP 过期，删掉 $IP_FILE 或显式设置 QUEST_IP。" >&2
  echo "  当前 adb devices：" >&2
  adb devices -l >&2 || true
  exit 1
fi

if [ "$SERIAL" != *:* ]; then
  DETECTED_IP="$(adb -s "$SERIAL" shell ip -4 route 2>/dev/null |
    awk '{for (i = 1; i < NF; i++) if ($i == "src") {print $(i + 1); exit}}' || true)"
  if [ -n "$DETECTED_IP" ]; then
    printf '%s\n' "$DETECTED_IP" > "$IP_FILE"
    IP="$DETECTED_IP"
  fi
fi

if ! adb -s "$SERIAL" shell pm path "$PACKAGE" >/dev/null 2>&1; then
  if [ ! -f "$APK" ]; then
    echo "✗ Quest 未安装 $PACKAGE，且找不到 APK：$APK" >&2
    exit 1
  fi
  echo "Quest 未安装 XRoboToolkit，正在安装 $APK …"
  adb -s "$SERIAL" install -r "$APK" >/dev/null || { echo "✗ APK 安装失败" >&2; exit 1; }
fi

echo "通过 adb $SERIAL 启动 XRoboToolkit…"
adb -s "$SERIAL" shell monkey -p "$PACKAGE" -c android.intent.category.LAUNCHER 1 >/dev/null \
  || { echo "✗ APK 启动失败" >&2; exit 1; }

APP_OK=""
for _ in $(seq 1 10); do
  if app_running "$SERIAL"; then APP_OK=1; break; fi
  sleep 0.5
done
[ -n "$APP_OK" ] || { echo "✗ XRoboToolkit 进程没有起来" >&2; exit 1; }

# USB 模式：建立无线 adb，便于拔线后继续。
if [ "$SERIAL" != *:* ]; then
  if [ -z "$IP" ]; then
    echo "✗ 读不到 Quest IP，无法建立无线 adb" >&2
    exit 1
  fi
  if ! adb -s "$SERIAL" tcpip "$PORT" >/dev/null 2>&1; then
    echo "✗ 无法开启 Quest 无线 adb（tcpip $PORT）" >&2
    exit 1
  fi
  sleep 1
  WIRELESS="${IP}:${PORT}"
  adb connect "$WIRELESS" >/dev/null 2>&1 || true
  WIRELESS_OK=""
  for _ in $(seq 1 15); do
    if ready "$WIRELESS"; then WIRELESS_OK=1; break; fi
    sleep 1
  done
  [ -n "$WIRELESS_OK" ] || { echo "✗ Quest 无线 adb 连接失败" >&2; exit 1; }
  app_running "$WIRELESS" || { echo "✗ 无线连接后 APK 不在运行" >&2; exit 1; }
  if [ "$WAIT_UNPLUG" = "1" ] && usb_present "$SERIAL"; then
    echo "================================================================" >&2
    echo "XRoboToolkit 已启动，无线 adb 正常。请拔掉 USB 数据线。" >&2
    echo "拔掉后按 Enter 复检；不拔会继续等待。" >&2
    if [ ! -t 0 ]; then
      echo "⚠ 当前不是交互终端，跳过等待；请手动拔线。" >&2
    else
      while usb_present "$SERIAL"; do
        read -r -p "拔线后按 Enter… " _ || true
        sleep 1
      done
    fi
  elif usb_present "$SERIAL"; then
    echo "⚠ USB 数据线仍连接；建议拔掉，避免和无线/PC-service 抢设备。" >&2
  fi
  SERIAL="$WIRELESS"
fi

ready "$SERIAL" && app_running "$SERIAL" || { echo "✗ Quest 最终状态异常" >&2; exit 1; }
echo "✓ Quest APK 已在运行（adb $SERIAL）"
echo "  请在头显里确认 Tracking 和 Send 已开启。"
