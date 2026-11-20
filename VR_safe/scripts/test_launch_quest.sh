#!/usr/bin/env bash
# 用假 adb 验证 Quest 启动器的 USB 主路径（不碰真 Quest）。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/bin"
cat > "$TMP/bin/adb" <<'ADB'
#!/usr/bin/env bash
if [ "$1" = "start-server" ]; then exit 0; fi
if [ "$1" = "devices" ]; then
  printf 'List of devices attached\nFAKEUSB\tdevice\n'; exit 0
fi
if [ "$1" = "connect" ]; then exit 0; fi
if [ "$1" = "-s" ]; then
  shift 2
  case "$*" in
    "get-state") echo device ;;
    "shell ip -4 route") echo "default via 10.0.0.1 dev wlan0 src 10.0.0.9" ;;
    "shell pm path com.xrobotoolkit.client.quest") echo "package:/data/app/base.apk" ;;
    "shell pidof com.xrobotoolkit.client.quest") echo 1234 ;;
    "shell monkey"*) exit 0 ;;
    "tcpip 5555") exit 0 ;;
    *) exit 0 ;;
  esac
  exit 0
fi
exit 0
ADB
chmod +x "$TMP/bin/adb"
: > "$TMP/fake.apk"
OUT="$(PATH="$TMP/bin:$PATH" QUEST_IP_FILE="$TMP/quest_ip" QUEST_APK="$TMP/fake.apk" \
  "$ROOT/scripts/launch_quest.sh" 2>&1)" || { echo "$OUT"; exit 1; }
echo "$OUT" | grep -q "Quest APK 已在运行" || { echo "$OUT"; exit 1; }
grep -q "10.0.0.9" "$TMP/quest_ip" || { echo "没有写回 Quest IP"; exit 1; }
echo "✓ launch_quest USB 路径通过（假 adb）"
