#!/usr/bin/env bash
# 启动 VR_safe。真机默认前台：日志写文件，终端每秒显示一行状态，Ctrl-C 停全部。
# 后台：./scripts/start.sh --real --background
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source scripts/env.sh

MODE="dry_run"
ALLOW_UNCAL=""
START_QUEST=1
FOREGROUND=1
CAMERA_VIEW=""
for arg in "$@"; do
  case "$arg" in
    --real) MODE="real" ;;
    --dry-run) MODE="dry_run" ;;
    --allow-uncalibrated) ALLOW_UNCAL="--allow-uncalibrated" ;;
    --no-quest) START_QUEST=0 ;;
    --foreground) FOREGROUND=1 ;;
    --background) FOREGROUND=0 ;;
    --camera-view) CAMERA_VIEW=1 ;;
    --no-camera-view) CAMERA_VIEW=0 ;;
    *) echo "未知参数：$arg" >&2; exit 2 ;;
  esac
done

PIDF="$ROOT/log/vr_safe.pid"
if [ -f "$PIDF" ] && kill -0 "$(cat "$PIDF")" 2>/dev/null; then
  echo "已经在跑（PID $(cat "$PIDF")），先 scripts/stop.sh" >&2
  exit 1
fi

echo "== 1/5 preflight =="
python3 scripts/preflight.py --mode "$MODE" ${ALLOW_UNCAL:+--allow-uncalibrated}

if [ "$MODE" = "real" ] && [ "$START_QUEST" = "1" ]; then
  echo "== 2/5 Quest 启动 =="
  if ! QUEST_WAIT_UNPLUG="${QUEST_WAIT_UNPLUG:-0}" ./scripts/launch_quest.sh; then
    echo "✗ Quest 未就绪，遥操作不启动。" >&2
    echo "  连 USB 或设置 QUEST_IP=... 后重试；APK 已在跑可加 --no-quest。" >&2
    exit 1
  fi
else
  echo "== 2/5 Quest 启动（跳过） =="
fi

mkdir -p "$ROOT/log"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="$ROOT/log/run_${STAMP}.log"
ln -sfn "$(basename "$LOG")" "$ROOT/log/latest.log"
ln -sfn "$(basename "$LOG")" "$ROOT/log/vr_safe.log"
# 保留最近 10 次运行日志，避免无限增长。
ls -1t "$ROOT"/log/run_*.log 2>/dev/null | tail -n +11 | xargs -r rm -f || true

# 默认：真机有桌面显示就开相机窗口；--no-camera-view 关闭。
if [ -z "$CAMERA_VIEW" ]; then
  if [ "$MODE" = "real" ] && { [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]; }; then
    CAMERA_VIEW=1
  else
    CAMERA_VIEW=0
  fi
fi
if [ "$MODE" != "real" ]; then
  CAMERA_VIEW=0
fi
if [ "$CAMERA_VIEW" = "1" ] && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "⚠ 没有 DISPLAY/WAYLAND_DISPLAY，关闭相机窗口"
  CAMERA_VIEW=0
fi

echo "== 3/5 启动节点 =="
if [ "$MODE" = "dry_run" ]; then
  CMD=(ros2 launch vr_safe_bringup dry_run.launch.py)
else
  CMD=(ros2 launch vr_safe_bringup vr_safe.launch.py)
  [ -n "$ALLOW_UNCAL" ] && CMD+=("allow_uncalibrated:=true")
  if [ "$CAMERA_VIEW" = "1" ]; then
    CMD+=("show_camera:=true")
  else
    CMD+=("show_camera:=false")
  fi
fi

if [ "$FOREGROUND" = "1" ]; then
  "${CMD[@]}" >"$LOG" 2>&1 &
else
  nohup "${CMD[@]}" >"$LOG" 2>&1 &
fi
LAUNCH_PID=$!
echo "$LAUNCH_PID" > "$PIDF"
sleep 1
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  echo "✗ 启动失败，日志：" >&2
  tail -30 "$LOG" >&2
  rm -f "$PIDF"
  exit 1
fi
echo "  节点已拉起（PID $LAUNCH_PID），日志 $LOG"
if [ "$CAMERA_VIEW" = "1" ]; then
  echo "  相机窗口：已开启（--no-camera-view 可关闭）"
else
  echo "  相机窗口：关闭（--camera-view 可开启）"
fi
DASH_PID=""
cleanup() {
  trap - HUP INT TERM
  echo
  [ -n "${DASH_PID:-}" ] && kill -TERM "$DASH_PID" 2>/dev/null || true
  ./scripts/stop.sh >/dev/null 2>&1 || true
  exit 0
}
trap cleanup HUP INT TERM

if [ "$MODE" = "real" ]; then
  TIMEOUT="${QUEST_DATA_TIMEOUT:-15}"
  echo "== 4/5 等待 Quest 数据（最多 ${TIMEOUT}s，启用侧都要到） =="
  if ! python3 scripts/wait_teleop.py --timeout "$TIMEOUT"; then
    echo "✗ 没等到 Quest 数据，停止本次启动。检查头显 Tracking/Send、PC-service、USB/无线。" >&2
    ./scripts/stop.sh >/dev/null 2>&1 || true
    exit 1
  fi
else
  echo "== 4/5 等待 Quest 数据（dry-run 跳过） =="
fi

if [ "$FOREGROUND" = "0" ]; then
  echo "== 5/5 已转后台 =="
  echo "  日志：tail -f $LOG"
  echo "  状态：python3 scripts/status.py"
  echo "  停止：./scripts/stop.sh"
  exit 0
fi

echo "== 5/5 运行中（Ctrl-C 停止） =="
python3 scripts/status_dashboard.py &
DASH_PID=$!
while kill -0 "$LAUNCH_PID" 2>/dev/null; do
  sleep 0.5
done
kill -TERM "$DASH_PID" 2>/dev/null || true
wait "$DASH_PID" 2>/dev/null || true

rm -f "$PIDF"
echo "⚠ launch 已退出。日志尾部："
tail -n 20 "$LOG" || true
if [ "$MODE" = "dry_run" ]; then
  exit 0
fi
exit 1
