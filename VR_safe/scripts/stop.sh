#!/usr/bin/env bash
# 停止 VR_safe：先 SIGINT 让各节点走清理，再兜底 SIGTERM；不要 kill -9 相机进程。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDF="$ROOT/log/vr_safe.pid"

if [ -f "$PIDF" ]; then
  PID="$(cat "$PIDF")"
  if kill -0 "$PID" 2>/dev/null; then
    CMD="$(tr '\0' ' ' < "/proc/$PID/cmdline" 2>/dev/null || true)"
    case "$CMD" in
      *ros2*launch*|*vr_safe*) ;;
      *) echo "⚠ PID $PID 不是 VR_safe 的进程，跳过：$CMD" >&2; PID="" ;;
    esac
  else
    PID=""
  fi
  if [ -n "$PID" ]; then
    kill -INT "$PID" 2>/dev/null || true
    for _ in $(seq 1 100); do
      kill -0 "$PID" 2>/dev/null || break
      sleep 0.1
    done
    kill -TERM "$PID" 2>/dev/null || true
  fi
  rm -f "$PIDF"
fi

pkill -INT -f "vr_safe_supervisor|vr_safe_fakes|vr_safe_teleop|vr_safe_perception|rc_ctrl_node|vr_safe_chassis" 2>/dev/null || true
echo "✓ VR_safe 已停"
