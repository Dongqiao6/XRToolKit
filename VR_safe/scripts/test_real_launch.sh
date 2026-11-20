#!/usr/bin/env bash
# 无硬件启动冒烟：真机 launch 的节点都起来，但 teleop/臂/底盘走 dry-run。
# 用来验证入口、参数类型、配置路径和节点装配；不验证 HIL 时序。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source scripts/env.sh
LOG="$ROOT/log/real_launch_smoke.log"
mkdir -p "$ROOT/log"
set +e
timeout 20 ros2 launch vr_safe_bringup vr_safe.launch.py \
  dry_run:=true start_perception:=false start_amr:=true start_arms:=true start_teleop:=true \
  >"$LOG" 2>&1
rc=$?
set -e
if grep -qE "process has died|Traceback" "$LOG"; then
  echo "✗ 真机 launch 冒烟失败（rc=$rc）；见 $LOG"
  tail -40 "$LOG"
  exit 1
fi
if ! grep -q "rc_ctrl_node started" "$LOG" || ! grep -q "vr_safe_chassis 已启动" "$LOG"; then
  echo "✗ 没看到臂/底盘启动；见 $LOG"
  tail -40 "$LOG"
  exit 1
fi
echo "✓ 真机 launch 冒烟通过（rc=$rc，dry_run 节点；未连硬件）"
