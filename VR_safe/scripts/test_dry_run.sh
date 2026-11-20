#!/usr/bin/env bash
# 自动化 dry-run：启动 supervisor/fakes/scenario，期待 PASS。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source scripts/env.sh
LOG="$ROOT/log/dry_run_test.log"
mkdir -p "$ROOT/log"
set +e
timeout 90 ros2 launch vr_safe_bringup dry_run.launch.py >"$LOG" 2>&1
rc=$?
set -e
if grep -q "PASS: dry-run 安全闭环全部通过" "$LOG"; then
  echo "✓ dry-run 通过（launch rc=$rc）；日志 $LOG"
  exit 0
fi
echo "✗ dry-run 失败（launch rc=$rc）；见 $LOG"
tail -40 "$LOG"
exit 1
