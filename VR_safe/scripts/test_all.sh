#!/usr/bin/env bash
# 全量测试：单元 + 状态机 + dry-run 闭环 + 真机 launch 冒烟。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source scripts/env.sh

echo "== supervisor =="
(cd src/vr_safe_supervisor && PYTHONPATH=.:$PYTHONPATH python3 -m pytest -q)

echo "== perception =="
(cd src/vr_safe_perception && PYTHONPATH=.:$PYTHONPATH python3 -m pytest -q test)

echo "== chassis =="
(cd src/vr_safe_chassis && PYTHONPATH=.:$PYTHONPATH python3 -m pytest -q test)

echo "== teleop =="
(cd src/vr_safe_teleop && PYTHONPATH=.:$PYTHONPATH python3 -m pytest -q)

echo "== camera view =="
(cd src/vr_safe_camera_view && PYTHONPATH=.:$PYTHONPATH python3 -m pytest -q test)

echo "== arm C++ =="
(cd build/vr_safe_arm && ctest --output-on-failure)

echo "== Quest 启动器（假 adb） =="
./scripts/test_launch_quest.sh

echo "== start.sh 前台 dry-run =="
timeout 40 ./scripts/start.sh --dry-run > "$ROOT/log/start_dry_test.log" 2>&1
grep -q "PASS: dry-run 安全闭环全部通过" "$ROOT/log/latest.log"
echo "✓ start.sh 前台 dry-run 通过"

echo "== dry-run 闭环 =="
./scripts/test_dry_run.sh

echo "== 真机 launch 冒烟（dry_run 节点） =="
./scripts/test_real_launch.sh

echo "✓ 全量测试通过"
