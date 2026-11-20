#!/usr/bin/env bash
# 离线优先地准备 VR_safe：venv、依赖、模型检查、colcon build。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

need() { [ -e "$1" ] || { echo "✗ 缺少 $1" >&2; exit 1; }; }

echo "== 1/5 venv =="
if [ ! -x .venv/bin/python ]; then
  python3 -m venv .venv
fi
. .venv/bin/activate

echo "== 2/5 Python 依赖 =="
if [ "${VR_SAFE_OFFLINE:-0}" = "1" ]; then
  need vendor/wheels
  pip install --no-index --find-links vendor/wheels -r requirements.txt
else
  pip install --find-links vendor/wheels -r requirements.txt
fi

echo "== 3/5 模型 =="
for f in models/pose_landmarker_full.task models/hand_landmarker.task; do
  need "$f"
done

echo "== 4/5 配置 =="
need config/vr_safe.yaml

echo "== 5/5 构建 =="
source /opt/ros/humble/setup.bash
colcon build --event-handlers console_direct+

echo "✓ VR_safe 准备好了。source scripts/env.sh 后跑 scripts/start.sh --dry-run"
