#!/usr/bin/env bash
# 进入 VR_safe 环境：ROS 2 + 本工程 venv。
# 用法：source scripts/env.sh
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export VR_SAFE_ROOT="$ROOT"

if [ ! -x "$ROOT/.venv/bin/python" ]; then
  echo "✗ 没有 $ROOT/.venv，先跑 scripts/bootstrap.sh" >&2
  return 1 2>/dev/null || exit 1
fi

source /opt/ros/humble/setup.bash
if [ -f "$ROOT/install/setup.bash" ]; then
  source "$ROOT/install/setup.bash"
fi

# 默认隔离：不和 demo / 现场 domain 0 串台。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="$ROOT/log/ros"
export ROS_HOME="$ROOT/.ros"
export PYTHONNOUSERSITE=1
export MPLCONFIGDIR="$ROOT/.cache/matplotlib"
export XDG_CACHE_HOME="$ROOT/.cache"
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME" "$MPLCONFIGDIR"

# venv 不用 --system-site-packages；rclpy 从 ROS 的 dist-packages 注入。
export PYTHONPATH="$ROOT/.venv/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export PATH="$ROOT/.venv/bin:$PATH"
