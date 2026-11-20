# Source this file before running anything:
#   source Vision_Teleop/env.sh
#
# It loads ROS 2 Humble, the local colcon install (if built), the vision_bridge
# python package from source, and finally the project venv.

_vision_root="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export VISION_TELEOP_ROOT="$_vision_root"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [ -f "$_vision_root/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$_vision_root/install/setup.bash"
fi

# rclpy comes from the system ROS install; the venv is deliberately isolated
# (no --system-site-packages) so numpy stays at the version we pin.
export PYTHONPATH="$_vision_root/src/vision_bridge:${PYTHONPATH:-}"

# shellcheck disable=SC1091
source "$_vision_root/.venv/bin/activate"

export MPLCONFIGDIR="$_vision_root/.cache/matplotlib"

# Keep ROS logs and caches inside the project so the workspace stays
# self-contained (and read-only homes still work).
export ROS_LOG_DIR="$_vision_root/.ros/log"
export ROS_HOME="$_vision_root/.ros/home"

# OpenCV's Qt GUI complains about a missing bundled font directory; point it at
# the system DejaVu fonts when they exist.
if [ -d /usr/share/fonts/truetype/dejavu ]; then
  export QT_QPA_FONTDIR=/usr/share/fonts/truetype/dejavu
fi

unset _vision_root
