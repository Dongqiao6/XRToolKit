#!/usr/bin/env python3
"""等待 Quest 遥操作数据：默认等所有启用臂的 target。

`TeleopHeartbeat` 只能证明桥进程活着；`/vr_safe/teleop/*_target` 才证明
Quest 的位姿数据真的到了。启动脚本用这个区分“桥起来了”和“头显没数据”。

启用侧从 config/teleop.yaml 读取（enable_left / enable_right）；读不到就按
双臂处理。单臂配置只等启用那一侧。
"""

import argparse
import os
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def default_sides():
    root = os.environ.get("VR_SAFE_ROOT", os.getcwd())
    path = os.path.join(root, "config", "teleop.yaml")
    left, right = True, True
    try:
        with open(path) as f:
            cfg = yaml.safe_load(f) or {}
        params = (cfg.get("vr_safe_teleop_bridge") or {}).get("ros__parameters") or {}
        left = bool(params.get("enable_left", True))
        right = bool(params.get("enable_right", True))
    except Exception:
        pass
    sides = [s for s, on in (("left", left), ("right", right)) if on]
    return sides or ["left", "right"]


class WaitTeleop(Node):
    def __init__(self, sides):
        super().__init__("vr_safe_wait_teleop")
        self.sides = tuple(sides)
        self.got = set()
        for side in self.sides:
            self.create_subscription(
                PoseStamped, f"/vr_safe/teleop/{side}_target",
                lambda _m, s=side: self.got.add(s), 10)

    def done(self):
        return set(self.sides) <= self.got


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--sides", default="",
                    help="逗号分隔：left,right；默认按 teleop.yaml 的启用侧")
    args = ap.parse_args()
    if args.sides:
        sides = [s.strip() for s in args.sides.split(",") if s.strip()]
    else:
        sides = default_sides()
    rclpy.init()
    node = WaitTeleop(sides)
    end = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        got, need = set(node.got), set(sides)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if need <= got:
        print(f"✓ 收到 Quest 数据（{','.join(sorted(got))}）")
        return 0
    missing = ",".join(sorted(need - got)) or ",".join(sorted(need))
    print(f"✗ {args.timeout:.0f}s 内没有收到 Quest 数据（缺 {missing}）",
          file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
