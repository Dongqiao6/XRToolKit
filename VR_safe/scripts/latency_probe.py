#!/usr/bin/env python3
"""HIL 延迟探针：观察变成 OCCUPIED/UNKNOWN 后，双臂 blocked + 底盘 hold 的延迟。

用法（真机或 dry-run 均可）：
  source scripts/env.sh
  python3 scripts/latency_probe.py --events 10

输出每次 ms 和 p95。验收目标：p95 <= 200 ms。
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from vr_safe_interfaces.msg import (ArmState, ChassisState, SafetyObservation)


class LatencyProbe(Node):
    def __init__(self):
        super().__init__("vr_safe_latency_probe")
        self.t0 = None
        self.t0_wall = None
        self.pending = False
        self.arm = {}
        self.chassis = None
        self.samples = []
        self.create_subscription(SafetyObservation, "/vr_safe/observation",
                                 self._on_obs, 10)
        for side in ("left", "right"):
            self.create_subscription(
                ArmState, f"/vr_safe/arm/{side}/state",
                lambda m, s=side: self._on_arm(s, m), 10)
        self.create_subscription(ChassisState, "/vr_safe/chassis/state",
                                 self._on_chassis, 10)

    def _on_obs(self, msg):
        if msg.state in (SafetyObservation.OCCUPIED, SafetyObservation.UNKNOWN):
            self.t0 = time.monotonic()
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            self.t0_wall = stamp if stamp > 0.0 else None
            self.pending = True
            self.arm = {}
            self.chassis = None

    def _on_arm(self, side, msg):
        self.arm[side] = msg
        self._check()

    def _on_chassis(self, msg):
        self.chassis = msg
        self._check()

    def _check(self):
        if not self.pending or self.t0 is None:
            return
        if set(self.arm) != {"left", "right"} or self.chassis is None:
            return
        if not all(a.command_blocked and a.motion_stopped
                   for a in self.arm.values()):
            return
        if not self.chassis.hold_active:
            return
        # 优先用观察消息的 header 时间戳，量的是“观察发布 → 执行端停稳”；
        # 没有时间戳时才退回本进程收到消息的时刻。
        if self.t0_wall is not None:
            ms = (time.time() - self.t0_wall) * 1000.0
        else:
            ms = (time.monotonic() - self.t0) * 1000.0
        self.samples.append(ms)
        print(f"  sample {len(self.samples)}: {ms:.1f} ms", flush=True)
        self.pending = False
        self.t0 = None
        self.t0_wall = None

    def p95(self):
        if not self.samples:
            return None
        s = sorted(self.samples)
        idx = min(len(s) - 1, int(round(0.95 * (len(s) - 1))))
        return s[idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--events", type=int, default=10)
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    node = LatencyProbe()
    end = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and len(node.samples) < args.events and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        p95 = node.p95()
        print(f"\n样本 {len(node.samples)} 次；p95 = "
              f"{'—' if p95 is None else f'{p95:.1f} ms'}", flush=True)
        if p95 is not None:
            print("✓ 通过（p95 <= 200 ms）" if p95 <= 200 else "✗ 未通过（p95 > 200 ms）")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if p95 is not None and p95 <= 200 else 1


if __name__ == "__main__":
    raise SystemExit(main())
