#!/usr/bin/env python3
"""终端状态面板：许可、观察、双臂、底盘。"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from vr_safe_interfaces.msg import (ArmState, ChassisState, MotionPermit,
                                    SafetyObservation)


class Status(Node):
    def __init__(self):
        super().__init__("vr_safe_status")
        self.permit = None
        self.obs = None
        self.arm = {}
        self.chassis = None
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(MotionPermit, "/vr_safe/permit",
                                 lambda m: setattr(self, "permit", m), qos)
        self.create_subscription(SafetyObservation, "/vr_safe/observation",
                                 lambda m: setattr(self, "obs", m), 10)
        for side in ("left", "right"):
            self.create_subscription(
                ArmState, f"/vr_safe/arm/{side}/state",
                lambda m, s=side: self.arm.__setitem__(s, m), 10)
        self.create_subscription(ChassisState, "/vr_safe/chassis/state",
                                 lambda m: setattr(self, "chassis", m), 10)

    @staticmethod
    def _decision(n):
        return {0: "GRANTED", 1: "SLOW", 2: "REVOKED"}.get(n, str(n))

    @staticmethod
    def _state(n):
        return {0: "CLEAR", 1: "NEAR", 2: "OCCUPIED", 3: "UNKNOWN",
                4: "ESTOP"}.get(n, str(n))

    def render_line(self):
        p, o, c = self.permit, self.obs, self.chassis
        if p is None:
            perm = "?"
        else:
            perm = self._decision(p.decision)
            if p.latched:
                perm += "(latched)"
            if p.reason:
                perm += ":" + p.reason[:24]
        obs = "?" if o is None else self._state(o.state)
        parts = [f"permit={perm}", f"obs={obs}"]
        for side in ("left", "right"):
            a = self.arm.get(side)
            parts.append(f"{side[0].upper()}="
                         + ("?" if a is None else
                            f"{'blocked' if a.command_blocked else 'run'}/"
                            f"{'stopped' if a.motion_stopped else 'moving'}"))
        parts.append("chassis=" + ("?" if c is None else c.state))
        return "  ".join(parts)

    def render(self):
        p = self.permit
        lines = []
        if p is None:
            lines.append("许可: （还没收到）")
        else:
            lines.append(f"许可: {self._decision(p.decision)}"
                         f"  latched={p.latched}  source={p.source}\n"
                         f"      原因: {p.reason}")
        o = self.obs
        if o is None:
            lines.append("观察: （还没收到）")
        else:
            lines.append(f"观察: {self._state(o.state)}  source={o.source_id}"
                         f"  zone={o.zone_id}  conf={o.confidence:.2f}")
        for side in ("left", "right"):
            a = self.arm.get(side)
            if a is None:
                lines.append(f"臂 {side}: （还没收到）")
            else:
                lines.append(f"臂 {side}: {a.state}  blocked={a.command_blocked}"
                             f"  stopped={a.motion_stopped}"
                             f"  tcp={a.tcp_speed_mps:.3f} m/s")
        c = self.chassis
        if c is None:
            lines.append("底盘: （还没收到）")
        else:
            lines.append(f"底盘: {c.state}  blocked={c.command_blocked}"
                         f"  hold={c.hold_active}")
        return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--line", action="store_true",
                    help="只打印一行紧凑状态，供 start.sh 每秒调用")
    ap.add_argument("--wait", type=float, default=1.5,
                    help="等第一条消息的秒数")
    args = ap.parse_args()
    rclpy.init()
    node = Status()
    try:
        end = time.monotonic() + max(0.0, args.wait)
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        if args.once:
            if args.line:
                print(node.render_line(), flush=True)
            else:
                print(node.render())
            return 0
        while rclpy.ok():
            print("\033[2J\033[H" + node.render(), flush=True)
            for _ in range(10):
                if not rclpy.ok():
                    break
                rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
