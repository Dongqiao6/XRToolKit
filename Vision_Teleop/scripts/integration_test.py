#!/usr/bin/env python3
"""ROS wiring test without camera or robot.

Publishes a synthetic target through the real VisionBridge node and checks the
messages that reach /vision/rc_ctrl/*:

    source env.sh
    python scripts/integration_test.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import rclpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "vision_bridge"))

from geometry_msgs.msg import PoseStamped  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402

from vision_bridge.node import VisionBridge  # noqa: E402

poses: list[PoseStamped] = []
joys: list[Joy] = []


def main() -> int:
    rclpy.init()
    node = VisionBridge()
    node.create_subscription(
        PoseStamped, "/vision/rc_ctrl/left_target", lambda m: poses.append(m), 10
    )
    node.create_subscription(
        PoseStamped, "/vision/rc_ctrl/right_target", lambda m: poses.append(m), 10
    )
    node.create_subscription(Joy, "/vision/rc_ctrl/button", lambda m: joys.append(m), 10)

    # Drive the publisher manually so the assertion sees every step.
    node._timer.cancel()

    # Pretend the left arm just engaged with a target 10 cm ahead.
    node.left.anchor_base = np.array([0.0, 0.0, 0.0])
    node.left.anchor_published = np.array([0.0, 0.0, 0.0])
    node.left.base_target = np.array([0.1, 0.0, 0.0])
    node.left.published = np.array([0.0, 0.0, 0.0])

    published: list[float] = []
    for _ in range(40):
        node._publish()
        published.append(float(node.left.published[0]))
        rclpy.spin_once(node, timeout_sec=0.02)

    ok = True
    if not poses:
        print("FAIL: no PoseStamped published")
        return 1
    xs = [p.pose.position.x for p in poses]
    print(f"poses: {len(poses)}  x range [{min(xs):.4f}, {max(xs):.4f}]")
    if max(xs) > 0.1 + 1e-6:
        print("FAIL: overshoot past target")
        ok = False
    if max(xs) < 0.05:
        print("FAIL: target not approached")
        ok = False
    steps = np.diff(published)
    if (steps < -1e-9).any():
        print("FAIL: target moved backwards")
        ok = False
    if len(steps) and steps.max() > node.config.safety.max_step_m + 1e-9:
        print(f"FAIL: step {steps.max():.4f} exceeds max_step_m")
        ok = False
    print(f"published: {len(published)} steps, max {steps.max()*1000:.2f} mm, final {published[-1]:.4f} m")

    if not joys:
        print("FAIL: no Joy published")
        return 1
    last = joys[-1]
    print(f"joy: axes={len(last.axes)} left_grip={last.axes[0]:.1f} right_grip={last.axes[1]:.1f}")
    if last.axes[0] != 1.0:
        print("FAIL: left grip should be engaged")
        ok = False
    if last.axes[1] != 0.0:
        print("FAIL: right grip should be released")
        ok = False

    node.destroy_node()
    rclpy.shutdown()
    print("integration ok" if ok else "integration failed")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
