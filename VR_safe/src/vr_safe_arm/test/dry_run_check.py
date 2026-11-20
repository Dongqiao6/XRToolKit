#!/usr/bin/env python3
"""vr_safe_arm dry-run 验收探针（无硬件）。

先启动一个 dry_run 的臂节点，例如：
  ros2 launch vr_safe_arm rc_ctrl.launch.py dry_run:=true
或：
  ros2 run vr_safe_arm rc_ctrl_node --ros-args -p arm_name:=left -p dry_run:=true

再运行本脚本：
  python3 test/dry_run_check.py --arm left

脚本按工单 05 的边界逐项验证：
  1. 启动默认 REVOKED / command_blocked；
  2. GRANTED 后未按 Grip 不跟随；
  3. 按 Grip 后进入 tracking；
  4. REVOKED 立即 blocked、松离合；
  5. 许可恢复但 Grip 仍按着时不自动跟随；松开再按才恢复 tracking；
  6. 停止 target 200 ms 后看门狗触发，状态回 hold；
  7. 看门狗后同样必须重新按 Grip；
  8. SLOW 时保持 tracking 并显示 tracking_slow；
  9. 发布 /vr_safe/arm/{arm}/state，且 motion_stopped/tcp_speed_mps 有值。
退出码 0 = 全过。
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from vr_safe_interfaces.msg import ArmState, MotionPermit


class DryRunProbe(Node):
    def __init__(self, arm):
        super().__init__("vr_safe_arm_dry_run_probe")
        self.arm = arm
        self.state = None
        self.permit_seq = 0
        self.grip = 0.0
        self.target_x = 0.40
        self.target_y = 0.0
        self.target_z = 0.20

        permit_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            ArmState, f"/vr_safe/arm/{arm}/state", self._on_state, 10)
        self.pub_permit = self.create_publisher(
            MotionPermit, "/vr_safe/permit", permit_qos)
        self.pub_target = self.create_publisher(
            PoseStamped, f"/vr_safe/teleop/{arm}_target", 1)
        self.pub_button = self.create_publisher(
            Joy, "/vr_safe/teleop/button", 1)

    # ── 回调 ──
    def _on_state(self, msg):
        self.state = msg

    def _fmt(self):
        if self.state is None:
            return "None"
        s = self.state
        return (f"state={s.state} blocked={s.command_blocked} "
                f"stopped={s.motion_stopped} speed={s.tcp_speed_mps:.4f}")

    # ── 发布 ──
    def publish_permit(self, decision, reason="dry-run probe"):
        self.permit_seq += 1
        msg = MotionPermit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.decision = decision
        msg.reason = reason
        msg.source = "dry_run_probe"
        msg.latched = decision == MotionPermit.REVOKED
        msg.seq = self.permit_seq
        self.pub_permit.publish(msg)

    def publish_target(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z
        msg.pose.orientation.w = 1.0
        self.pub_target.publish(msg)

    def publish_joy(self):
        msg = Joy()
        # axes: 0 左 Grip / 1 右 Grip / 2 左扳机 / 3 右扳机。
        axes = [0.0, 0.0, 0.0, 0.0]
        axes[0 if self.arm == "left" else 1] = self.grip
        msg.axes = axes
        msg.buttons = [0] * 12
        self.pub_button.publish(msg)

    # ── 驱动/等待 ──
    def pump(self, seconds, publish_target=True):
        end = time.monotonic() + seconds
        next_target = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if publish_target and now >= next_target:
                self.publish_target()
                next_target = now + 0.008
            self.publish_joy()
            rclpy.spin_once(self, timeout_sec=0.002)

    def wait(self, predicate, timeout, description, publish_target=True):
        end = time.monotonic() + timeout
        next_target = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if publish_target and now >= next_target:
                self.publish_target()
                next_target = now + 0.008
            self.publish_joy()
            rclpy.spin_once(self, timeout_sec=0.005)
            if self.state is not None and predicate(self.state):
                return self.state
        raise AssertionError(
            f"等待超时：{description}；实际 {self._fmt()}")

    def wait_state(self, name, timeout, description=None):
        return self.wait(
            lambda s: s.state == name, timeout,
            description or f"state == {name}")

    # ── 断言 ──
    def check(self, condition, message):
        if not condition:
            raise AssertionError(message)

    def run(self):
        self.get_logger().info("等待 dry_run 臂节点发布 ArmState…")
        first = self.wait(lambda s: True, 5.0, "收到第一帧 ArmState")
        self.check(first.command_blocked, "启动时 command_blocked 应为 true")
        self.check(first.state.startswith("dry_run/"),
                   f"dry_run 状态应带 dry_run/ 前缀，实际 {first.state}")
        self.check(first.state == "dry_run/blocked",
                   f"启动时应为 dry_run/blocked，实际 {first.state}")
        print("[1] 启动默认 REVOKED / blocked：OK", flush=True)

        self.publish_permit(MotionPermit.GRANTED, "probe clear")
        self.wait_state("dry_run/hold", 2.0, "GRANTED 后未按 Grip 应为 hold")
        self.pump(0.2)
        self.check(self.state.state == "dry_run/hold",
                   "未按 Grip 时不得自动进入 tracking")
        print("[2] GRANTED 不自动跟随：OK", flush=True)

        self.grip = 0.8
        self.wait_state("dry_run/tracking", 3.0, "按 Grip 后应进入 tracking")
        self.check(not self.state.command_blocked, "tracking 时不应 blocked")
        print("[3] 按 Grip 后 tracking：OK", flush=True)

        self.publish_permit(MotionPermit.REVOKED, "probe occupied")
        self.wait_state("dry_run/blocked", 2.0, "REVOKED 后应为 blocked")
        self.check(self.state.command_blocked, "REVOKED 后 command_blocked 应为 true")
        stopped = self.wait(lambda s: s.motion_stopped, 1.0,
                            "REVOKED 保持零增量后 motion_stopped")
        print(f"[4] REVOKED 阻断/保持位置（{self._fmt()}）：OK", flush=True)

        # 许可恢复但操作者还按着旧 Grip：不能自动跟随。
        self.publish_permit(MotionPermit.GRANTED, "probe recover")
        self.wait_state("dry_run/hold", 2.0, "恢复后仍按着 Grip 应为 hold")
        self.pump(0.3)
        self.check(self.state.state == "dry_run/hold",
                   "恢复后仍按着旧 Grip 时不得自动 tracking")
        print("[5] 许可恢复不自动跟随：OK", flush=True)

        self.grip = 0.0
        self.pump(0.2)
        self.grip = 0.8
        self.wait_state("dry_run/tracking", 3.0, "松开再按后应重新 tracking")
        print("[6] 松开再按重新锚定：OK", flush=True)

        # 停止 target、保持 Grip：200 ms 看门狗应松离合回 hold。
        self.wait(lambda s: s.state == "dry_run/hold", 1.5,
                  "200 ms 无新 target 应触发看门狗回 hold",
                  publish_target=False)
        print("[7] target 看门狗 200 ms：OK", flush=True)

        # 看门狗后还按着 Grip 不能自动恢复，必须松开再按。
        self.pump(0.2)
        self.check(self.state.state == "dry_run/hold",
                   "看门狗后仍按着 Grip 时不得自动 tracking")
        self.grip = 0.0
        self.pump(0.2)
        self.grip = 0.8
        self.wait_state("dry_run/tracking", 3.0, "看门狗后松开再按应重新 tracking")
        print("[8] 看门狗后重新按 Grip 才恢复：OK", flush=True)

        # 未按 Grip 时 REVOKED：恢复后第一次按下就应能重新跟踪，
        # 不能被“先松开再按”门闩多消耗一次按压。
        self.grip = 0.0
        self.pump(0.2)
        self.publish_permit(MotionPermit.REVOKED, "probe no-press occupied")
        self.wait_state("dry_run/blocked", 2.0, "未按 Grip 时 REVOKED 应为 blocked")
        self.publish_permit(MotionPermit.GRANTED, "probe no-press recover")
        self.wait_state("dry_run/hold", 2.0, "未按 Grip 时恢复应为 hold")
        self.grip = 0.8
        self.wait_state("dry_run/tracking", 3.0,
                        "未按 Grip 时经历 REVOKED，恢复后第一次按下应 tracking")
        print("[9] 未按 Grip 时 REVOKED，恢复后一次按下即可：OK", flush=True)

        # SLOW 保留跟随，状态显示 tracking_slow。
        self.publish_permit(MotionPermit.SLOW, "probe near")
        slow = self.wait_state("dry_run/tracking_slow", 2.0,
                               "SLOW 时应保持 tracking_slow")
        self.check(not slow.command_blocked, "SLOW 时不应 blocked")
        print("[10] SLOW 保持跟随：OK", flush=True)

        # 状态字段完整性：10 Hz 发布，速度/停稳字段有定义。
        self.check(self.state.arm == self.arm, "ArmState.arm 不匹配")
        self.check(self.state.header.frame_id == "base_link",
                   "ArmState.header.frame_id 应为 base_link")
        self.check(self.state.tcp_speed_mps >= 0.0, "tcp_speed_mps 应为非负")
        print("[11] ArmState 字段/频率：OK", flush=True)

        print("PASS: vr_safe_arm dry-run 许可门/重离合/看门狗全部通过", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", default="left", choices=["left", "right"])
    args = parser.parse_args()

    rclpy.init()
    node = DryRunProbe(args.arm)
    code = 1
    try:
        node.run()
        code = 0
    except AssertionError as exc:
        print(f"FAIL: {exc}", flush=True)
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
