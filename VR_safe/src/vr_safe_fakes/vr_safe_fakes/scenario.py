"""Dry-run 场景：通过 /vr_safe/permit 和 fake sink 验证安全闭环。

这是一个普通脚本（不是定时器节点），方便在 CI 里跑：
  ros2 run vr_safe_fakes scenario
退出码 0 = 全过，1 = 失败。
"""

import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_srvs.srv import Trigger

from vr_safe_interfaces.msg import (ArmState, ChassisState, MotionPermit,
                                    SafetyObservation, TeleopHeartbeat)
from vr_safe_interfaces.srv import ConfirmResume


class Scenario(Node):
    def __init__(self):
        super().__init__("dry_run_scenario")
        self.permit = None
        self.arm = {}
        self.chassis = None
        self.hb_seq = 0
        self.hb_enabled = True

        permit_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(MotionPermit, "/vr_safe/permit", self._on_permit,
                                 permit_qos)
        for arm in ("left", "right"):
            self.create_subscription(
                ArmState, f"/vr_safe/arm/{arm}/state",
                lambda m, a=arm: self.arm.__setitem__(a, m), 10)
        self.create_subscription(ChassisState, "/vr_safe/chassis/state",
                                 lambda m: setattr(self, "chassis", m), 10)

        self.pub_obs = self.create_publisher(SafetyObservation, "/vr_safe/observation", 10)
        self.pub_hb = self.create_publisher(
            TeleopHeartbeat, "/vr_safe/teleop/heartbeat", 10)
        self.cli_confirm = self.create_client(ConfirmResume, "/vr_safe/confirm_resume")
        self.cli_clear = self.create_client(Trigger, "/vr_safe/clear_estop")

    # ── 回调 ──
    def _on_permit(self, msg):
        self.permit = msg

    # ── 发布 ──
    def heartbeat(self):
        if not self.hb_enabled:
            return
        self.hb_seq += 1
        m = TeleopHeartbeat()
        m.header.stamp = self.get_clock().now().to_msg()
        m.source_id = "teleop"
        m.seq = self.hb_seq
        self.pub_hb.publish(m)

    def observe(self, state: str, detail: str = ""):
        m = SafetyObservation()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "camera_color_optical_frame"
        m.source_id = "camera_top"
        m.zone_id = "default"
        m.state = getattr(SafetyObservation, state)
        m.confidence = 1.0
        m.evidence = 0
        m.detail_json = detail
        self.pub_obs.publish(m)

    # ── 等待 ──
    def spin_for(self, seconds, obs=None, hb=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if hb:
                self.heartbeat()
            if obs is not None:
                self.observe(obs)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait(self, pred, timeout, obs=None, hb=True, what="条件"):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if hb:
                self.heartbeat()
            if obs is not None:
                self.observe(obs)
            rclpy.spin_once(self, timeout_sec=0.02)
            if pred():
                return True
        return False

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"服务不存在：{client.srv_name}")
        future = client.call_async(request)
        end = time.monotonic() + 3.0
        while time.monotonic() < end and not future.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise RuntimeError(f"服务超时：{client.srv_name}")
        return future.result()

    # ── 断言 ──
    def check(self, cond, msg):
        if not cond:
            raise AssertionError(msg)

    def expect(self, decision, latched=None, source=None, timeout=2.0, obs=None):
        ok = self.wait(
            lambda: self.permit is not None
            and self.permit.decision == decision
            and (latched is None or self.permit.latched == latched)
            and (source is None or self.permit.source == source),
            timeout, obs=obs)
        self.check(ok, f"期望 decision={decision} latched={latched} source={source}，"
                       f"实际 {self._fmt()}")
        return self.permit

    def _fmt(self):
        p = self.permit
        if p is None:
            return "None"
        return f"decision={p.decision} latched={p.latched} source={p.source} reason={p.reason}"

    def run(self):
        self.get_logger().info("等待 supervisor / fake sink…")
        self.check(self.wait(lambda: self.permit is not None, 5.0),
                   "没有收到 /vr_safe/permit")
        self.expect(MotionPermit.REVOKED, latched=False)

        self.get_logger().info("[1] CLEAR + heartbeat → GRANTED")
        self.expect(MotionPermit.GRANTED, latched=False, obs="CLEAR")
        self.check(self.wait(lambda: self.arm.get("left") is not None
                             and self.chassis is not None, 2.0),
                   "fake sink 没有发状态")
        self.check(self.wait(lambda: not self.arm["left"].command_blocked, 2.0,
                             obs="CLEAR"), "GRANTED 时 fake arm 仍 blocked")
        self.check(self.wait(lambda: not self.chassis.command_blocked, 2.0,
                             obs="CLEAR"), "GRANTED 时 fake chassis 仍 blocked")

        self.get_logger().info("[2] NEAR → SLOW")
        self.expect(MotionPermit.SLOW, latched=False, obs="NEAR")

        self.get_logger().info("[3] OCCUPIED → REVOKED + latched")
        self.expect(MotionPermit.REVOKED, latched=True, obs="OCCUPIED")
        self.check(self.wait(lambda: self.arm["left"].command_blocked, 2.0,
                             obs="OCCUPIED"), "OCCUPIED 时 fake arm 没有 blocked")
        self.check(self.wait(lambda: self.chassis.hold_active, 2.0,
                             obs="OCCUPIED"), "OCCUPIED 时 fake chassis 没有 hold")

        self.get_logger().info("[4] 区域被占时 confirm 被拒")
        res = self.call(self.cli_confirm,
                        ConfirmResume.Request(confirmed_by="dry-run", note=""))
        self.check(not res.accepted, f"区域被占时 confirm 竟然接受：{res.message}")
        self.expect(MotionPermit.REVOKED, latched=True, obs="OCCUPIED")

        self.get_logger().info("[5] CLEAR 但仍锁存，直到 confirm")
        self.expect(MotionPermit.REVOKED, latched=True, obs="CLEAR")
        res = self.call(self.cli_confirm,
                        ConfirmResume.Request(confirmed_by="dry-run", note="clear"))
        self.check(res.accepted, f"区域清空后 confirm 被拒：{res.message}")
        self.expect(MotionPermit.GRANTED, latched=False, obs="CLEAR")

        self.get_logger().info("[6] 心跳丢失 → REVOKED + latched")
        self.hb_enabled = False
        self.expect(MotionPermit.REVOKED, latched=True, source="teleop",
                    timeout=2.0, obs="CLEAR")
        self.check(self.wait(lambda: self.arm["left"].command_blocked, 2.0,
                             obs="CLEAR", hb=False),
                   "心跳丢失时 fake arm 没有 blocked")
        res = self.call(self.cli_confirm,
                        ConfirmResume.Request(confirmed_by="dry-run", note=""))
        self.check(not res.accepted, "心跳丢失时 confirm 竟然接受")

        self.get_logger().info("[7] 心跳恢复 + confirm → GRANTED")
        self.hb_enabled = True
        self.spin_for(0.15, obs="CLEAR", hb=True)
        res = self.call(self.cli_confirm,
                        ConfirmResume.Request(confirmed_by="dry-run", note="hb back"))
        self.check(res.accepted, f"心跳恢复后 confirm 被拒：{res.message}")
        self.expect(MotionPermit.GRANTED, latched=False, obs="CLEAR")

        self.get_logger().info("[8] 系统急停 → REVOKED；clear_estop 后仍需 confirm")
        self.observe("ESTOP", "left joystick")
        self.expect(MotionPermit.REVOKED, latched=True, source="system_estop",
                    obs="CLEAR")
        res = self.call(self.cli_clear, Trigger.Request())
        self.check(res.success, f"clear_estop 失败：{res.message}")
        self.expect(MotionPermit.REVOKED, latched=True, obs="CLEAR")
        res = self.call(self.cli_confirm,
                        ConfirmResume.Request(confirmed_by="dry-run", note="estop cleared"))
        self.check(res.accepted, f"急停解除后 confirm 被拒：{res.message}")
        self.expect(MotionPermit.GRANTED, latched=False, obs="CLEAR")

        print("PASS: dry-run 安全闭环全部通过", flush=True)
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = Scenario()
    code = 1
    try:
        code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 0
    except AssertionError as exc:
        print(f"FAIL: {exc}", flush=True)
        code = 1
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}", flush=True)
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
