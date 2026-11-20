"""ROS 包装：把观察/心跳/服务翻译成 SafetyStateMachine 的调用。"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_srvs.srv import Trigger

from vr_safe_interfaces.msg import MotionPermit, SafetyObservation, TeleopHeartbeat
from vr_safe_interfaces.srv import ConfirmResume

from .audit import AuditLog
from .config import config_hash, load_config
from .state import SafetyStateMachine

_STATE = {
    SafetyObservation.CLEAR: "CLEAR",
    SafetyObservation.NEAR: "NEAR",
    SafetyObservation.OCCUPIED: "OCCUPIED",
    SafetyObservation.UNKNOWN: "UNKNOWN",
    SafetyObservation.ESTOP: "ESTOP",
}
_DECISION = {
    "GRANTED": MotionPermit.GRANTED,
    "SLOW": MotionPermit.SLOW,
    "REVOKED": MotionPermit.REVOKED,
}


class SupervisorNode(Node):
    def __init__(self):
        super().__init__("vr_safe_supervisor")
        self.declare_parameter("config", "")
        path = self.get_parameter("config").value
        self.cfg = load_config(path)
        safety = self.cfg["safety"]
        topics = self.cfg["topics"]

        self.machine = SafetyStateMachine(
            observation_timeout_s=safety["observation_timeout_s"],
            heartbeat_timeout_s=safety["heartbeat_timeout_s"],
            require_teleop_heartbeat=safety["require_teleop_heartbeat"],
            required_sources=tuple(safety["required_sources"]),
            teleop_source=safety["teleop_source"],
            occupied_debounce_frames=safety["occupied_debounce_frames"],
        )
        self.audit = AuditLog(safety["audit_path"])
        self.audit.event("supervisor_start",
                         config_hash=config_hash(self.cfg),
                         required_sources=safety["required_sources"],
                         observation_timeout_s=safety["observation_timeout_s"],
                         heartbeat_timeout_s=safety["heartbeat_timeout_s"],
                         occupied_debounce_frames=safety["occupied_debounce_frames"])

        permit_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_permit = self.create_publisher(MotionPermit, topics["permit"], permit_qos)
        self.create_subscription(
            SafetyObservation, topics["observation"], self._on_observation, 10)
        self.create_subscription(
            TeleopHeartbeat, topics["heartbeat"], self._on_heartbeat, 10)
        self.create_service(
            ConfirmResume, topics["confirm_resume"], self._on_confirm)
        self.create_service(Trigger, topics["clear_estop"], self._on_clear_estop)

        self._last_pub_seq = None
        self._last_pub_t = 0.0
        self._last_audit_seq = -1
        period = 1.0 / float(safety["publish_rate_hz"])
        self._period = period
        self.create_timer(period, self._tick)
        self.get_logger().info(
            "supervisor 已启动：required=%s，观察超时 %.2fs，心跳超时 %.2fs"
            % (safety["required_sources"],
               safety["observation_timeout_s"],
               safety["heartbeat_timeout_s"]))

    # ── 输入 ──
    def _on_observation(self, msg: SafetyObservation):
        state = _STATE.get(msg.state)
        if state is None:
            self.get_logger().warn(f"忽略未知观察状态 {msg.state}")
            return
        source = msg.source_id or "camera_top"
        now = time.monotonic()
        if state == "ESTOP":
            p = self.machine.press_estop(now, source_id=source)
            self.audit.event("estop_pressed", source=source,
                             stamp=_stamp(msg.header.stamp))
        else:
            p = self.machine.observe(source, state, now, msg.confidence)
            self.audit.event("observation", source=source,
                             zone=msg.zone_id, state=state,
                             confidence=float(msg.confidence),
                             evidence=int(msg.evidence),
                             detail_json=msg.detail_json[:2000],
                             stamp=_stamp(msg.header.stamp))
        self._publish(p, force=True)

    def _on_heartbeat(self, msg: TeleopHeartbeat):
        p = self.machine.heartbeat(msg.source_id or "teleop", time.monotonic())
        self._publish(p)

    def _on_confirm(self, req, res):
        accepted, message, p = self.machine.confirm(
            req.confirmed_by, time.monotonic(), req.note)
        self.audit.event("confirm_resume", accepted=bool(accepted),
                         confirmed_by=req.confirmed_by, note=req.note,
                         message=message, permit=_permit_dict(p))
        self._publish(p, force=True)
        res.accepted = bool(accepted)
        res.message = message
        return res

    def _on_clear_estop(self, _req, res):
        p = self.machine.clear_estop(time.monotonic())
        self.audit.event("estop_button_cleared", permit=_permit_dict(p))
        self._publish(p, force=True)
        res.success = True
        res.message = "按钮锁存已清除；许可恢复仍需 confirm_resume"
        return res

    # ── 周期 ──
    def _tick(self):
        p = self.machine.tick(time.monotonic())
        self._publish(p)

    def _publish(self, p, force=False):
        now = time.monotonic()
        if not force and p.seq == self._last_pub_seq and \
                (now - self._last_pub_t) < max(0.5, self._period):
            return
        m = MotionPermit()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.decision = _DECISION[p.decision]
        m.reason = p.reason
        m.source = p.source
        m.latched = bool(p.latched)
        m.seq = int(p.seq)
        self.pub_permit.publish(m)
        if p.seq != self._last_audit_seq:
            self.audit.event("permit_change", permit=_permit_dict(p))
            self._last_audit_seq = p.seq
        self._last_pub_seq = p.seq
        self._last_pub_t = now

    def shutdown(self):
        now = time.monotonic()
        p = self.machine.press_estop(now, source_id="supervisor_shutdown")
        self.audit.event("supervisor_shutdown", permit=_permit_dict(p))
        try:
            self._publish(p, force=True)
        except Exception:
            pass  # 进程退出时 context 可能已失效，不能再发
        self.audit.close()


def _stamp(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _permit_dict(p):
    return {"decision": p.decision, "reason": p.reason, "source": p.source,
            "latched": bool(p.latched), "seq": int(p.seq)}


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SupervisorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
