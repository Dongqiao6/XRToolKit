"""Fake 底盘执行端：只订阅许可，不碰 AMR SDK。"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from vr_safe_interfaces.msg import ChassisState, MotionPermit


class FakeChassis(Node):
    def __init__(self):
        super().__init__("fake_chassis")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(MotionPermit, "/vr_safe/permit", self._on_permit, qos)
        self.pub = self.create_publisher(ChassisState, "/vr_safe/chassis/state", 10)
        self.state = "dry_run"
        self.blocked = False
        self.hold = True
        self.create_timer(0.1, self._tick)
        self.get_logger().info("fake_chassis 已启动（dry_run）")

    def _on_permit(self, msg: MotionPermit):
        self.blocked = msg.decision == MotionPermit.REVOKED
        self.hold = msg.decision == MotionPermit.REVOKED
        self.state = {MotionPermit.GRANTED: "manual",
                      MotionPermit.SLOW: "slow",
                      MotionPermit.REVOKED: "hold"}.get(msg.decision, "hold")

    def _tick(self):
        m = ChassisState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.state = self.state
        m.command_blocked = bool(self.blocked)
        m.hold_active = bool(self.hold)
        m.linear_mps = 0.0
        m.angular_rps = 0.0
        m.detail_json = json.dumps({"dry_run": True})
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FakeChassis()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
