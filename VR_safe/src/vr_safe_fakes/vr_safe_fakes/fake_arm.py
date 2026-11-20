"""Fake 臂执行端：只订阅许可，不碰 JAKA SDK。用于 dry-run 和测试。"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from vr_safe_interfaces.msg import ArmState, MotionPermit


class FakeArm(Node):
    def __init__(self):
        super().__init__("fake_arm")
        self.declare_parameter("arm", "left")
        self.arm = self.get_parameter("arm").value
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(MotionPermit, "/vr_safe/permit", self._on_permit, qos)
        self.pub = self.create_publisher(ArmState, f"/vr_safe/arm/{self.arm}/state", 10)
        self.declare_parameter("dry_run", True)
        self.state = "dry_run"
        self.blocked = False
        self.stopped = True
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"fake_arm[{self.arm}] 已启动（dry_run）")

    def _on_permit(self, msg: MotionPermit):
        self.blocked = msg.decision == MotionPermit.REVOKED
        self.stopped = msg.decision == MotionPermit.REVOKED
        self.state = {MotionPermit.GRANTED: "run",
                      MotionPermit.SLOW: "slow",
                      MotionPermit.REVOKED: "hold"}.get(msg.decision, "hold")

    def _tick(self):
        m = ArmState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.arm = self.arm
        m.command_blocked = bool(self.blocked)
        m.motion_stopped = bool(self.stopped)
        m.tcp_speed_mps = 0.0
        m.state = self.state
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FakeArm()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
