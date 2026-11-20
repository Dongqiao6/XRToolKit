"""实时相机画面：订阅感知压缩图/观察/许可/区域/确认结果并叠加显示。"""

import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from vr_safe_interfaces.msg import MotionPermit, SafetyObservation

from .overlay import OverlayState, render, smooth_persons


class CameraView(Node):
    def __init__(self):
        super().__init__("vr_safe_camera_view")
        self.state = OverlayState()
        self.zone = {}
        self.zone_raw = None
        self.persons = []
        self.confirm_until = 0.0
        self.window_open = False
        self.window = str(self.declare_parameter("window", "VR_safe camera").value)

        permit_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        zone_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(CompressedImage, "/vr_safe/camera/compressed",
                                 self._on_image, image_qos)
        self.create_subscription(MotionPermit, "/vr_safe/permit",
                                 self._on_permit, permit_qos)
        self.create_subscription(SafetyObservation, "/vr_safe/observation",
                                 self._on_observation, 10)
        self.create_subscription(String, "/vr_safe/perception/zone",
                                 self._on_zone, zone_qos)
        self.create_subscription(String, "/vr_safe/teleop/confirm_status",
                                 self._on_confirm, 10)
        self.create_timer(0.1, self._poll_key)
        self.get_logger().info("相机窗口已启动；q / Esc 关闭")

    def _on_permit(self, msg):
        decision = {0: "GRANTED", 1: "SLOW",
                    2: "REVOKED"}.get(msg.decision, "UNKNOWN")
        if decision != self.state.permit_decision:
            # 新许可状态到了，不要再用上一次“已恢复/被拒绝”的提示盖住它。
            self.state.confirm_status = ""
            self.confirm_until = 0.0
        self.state.permit_decision = decision
        self.state.permit_latched = bool(msg.latched)
        self.state.permit_reason = msg.reason
        self.state.permit_source = msg.source

    def _on_observation(self, msg):
        self.state.obs_state = {0: "CLEAR", 1: "NEAR", 2: "OCCUPIED",
                                3: "UNKNOWN", 4: "ESTOP"}.get(msg.state, "UNKNOWN")
        try:
            detail = json.loads(msg.detail_json or "{}")
        except Exception:
            detail = {}
        self.state.obs_reason = str(
            detail.get("why") or detail.get("reason") or "")
        self.persons = smooth_persons(
            self.persons, detail.get("persons") or [], alpha=0.35)

    def _on_zone(self, msg):
        self.zone_raw = msg.data
        try:
            self.zone = json.loads(msg.data or "{}")
        except Exception:
            self.zone = {}

    def _on_confirm(self, msg):
        self.state.confirm_status = msg.data
        self.confirm_until = time.monotonic() + 4.0

    def _poll_key(self):
        # 图像断流时也要能响应 q/Esc 和窗口关闭。
        if not self.window_open:
            return
        try:
            if cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                rclpy.shutdown()
                return
        except Exception:
            pass
        if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
            rclpy.shutdown()

    def _on_image(self, msg):
        try:
            buf = np.frombuffer(bytes(msg.data), np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        except Exception as exc:
            self.get_logger().warning(f"图像解码失败：{exc}")
            return
        if frame is None:
            return
        if time.monotonic() > self.confirm_until:
            self.state.confirm_status = ""
        vis = render(frame, self.state, self.zone, self.persons)
        cv2.imshow(self.window, vis)
        self.window_open = True


def main(args=None):
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("✗ 没有 DISPLAY/WAYLAND_DISPLAY，camera_view 无法开窗", file=sys.stderr)
        return 1
    rclpy.init(args=args)
    node = None
    try:
        node = CameraView()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
