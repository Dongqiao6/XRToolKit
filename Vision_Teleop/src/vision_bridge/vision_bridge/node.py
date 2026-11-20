"""ROS 2 node: D435 -> pose/hand landmarks -> base-frame targets -> /vision topics.

Design notes
------------
* Perception runs in its own thread at camera rate (30 Hz).
* Publishing runs on a ROS timer at 125 Hz: it rate-limits the target and
  republishes it, so the downstream ``rc_ctrl`` node receives the same smooth
  increment stream it was designed for.
* Clutch semantics live in :mod:`vision_bridge.clutch`; this module only turns
  the decision into grip values and target positions.
"""

from __future__ import annotations

import math
import os
import select
import sys
import threading
from contextlib import contextmanager
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy

from .camera import Frame, RealSenseSource
from .clutch import Clutch, Gesture, GestureFilter, HandObservation, Mode
from .config import AppConfig, load_config
from .hands import HandSample, HandTracker
from .mapping import (
    BodyFrameEstimator,
    Mapper,
    OneEuroFilter,
    depth_at,
    depth_nearest,
    deproject,
    tool_point,
)
from .overlay import (
    LEFT_ELBOW,
    LEFT_HIP,
    LEFT_INDEX,
    LEFT_SHOULDER,
    LEFT_WRIST,
    RIGHT_ELBOW,
    RIGHT_HIP,
    RIGHT_INDEX,
    RIGHT_SHOULDER,
    RIGHT_WRIST,
    Hud,
    draw_overlay,
)
from .pose import PoseResult, PoseTracker

@contextmanager
def quiet_native(enabled: bool):
    """Temporarily silence fd 2 while native model loaders print their banner."""
    if not enabled:
        yield
        return
    saved = os.dup(2)
    devnull = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull, 2)
        yield
    finally:
        os.dup2(saved, 2)
        os.close(saved)
        os.close(devnull)


@dataclass
class ArmState:
    gesture: Gesture = Gesture.NONE
    observation: HandObservation = field(default_factory=HandObservation)
    base_target: np.ndarray | None = None
    published: np.ndarray | None = None
    anchor_base: np.ndarray | None = None
    anchor_published: np.ndarray | None = None
    tool_px: np.ndarray | None = None
    last_seen: float = 0.0


class VisionBridge(Node):
    def __init__(self) -> None:
        super().__init__("vision_bridge")
        self.config: AppConfig = load_config()
        self.declare_parameter("left_target_topic", "/vision/rc_ctrl/left_target")
        self.declare_parameter("right_target_topic", "/vision/rc_ctrl/right_target")
        self.declare_parameter("joy_topic", "/vision/rc_ctrl/button")
        self.declare_parameter("window", self.config.vision.window.enabled)
        self.declare_parameter("publish_rate", self.config.vision.publish.rate)

        self.left_topic = self.get_parameter("left_target_topic").value
        self.right_topic = self.get_parameter("right_target_topic").value
        self.joy_topic = self.get_parameter("joy_topic").value
        self.show_window = bool(self.get_parameter("window").value)
        rate = float(self.get_parameter("publish_rate").value)

        qos = QoSProfile(depth=1)
        self.left_pub = self.create_publisher(PoseStamped, self.left_topic, qos)
        self.right_pub = self.create_publisher(PoseStamped, self.right_topic, qos)
        self.joy_pub = self.create_publisher(Joy, self.joy_topic, qos)

        self.mapper = Mapper(self.config.mapping, self.config.safety)
        self.body_frame = (
            BodyFrameEstimator(self.config.mapping.body_frame_alpha)
            if self.config.mapping.use_body_frame
            else None
        )
        self.clutch = Clutch(self.config.safety)
        self.left = ArmState()
        self.right = ArmState()
        self.left_filter = self._new_filter()
        self.right_filter = self._new_filter()

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._smoother_left = OneEuroFilter(self.config.vision.smoothing)
        self._smoother_right = OneEuroFilter(self.config.vision.smoothing)
        self._fps = 0.0
        self._latency_ms = 0.0
        self._hud = Hud()
        self._last_frame_time = 0.0
        self._perception: threading.Thread | None = None
        self._keyboard: threading.Thread | None = None
        self._window_ready = False
        self._last_bgr: np.ndarray | None = None
        self._hand_futures: dict[str, object] = {"left": None, "right": None}
        self._hand_last_submit = {"left": 0.0, "right": 0.0}
        self._hand_samples: dict[str, HandSample] = {"left": HandSample(), "right": HandSample()}
        self._observations = {"left": HandObservation(), "right": HandObservation()}
        self._engaged_any = False
        self._last_depth = {"left": 0.0, "right": 0.0}
        self._depth_history = {"left": [], "right": []}
        self._pixel_history = {"left": [], "right": []}
        self._depth_rejects = {"left": 0, "right": 0}
        self._last_target: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._last_pose: PoseResult | None = None
        self._left_hand: HandSample | None = None
        self._right_hand: HandSample | None = None

        self._timer = self.create_timer(1.0 / max(1.0, rate), self._publish)
        self.get_logger().info(
            f"publishing {self.left_topic} / {self.right_topic} / {self.joy_topic}"
        )

    def _new_filter(self) -> GestureFilter:
        safety = self.config.safety
        return GestureFilter(
            safety.pinch_close,
            safety.pinch_open,
            safety.fist_curl_enter,
            safety.fist_curl_exit,
            safety.gesture_debounce_s,
            safety.pinch_min_curl,
        )

    # -- lifecycle -------------------------------------------------------
    def stop_requested(self) -> bool:
        return self._stop.is_set()

    def start(self) -> None:
        self._perception = threading.Thread(target=self._perception_loop, daemon=True)
        self._perception.start()
        if not self.show_window and sys.stdin.isatty():
            self._keyboard = threading.Thread(target=self._stdin_loop, daemon=True)
            self._keyboard.start()

    def shutdown(self) -> None:
        self._stop.set()
        if self._perception is not None:
            self._perception.join(timeout=2.0)

    def _stdin_loop(self) -> None:
        while not self._stop.is_set():
            ready, _, _ = select.select([sys.stdin], [], [], 0.2)
            if not ready:
                continue
            key = sys.stdin.read(1)
            if key == "q":
                self._stop.set()
            elif key == "\x1b":
                self._on_emergency()

    def _on_emergency(self) -> None:
        with self._lock:
            self.clutch.toggle_emergency()
        self.get_logger().warning("emergency stop: arms frozen, re-arm with both fists")

    # -- perception ------------------------------------------------------
    def _perception_loop(self) -> None:
        camera = RealSenseSource(self.config.vision.camera)
        pose_tracker = None
        left_tracker = None
        right_tracker = None
        pool = ThreadPoolExecutor(max_workers=2)
        try:
            with quiet_native(self.config.vision.quiet_native):
                pose_tracker = PoseTracker(
                    self.config.path(self.config.vision.pose.model),
                    self.config.vision.pose.min_detection,
                    self.config.vision.pose.min_tracking,
                )
                left_tracker = HandTracker(
                    self.config.path(self.config.vision.hand.model),
                    self.config.vision.hand.crop_size,
                    self.config.vision.hand.min_detection,
                    self.config.vision.hand.min_presence,
                )
                right_tracker = HandTracker(
                    self.config.path(self.config.vision.hand.model),
                    self.config.vision.hand.crop_size,
                    self.config.vision.hand.min_detection,
                    self.config.vision.hand.min_presence,
                )

            # Re-open the camera after a USB blip instead of dying: a D435 on a
            # busy hub can drop off the bus and come back a second later.
            while not self._stop.is_set() and rclpy.ok():
                try:
                    with quiet_native(self.config.vision.quiet_native):
                        camera.start()
                except Exception as exc:
                    self.get_logger().error(f"camera unavailable: {exc}")
                    with self._lock:
                        self.clutch.toggle_emergency()
                    self._sleep_unless_stopped(2.0)
                    continue

                intr = camera._intrinsics
                self.get_logger().info(
                    f"camera {camera.serial} {intr.width}x{intr.height} "
                    f"fx={intr.fx:.1f} fy={intr.fy:.1f} cx={intr.cx:.1f} cy={intr.cy:.1f}"
                )
                self.get_logger().info("camera and models ready")
                try:
                    self._consume(camera, pose_tracker, left_tracker, right_tracker, pool)
                except Exception as exc:  # stream dropped mid-run
                    self.get_logger().error(f"stream stopped: {exc}")
                    with self._lock:
                        self.clutch.toggle_emergency()
                finally:
                    camera.stop()
                if not self._stop.is_set():
                    self.get_logger().warning("reopening the camera in 2 s")
                    self._sleep_unless_stopped(2.0)
        finally:
            pool.shutdown(wait=False)
            for tracker in (pose_tracker, left_tracker, right_tracker):
                if tracker is not None:
                    tracker.close()
            camera.stop()
            if self.show_window:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass

    def _sleep_unless_stopped(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while not self._stop.is_set() and time.monotonic() < deadline:
            time.sleep(0.05)

    def _consume(self, camera, pose_tracker, left_tracker, right_tracker, pool) -> None:
        while not self._stop.is_set() and rclpy.ok():
            loop_start = time.monotonic()
            frame = camera.read()
            if frame is None:
                continue
            self._process(frame, pose_tracker, left_tracker, right_tracker, pool)
            elapsed = time.monotonic() - loop_start
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / max(elapsed, 1e-3))
            if self.show_window:
                self._show()

    def _process(
        self,
        frame: Frame,
        pose_tracker: PoseTracker,
        left_tracker: HandTracker,
        right_tracker: HandTracker,
        pool: ThreadPoolExecutor,
    ) -> None:
        self._last_bgr = frame.bgr
        intrinsics = frame.intrinsics
        height, width = frame.bgr.shape[:2]
        pose = pose_tracker.process(frame.bgr, int(frame.stamp * 1000.0))

        dt = time.monotonic() - self._last_frame_time if self._last_frame_time else 0.0
        self._last_frame_time = time.monotonic()

        min_visibility = self.config.vision.pose.min_visibility

        if self.body_frame is not None and pose.present and not self._engaged_any:
            points = {
                index: self._point3d(frame, pose, index, min_visibility)
                for index in (LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP)
            }
            if points[LEFT_SHOULDER] is not None and points[RIGHT_SHOULDER] is not None:
                rotation = self.body_frame.update(
                    points[LEFT_SHOULDER],
                    points[RIGHT_SHOULDER],
                    points[LEFT_HIP],
                    points[RIGHT_HIP],
                )
                self.mapper.use_body_frame(rotation)
        wrists: dict[str, np.ndarray | None] = {"left": None, "right": None}
        elbows: dict[str, np.ndarray | None] = {"left": None, "right": None}
        centres: dict[str, np.ndarray | None] = {"left": None, "right": None}
        if pose.present:
            for side, wrist_index, elbow_index, hand_index in (
                ("left", LEFT_WRIST, LEFT_ELBOW, LEFT_INDEX),
                ("right", RIGHT_WRIST, RIGHT_ELBOW, RIGHT_INDEX),
            ):
                if pose.visible(wrist_index, min_visibility):
                    wrists[side] = pose.point(wrist_index, width, height)
                    if pose.visible(hand_index, min_visibility):
                        hand_px = pose.point(hand_index, width, height)
                        if hand_px is not None:
                            # Centre the crop on the palm, not the wrist joint.
                            centres[side] = wrists[side] + 0.5 * (hand_px - wrists[side])
                if pose.visible(elbow_index, min_visibility):
                    elbows[side] = pose.point(elbow_index, width, height)

        # Hand inference runs asynchronously: it only drives the clutch, while
        # the position path needs nothing but pose + depth. Blocking on it here
        # would cost 35-50 ms of latency on every command.
        for side in ("left", "right"):
            future = self._hand_futures[side]
            if future is not None and future.done():
                sample = future.result()
                self._hand_futures[side] = None
                self._hand_samples[side] = sample
                filt = self.left_filter if side == "left" else self.right_filter
                gesture = filt.update(
                    sample.present, sample.pinch_ratio, sample.curl, pinch_m=sample.pinch_m
                )
                self._observations[side] = HandObservation(
                    gesture=gesture,
                    pinch_ratio=sample.pinch_ratio,
                    pinch_m=sample.pinch_m,
                    curl=sample.curl,
                    present=sample.present,
                )
            if wrists[side] is None:
                filt = self.left_filter if side == "left" else self.right_filter
                filt.update(False, float("nan"), float("nan"))
                self._observations[side] = HandObservation()
                self._hand_futures[side] = None
            elif (
                self._hand_futures[side] is None
                and time.monotonic() - self._hand_last_submit[side]
                >= self.config.vision.hand.interval_s
            ):
                tracker = left_tracker if side == "left" else right_tracker
                self._hand_last_submit[side] = time.monotonic()
                self._hand_futures[side] = pool.submit(
                    tracker.process, frame.bgr, wrists[side], centres[side]
                )

        tool_targets: dict[str, np.ndarray | None] = {"left": None, "right": None}
        tool_pixels: dict[str, np.ndarray | None] = {"left": None, "right": None}
        for side in ("left", "right"):
            smoother = self._smoother_left if side == "left" else self._smoother_right
            wrist_px = wrists[side]
            if wrist_px is None:
                smoother.reset()
                continue
            pixel = self._stable_pixel(side, wrist_px)
            depth = depth_nearest(
                frame.depth_m, int(pixel[0]), int(pixel[1]),
                self.config.vision.camera.depth_window,
            )
            depth = self._stable_depth(side, depth)
            depth = self._gate_depth(side, depth)
            if depth <= 0.0:
                smoother.reset()
                self._last_target[side] = None
                continue
            wrist_cam = deproject(pixel[0], pixel[1], depth, intrinsics)
            elbow_cam = None
            elbow_px = elbows[side]
            if elbow_px is not None:
                elbow_depth = depth_at(
                        frame.depth_m, int(elbow_px[0]), int(elbow_px[1]),
                        self.config.vision.camera.depth_window,
                    )
                if elbow_depth > 0.0:
                    elbow_cam = deproject(elbow_px[0], elbow_px[1], elbow_depth, intrinsics)
            tool_cam = tool_point(wrist_cam, elbow_cam, self.config.mapping.tool_offset_m)
            target = self.mapper.clamp(self.mapper.to_base(tool_cam))
            target = self._apply_deadband(side, target)
            target = self._limit_target_step(side, target)
            tool_targets[side] = smoother(target, dt)
            tool_pixels[side] = wrist_px

        hands = self._hand_samples
        left_obs, right_obs = self._observations["left"], self._observations["right"]

        with self._lock:
            output = self.clutch.update(dt, left_obs, right_obs)
            self._update_arm(self.left, output.left_engaged, tool_targets["left"])
            self._update_arm(self.right, output.right_engaged, tool_targets["right"])
            self.left.observation = left_obs
            self.right.observation = right_obs
            self.left.gesture = output.left_gesture
            self.right.gesture = output.right_gesture
            self.left.tool_px = tool_pixels["left"]
            self.right.tool_px = tool_pixels["right"]
            self._last_pose = pose
            self._left_hand = hands["left"]
            self._right_hand = hands["right"]
            self._engaged_any = output.left_engaged or output.right_engaged
            self._latency_ms = (time.monotonic() - frame.stamp) * 1000.0
            self._hud = Hud(
                prompt=output.prompt,
                mode=output.mode.value,
                left_engaged=output.left_engaged,
                right_engaged=output.right_engaged,
                left_gesture=output.left_gesture.value,
                right_gesture=output.right_gesture.value,
                left_ratio=left_obs.pinch_ratio,
                right_ratio=right_obs.pinch_ratio,
                left_depth=self._last_depth["left"],
                right_depth=self._last_depth["right"],
                left_pinch_m=left_obs.pinch_m,
                right_pinch_m=right_obs.pinch_m,
                left_curl=left_obs.curl,
                right_curl=right_obs.curl,
                fps=self._fps,
                latency_ms=self._latency_ms,
                frozen=output.frozen,
            )

    def _stable_pixel(self, side: str, pixel: np.ndarray) -> np.ndarray:
        """Median of the last three wrist pixels while the hand is still.

        Vertical jitter comes mostly from the landmark's image-y noise. A
        motion gate keeps the median only when the three samples agree, so real
        movement is never delayed.
        """
        history = self._pixel_history[side]
        history.append(np.asarray(pixel, dtype=np.float64))
        del history[:-3]
        if len(history) < 3:
            return np.asarray(pixel, dtype=np.float64)
        stacked = np.stack(history)
        median = np.median(stacked, axis=0)
        spread = float(np.max(np.linalg.norm(stacked - median, axis=1)))
        if spread < 8.0:  # ~1.7 cm at 2 m: jitter, not motion
            return median
        return np.asarray(pixel, dtype=np.float64)

    def _stable_depth(self, side: str, depth: float) -> float:
        """Median of the last three readings while the hand is nearly static.

        Static hands are where depth noise turns into visible arm jitter; while
        the hand moves the raw value is used so forward motion stays crisp.
        """
        history = self._depth_history[side]
        if depth > 0.0:
            history.append(depth)
            del history[:-3]
        if len(history) < 3:
            return depth
        if max(history) - min(history) < 0.05:
            return float(np.median(history))
        return depth

    def _gate_depth(self, side: str, depth: float) -> float:
        """Reject depth jumps, but let a genuine move win after a few frames."""
        previous = self._last_depth[side]
        if depth <= 0.0:
            return previous if previous > 0.0 else 0.0
        if previous > 0.0 and abs(depth - previous) > self.config.safety.depth_gate_m:
            self._depth_rejects[side] += 1
            if self._depth_rejects[side] < 3:
                return previous
        self._depth_rejects[side] = 0
        self._last_depth[side] = depth
        return depth

    def _apply_deadband(self, side: str, target: np.ndarray) -> np.ndarray:
        """Hold the previous target while the new one is within the deadband."""
        held = self._last_target[side]
        if held is not None:
            if float(np.linalg.norm(target - held)) < self.config.safety.target_deadband_m:
                return held
        return target

    def _limit_target_step(self, side: str, target: np.ndarray) -> np.ndarray:
        """Cap the per-frame target jump so a glitch cannot start a long slide."""
        previous = self._last_target[side]
        if previous is not None:
            delta = target - previous
            distance = float(np.linalg.norm(delta))
            limit = self.config.safety.max_target_step_m
            if distance > limit:
                target = previous + delta * (limit / distance)
        self._last_target[side] = target
        return target

    def _point3d(self, frame: Frame, pose: PoseResult, index: int, min_visibility: float):
        """Landmark index -> 3D point in the camera frame, or None."""
        if not pose.visible(index, min_visibility):
            return None
        pixel = pose.point(index, frame.bgr.shape[1], frame.bgr.shape[0])
        if pixel is None:
            return None
        depth = depth_at(
            frame.depth_m,
            int(pixel[0]),
            int(pixel[1]),
            self.config.vision.camera.depth_window,
        )
        if depth <= 0.0:
            return None
        return deproject(pixel[0], pixel[1], depth, frame.intrinsics)

    def _update_arm(self, arm: ArmState, engaged: bool, base_target: np.ndarray | None) -> None:
        if engaged and base_target is not None:
            if arm.anchor_base is None:
                arm.anchor_base = base_target
                arm.anchor_published = (
                    arm.published.copy() if arm.published is not None else base_target.copy()
                )
                arm.published = arm.anchor_published.copy()
            arm.base_target = arm.anchor_published + (base_target - arm.anchor_base)
        elif not engaged:
            arm.anchor_base = None
            arm.anchor_published = None
            arm.base_target = None

    def _show(self) -> None:
        with self._lock:
            pose = self._last_pose
            left_hand = self._left_hand
            right_hand = self._right_hand
            hud = self._hud
            left_tool = self.left.tool_px
            right_tool = self.right.tool_px
        frame = getattr(self, "_last_bgr", None)
        if frame is None:
            return
        canvas = draw_overlay(
            frame,
            pose.landmarks if pose and pose.present else None,
            left_hand,
            right_hand,
            left_tool,
            right_tool,
            hud,
        )
        cv2.imshow("Vision Teleop", canvas)
        self._window_ready = True
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self._stop.set()
        elif key == 27:
            self._on_emergency()

    # -- publishing ------------------------------------------------------
    def _publish(self) -> None:
        now = time.monotonic()
        with self._lock:
            engaged_left = self.left.anchor_base is not None
            engaged_right = self.right.anchor_base is not None
            left_target = self.left.base_target
            right_target = self.right.base_target
            left_pub = self._step(self.left, left_target)
            right_pub = self._step(self.right, right_target)
            self.left.published = left_pub
            self.right.published = right_pub

        self._publish_pose(self.left_pub, left_pub)
        self._publish_pose(self.right_pub, right_pub)
        self.joy_pub.publish(self._joy(engaged_left, engaged_right))

    def _step(self, arm: ArmState, target: np.ndarray | None) -> np.ndarray | None:
        """Exponential follower between the 125 Hz publisher and the target.

        A hard per-tick clamp makes the arm move in bursts: the target jumps
        once per camera frame, the clamp spends a few ticks catching up, then
        stops. An exponential approach spreads the same motion over several
        ticks, which is what removes the "start-stop" feel.
        """
        if target is None:
            return arm.published
        if arm.published is None:
            arm.published = target.copy()
            return arm.published
        dt = 1.0 / max(1.0, self.config.vision.publish.rate)
        tau = max(1e-3, self.config.safety.follow_tau_s)
        alpha = 1.0 - math.exp(-dt / tau)
        remaining = float(np.linalg.norm(target - arm.published))
        if remaining < 1e-3:  # within 1 mm: land exactly, no endless creep
            arm.published = target.copy()
            return arm.published
        delta = (target - arm.published) * alpha
        distance = float(np.linalg.norm(delta))
        limit = self.config.safety.max_step_m
        if distance > limit:
            delta *= limit / max(distance, 1e-9)
        arm.published = arm.published + delta
        return arm.published

    def _publish_pose(self, publisher, point: np.ndarray | None) -> None:
        if point is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.config.vision.publish.frame_id
        msg.pose.position.x = float(point[0])
        msg.pose.position.y = float(point[1])
        msg.pose.position.z = float(point[2])
        msg.pose.orientation.w = 1.0
        publisher.publish(msg)

    def _joy(self, left_engaged: bool, right_engaged: bool) -> Joy:
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        axes = [0.0] * 8
        axes[0] = 1.0 if left_engaged else 0.0
        axes[1] = 1.0 if right_engaged else 0.0
        msg.axes = axes
        msg.buttons = [0] * 12
        return msg


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = VisionBridge()
    node.start()
    try:
        # Poll instead of spin() so pressing q in the window really exits.
        while rclpy.ok() and not node.stop_requested():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # SIGTERM makes rclpy tear the context down first
        node.get_logger().warning(f"spin ended: {exc}")
    finally:
        try:
            node.shutdown()
            node.destroy_node()
        except Exception:  # context already torn down by a signal
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
