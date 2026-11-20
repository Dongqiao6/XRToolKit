#!/usr/bin/env python3
"""相机观察源节点：相机/录像 → 五档判定 → /vr_safe/observation。

启动顺序：读 perception.yaml 并严格校验 → 校验外参和地面 → 建保护区
（polygon/reach）→ 加载 MediaPipe 模型 → 开相机或回放。任何一步配置或
标定问题都拒绝启动；相机打不开或连续读帧失败不退出，而是以 5 Hz 发布
UNKNOWN 并按 camera_retry_s 重试，避免「感知进程没了」被误当成安全。

用法：
    ros2 run vr_safe_perception observation_node --config .../perception.yaml
    ros2 run vr_safe_perception observation_node --replay clips/xxx --allow-uncalibrated
"""

import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from vr_safe_interfaces.msg import SafetyObservation

from .config import ConfigError, load_config
from .mapping import (CLEAR, EV_LOST, NEAR, OCCUPIED_SAFE, UNKNOWN)
from .paths import default_perception_config
from .sources import CameraSource, ClipSource, SourceError

# 四值字符串 → SafetyObservation 里的 uint8 常量
_MSG_STATE = {
    CLEAR: SafetyObservation.CLEAR,
    NEAR: SafetyObservation.NEAR,
    OCCUPIED_SAFE: SafetyObservation.OCCUPIED,
    UNKNOWN: SafetyObservation.UNKNOWN,
}


def _parse_args(argv):
    ap = argparse.ArgumentParser(
        description="VR_safe 相机观察源：输出四值 SafetyObservation")
    ap.add_argument("--config", default=default_perception_config(),
                    help="perception.yaml 路径")
    ap.add_argument("--replay", default="",
                    help="录像片段目录（不传就开实时相机）")
    ap.add_argument("--allow-uncalibrated", action="store_true",
                    help="开发降级：外参缺失/ok:false/地面不匹配时用设计值继续")
    ap.add_argument("--backend", choices=["orbbec", "realsense", "auto"],
                    default=None, help="覆盖配置里的相机后端")
    ap.add_argument("--zone-mode", choices=["polygon", "reach"], default=None,
                    help="覆盖配置里的保护区类型")
    ap.add_argument("--zone-file", default=None,
                    help="覆盖 polygon 区域文件（相对路径按当前目录解析）")
    ap.add_argument("--no-hands", action="store_true",
                    help="不跑手部模型（回放对照时快一倍）")
    ap.add_argument("--replay-fast", action="store_true",
                    help="按最大速度回放，不等原始帧间隔")
    ap.add_argument("--replay-loop", action="store_true", help="循环回放")
    ap.add_argument("--max-frames", type=int, default=0,
                    help="处理这么多帧后退出（0 = 不限，回放调试用）")
    ap.add_argument("--publish-camera", action="store_true",
                    help="发布压缩彩色图给 camera_view（默认关，省 CPU）")
    ap.add_argument("--camera-topic", default="/vr_safe/camera/compressed")
    ap.add_argument("--camera-fps", type=float, default=15.0)
    ap.add_argument("--jpeg-quality", type=int, default=80)
    args, ros_args = ap.parse_known_args(argv)
    if args.max_frames < 0:
        ap.error("--max-frames 不能为负")
    return args, ros_args


class ObservationNode(Node):
    def __init__(self, cfg, publish_camera=False,
                 camera_topic="/vr_safe/camera/compressed",
                 camera_fps=15.0, jpeg_quality=80):
        super().__init__("vr_safe_observation")
        self.cfg = cfg
        self.publish_camera = bool(publish_camera)
        self.camera_fps = max(1.0, float(camera_fps))
        self.jpeg_quality = max(10, min(95, int(jpeg_quality)))
        self._last_camera_pub = 0.0
        self._zone = None
        self._zone_sent = False
        self.source_id = str(cfg["node"]["source_id"])
        self.zone_id = str(cfg["node"]["zone_id"])
        self.frame_id = "camera_" + str(cfg["calibration"]["camera_id"])
        self.publish_period = 1.0 / float(cfg["node"]["publish_rate_hz"])
        self.unknown_rate_hz = float(cfg["node"]["unknown_rate_hz"])
        self.retry_s = float(cfg["node"]["camera_retry_s"])
        self.max_consecutive_errors = int(
            cfg["node"]["camera_max_consecutive_errors"])

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(
            SafetyObservation, str(cfg["node"]["topic"]), qos)
        self.pub_camera = None
        if self.publish_camera:
            cam_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.pub_camera = self.create_publisher(
                CompressedImage, camera_topic, cam_qos)
        zone_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_zone = self.create_publisher(
            String, "/vr_safe/perception/zone", zone_qos)
        self._clock_t0 = time.monotonic()
        self._last_pub = 0.0
        self._published = 0
        self._frame_errors = 0
        self.cfg_max_frames = 0

    def set_zone(self, zone_result):
        """装配完成后注入保护区，第一帧拿到内参时发布像素坐标。"""
        self._zone = zone_result
        self._zone_sent = False

    def _publish_zone(self, intr):
        if self._zone is None or self._zone_sent:
            return
        mode = getattr(self._zone, "mode", "polygon")
        zone = getattr(self._zone, "zone", None)
        payload = {"mode": mode,
                   "height_m": float(self.cfg["zone"]["height_m"]),
                   "near_m": float(self.cfg["zone"]["near_m"])}
        try:
            if mode == "polygon" and zone is not None:
                pts = [intr.project(p) for p in zone.corners3d]
                if any(p is None for p in pts):
                    return
                payload["corners_px"] = [[float(u), float(v)] for u, v in pts]
            elif mode == "reach" and zone is not None:
                spheres = []
                for origin, radius in zone.spheres:
                    q = intr.project(origin)
                    if q is None or origin[2] <= 0.05:
                        continue
                    r_px = radius * intr.fx / max(origin[2], 1e-3)
                    spheres.append([float(q[0]), float(q[1]), float(r_px)])
                payload["spheres"] = spheres
        except Exception as exc:
            self.get_logger().warning(f"保护区像素投影失败：{exc}")
            return
        m = String()
        m.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.pub_zone.publish(m)
        self._zone_sent = True

    def _publish_camera(self, frame):
        if self.pub_camera is None:
            return
        now = time.monotonic()
        if (now - self._last_camera_pub) < (1.0 / self.camera_fps):
            return
        self._last_camera_pub = now
        try:
            import cv2
            ok, buf = cv2.imencode(
                ".jpg", frame.color,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        except Exception as exc:
            self.get_logger().warning(f"相机 JPEG 编码失败：{exc}")
            return
        if not ok:
            return
        m = CompressedImage()
        m.header.stamp = self._stamp()
        m.header.frame_id = self.frame_id
        m.format = "jpeg"
        m.data = buf.tobytes()
        self.pub_camera.publish(m)

    # ── 发布 ──
    def _stamp(self):
        return self.get_clock().now().to_msg()

    def publish_observation(self, obs):
        msg = SafetyObservation()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.frame_id
        msg.source_id = obs.source_id
        msg.zone_id = obs.zone_id
        msg.state = _MSG_STATE[obs.state]
        msg.confidence = float(min(1.0, max(0.0, obs.confidence)))
        msg.evidence = int(obs.evidence)
        msg.detail_json = obs.detail_json
        self.pub.publish(msg)
        self._published += 1

    def publish_unknown(self, reason, **extra):
        """相机不可用/出错时的兜底观察。detail 里写明原因，便于审计。"""
        detail = {
            "five_state": "看不清",
            "safe_state": UNKNOWN,
            "reason": str(reason),
            "evidence": EV_LOST,
            "source_id": self.source_id,
            "zone_id": self.zone_id,
            **extra,
        }
        msg = SafetyObservation()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.frame_id
        msg.source_id = self.source_id
        msg.zone_id = self.zone_id
        msg.state = SafetyObservation.UNKNOWN
        msg.confidence = 0.0
        msg.evidence = EV_LOST
        msg.detail_json = json.dumps(detail, ensure_ascii=False,
                                     allow_nan=False, separators=(",", ":"))
        self.pub.publish(msg)
        self._published += 1

    # ── 主循环 ──
    def run_source(self, pipeline, source, replay):
        """跑帧源直到结束/出错。返回 "end" 或 "shutdown"。"""
        sent = 0
        self._frame_errors = 0
        for frame in source.frames():
            if not rclpy.ok():
                return "shutdown"
            now = time.monotonic()
            t = float(frame.t) if replay else (now - self._clock_t0)

            try:
                obs = pipeline.step(frame, t)
            except Exception as exc:
                self._frame_errors += 1
                self.publish_unknown(f"单帧处理失败：{type(exc).__name__}: {exc}",
                                     frame_errors=self._frame_errors)
                if self._frame_errors >= self.max_consecutive_errors:
                    raise SourceError(
                        f"连续 {self._frame_errors} 帧处理失败：{exc}") from exc
                continue
            self._frame_errors = 0
            self._publish_zone(frame.intr)
            self._publish_camera(frame)

            # 发布限速：推理比配置频率快时不刷屏，慢时自然降频。
            if (now - self._last_pub) >= self.publish_period:
                self.publish_observation(obs)
                self._last_pub = now
            sent += 1
            if self.cfg_max_frames and sent >= self.cfg_max_frames:
                return "end"
        return "end"

    def wait_unknown(self, source, reason, retry=True, pipeline=None):
        """以 unknown_rate_hz 持续发 UNKNOWN；相机可重开时返回 True。"""
        period = 1.0 / self.unknown_rate_hz
        next_retry = time.monotonic() + self.retry_s
        warned = False
        while rclpy.ok():
            self.publish_unknown(reason, retry=bool(retry))
            if not warned:
                self.get_logger().error(
                    f"{reason}；以 {self.unknown_rate_hz:.1f} Hz 发布 UNKNOWN"
                    + (f"，每 {self.retry_s:.1f} s 重试相机" if retry else ""))
                warned = True
            now = time.monotonic()
            if retry and now >= next_retry:
                try:
                    source.close()
                    source.open()
                    if pipeline is not None:
                        # 重连后不能沿用故障前的可信历史，第一帧必须重新过质量门。
                        pipeline.reset()
                    self.get_logger().info("相机已重新打开，恢复观察")
                    return True
                except SourceError as exc:
                    reason = str(exc)
                    next_retry = now + self.retry_s
            time.sleep(period)
        return False


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    args, ros_args = _parse_args(argv)

    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        print(f"✗ perception 配置不合法：{exc}", file=sys.stderr)
        return 2

    # 命令行覆盖只改内存里的配置，不写回文件。
    if args.backend:
        cfg["camera"]["backend"] = args.backend
    if args.zone_mode:
        cfg["calibration"]["zone_mode"] = args.zone_mode
    if args.zone_file:
        cfg["calibration"]["zone_file"] = os.path.abspath(args.zone_file)

    init_args = None
    if ros_args:
        init_args = [sys.argv[0]] + ros_args
    rclpy.init(args=init_args)

    node = None
    runtime = None
    source = None
    try:
        node = ObservationNode(
            cfg, publish_camera=bool(args.publish_camera),
            camera_topic=str(args.camera_topic),
            camera_fps=float(args.camera_fps),
            jpeg_quality=int(args.jpeg_quality))
        node.cfg_max_frames = int(args.max_frames)

        # 感知装配（标定、区域、模型）失败属于启动失败，必须拒绝启动，
        # 不能进入 UNKNOWN 兜底——UNKNOWN 兜底只覆盖运行期相机故障。
        try:
            from .runtime import build_runtime
            runtime = build_runtime(
                cfg, args.config,
                allow_uncalibrated=bool(args.allow_uncalibrated),
                no_hands=bool(args.no_hands),
                log=lambda msg: node.get_logger().info(str(msg)))
        except Exception as exc:
            node.get_logger().error(f"感知启动失败，拒绝启动：{exc}")
            return 2

        if args.replay:
            try:
                source = ClipSource(
                    args.replay, loop=bool(args.replay_loop),
                    realtime=not bool(args.replay_fast),
                    max_frames=int(args.max_frames))
            except SourceError as exc:
                node.get_logger().error(str(exc))
                return 2
            mode = f"回放 {args.replay}"
        else:
            source = CameraSource(
                backend=cfg["camera"]["backend"],
                width=int(cfg["camera"]["width"]),
                height=int(cfg["camera"]["height"]),
                fps=int(cfg["camera"]["fps"]),
                max_consecutive_errors=node.max_consecutive_errors)
            mode = "实时相机"
        node.set_zone(runtime.zone)

        node.get_logger().info(
            f"观察源启动：{mode}，source_id={node.source_id}，"
            f"zone_id={node.zone_id}，区域={runtime.zone.mode}，"
            f"发布 {cfg['node']['topic']}")

        while rclpy.ok():
            try:
                result = node.run_source(runtime.pipeline, source,
                                         replay=bool(args.replay))
            except SourceError as exc:
                if args.replay:
                    node.get_logger().error(f"回放源失败：{exc}")
                    return 1
                if not node.wait_unknown(source, str(exc), retry=True,
                                         pipeline=runtime.pipeline):
                    break
                continue
            if result == "shutdown":
                break
            return 0
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        if source is not None:
            source.close()
        if runtime is not None:
            runtime.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
