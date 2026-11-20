"""RealSense capture: colour + depth aligned to colour."""

from __future__ import annotations

import time
from dataclasses import dataclass

import os

import cv2
import numpy as np

from .config import CameraConfig
from .mapping import Intrinsics

try:  # pragma: no cover - hardware dependency
    import pyrealsense2 as rs
except ImportError:  # pragma: no cover
    rs = None


@dataclass
class Frame:
    bgr: np.ndarray
    depth_m: np.ndarray
    intrinsics: Intrinsics
    stamp: float  # seconds, monotonic

    @property
    def size(self) -> tuple[int, int]:
        height, width = self.bgr.shape[:2]
        return width, height


class RealSenseSource:
    """Thin wrapper around ``pyrealsense2`` with depth aligned to colour."""

    def __init__(self, config: CameraConfig, serial_override: str | None = None):
        if rs is None:
            raise RuntimeError(
                "pyrealsense2 is not installed. Run scripts/setup_env.sh first."
            )
        if serial_override is not None:
            config = CameraConfig(
                serial=serial_override,
                color=config.color,
                depth=config.depth,
                log_level=config.log_level,
                depth_window=config.depth_window,
            )
        self.config = config
        self._pipeline = None
        self._align = None
        self._intrinsics: Intrinsics | None = None
        self._started = False
        self.serial = config.serial

    def start(self) -> None:
        cfg = self.config
        try:  # "sequence size exceeds remaining buffer" is benign SDK chatter
            level = getattr(rs.log_severity, cfg.log_level, rs.log_severity.error)
            if cfg.log_level == "none":
                # Send SDK messages to /dev/null instead of the terminal.
                rs.log_to_file(level, os.devnull)
            else:
                rs.log_to_console(level)
        except Exception:
            pass
        pipeline_cfg = rs.config()
        if cfg.serial:
            pipeline_cfg.enable_device(cfg.serial)
        pipeline_cfg.enable_stream(
            rs.stream.color, cfg.color.width, cfg.color.height, rs.format.bgr8, cfg.color.fps
        )
        pipeline_cfg.enable_stream(
            rs.stream.depth, cfg.depth.width, cfg.depth.height, rs.format.z16, cfg.depth.fps
        )
        self._pipeline = rs.pipeline()
        try:
            profile = self._pipeline.start(pipeline_cfg)
        except Exception as exc:
            self._pipeline = None
            available = list_devices()
            raise RuntimeError(
                f"camera start failed for serial {cfg.serial or '<first>'}: {exc}. "
                f"devices currently visible: {available or 'none'}. "
                "Another process may still hold the camera (pkill -f vision_bridge.node), "
                "or the requested streams are unsupported."
            ) from exc
        self._started = True
        device = profile.get_device()
        try:
            self.serial = str(device.get_info(rs.camera_info.serial_number))
        except Exception:  # device may not expose a serial
            self.serial = cfg.serial
        colour = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = colour.get_intrinsics()
        # RealSense names the principal point ppx/ppy; older bindings exposed
        # cx/cy aliases, so accept both.
        cx = float(getattr(intr, "ppx", None) if hasattr(intr, "ppx") else intr.cx)
        cy = float(getattr(intr, "ppy", None) if hasattr(intr, "ppy") else intr.cy)
        self._intrinsics = Intrinsics(
            fx=float(intr.fx),
            fy=float(intr.fy),
            cx=cx,
            cy=cy,
            width=int(intr.width),
            height=int(intr.height),
        )
        self._align = rs.align(rs.stream.color)

    def read(self, timeout_ms: int = 1000) -> Frame | None:
        if self._pipeline is None or self._intrinsics is None:
            raise RuntimeError("RealSenseSource.start() must be called first")
        # Drain whatever is queued and keep the newest frame: at 30 fps capture
        # and ~12 fps processing the queue would otherwise add 1-3 frames of
        # latency to every command.
        frames = None
        for _ in range(8):
            try:
                queued = self._pipeline.poll_for_frames()
            except Exception:
                break
            if queued is None:
                break
            try:
                empty = queued.size() == 0
            except Exception:
                empty = not bool(queued)
            if empty:
                break
            frames = queued
        if frames is None:
            frames = self._pipeline.wait_for_frames(timeout_ms)
        aligned = self._align.process(frames)
        colour = aligned.get_color_frame()
        depth = aligned.get_depth_frame()
        if not colour or not depth:
            return None
        bgr = np.asanyarray(colour.get_data())
        raw_depth = np.asanyarray(depth.get_data())
        depth_m = raw_depth.astype(np.float32) * 0.001
        # rs.align normally returns colour-sized depth; guard the rare case
        # where the driver hands back a different resolution.
        if depth_m.shape[:2] != bgr.shape[:2]:
            depth_m = cv2.resize(
                depth_m, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST
            )
        return Frame(
            bgr=bgr,
            depth_m=depth_m,
            intrinsics=self._intrinsics,
            stamp=time.monotonic(),
        )

    def stop(self) -> None:
        if self._pipeline is not None and self._started:
            try:
                self._pipeline.stop()
            except Exception:  # session already dropped by the driver
                pass
        self._pipeline = None
        self._started = False
        self._intrinsics = None

    def __enter__(self) -> "RealSenseSource":
        self.start()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.stop()


def list_devices() -> list[str]:
    """Return connected RealSense serial numbers, empty when none are visible."""
    if rs is None:
        return []
    ctx = rs.context()
    return [device.get_info(rs.camera_info.serial_number) for device in ctx.query_devices()]
