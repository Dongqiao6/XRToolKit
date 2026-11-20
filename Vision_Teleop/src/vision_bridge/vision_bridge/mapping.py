"""Geometry helpers: depth deprojection, wrist-to-tool offset, camera->base map."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import MappingConfig, SafetyConfig, SmoothingConfig


@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


def deproject(px: float, py: float, depth_m: float, k: Intrinsics) -> np.ndarray:
    """Back-project a pixel to a 3D point in the camera frame (metres).

    RealSense camera frame: +x right, +y down, +z forward.
    """
    return np.array(
        [
            (px - k.cx) * depth_m / k.fx,
            (py - k.cy) * depth_m / k.fy,
            depth_m,
        ],
        dtype=np.float64,
    )


def depth_at(depth_m: np.ndarray, px: int, py: int, window: int = 2) -> float:
    """Median depth around a pixel, ignoring zeros (invalid / no return)."""
    height, width = depth_m.shape[:2]
    x0, x1 = max(0, px - window), min(width, px + window + 1)
    y0, y1 = max(0, py - window), min(height, py + window + 1)
    patch = depth_m[y0:y1, x0:x1]
    valid = patch[patch > 0.0]
    if valid.size == 0:
        return 0.0
    return float(np.median(valid))


def depth_nearest(depth_m: np.ndarray, px: int, py: int, window: int = 4, band: float = 0.06) -> float:
    """Depth of the nearest surface around a pixel.

    A wrist pixel often straddles the arm and the background, and a plain
    median then jumps between the two. Taking the median of the nearest
    cluster (within ``band`` metres of the closest return) keeps the reading on
    the arm and rejects the background behind it.
    """
    height, width = depth_m.shape[:2]
    x0, x1 = max(0, px - window), min(width, px + window + 1)
    y0, y1 = max(0, py - window), min(height, py + window + 1)
    patch = depth_m[y0:y1, x0:x1]
    valid = patch[patch > 0.0]
    if valid.size == 0:
        return 0.0
    nearest = float(valid.min())
    cluster = valid[valid <= nearest + band]
    return float(np.median(cluster))


def tool_point(wrist: np.ndarray, elbow: np.ndarray | None, offset_m: float) -> np.ndarray:
    """Move the wrist point along the forearm so it approximates the palm.

    With no usable elbow the wrist is returned unchanged: a wrong offset is
    worse than none.
    """
    if offset_m == 0.0 or elbow is None:
        return wrist
    direction = wrist - elbow
    norm = float(np.linalg.norm(direction))
    if norm < 1e-6:
        return wrist
    return wrist + direction / norm * offset_m


class Mapper:
    """Map camera-frame points into the robot base frame."""

    def __init__(self, mapping: MappingConfig, safety: SafetyConfig):
        rotation = np.asarray(mapping.rotation, dtype=np.float64)
        if rotation.shape != (3, 3):
            raise ValueError("mapping.rotation must be 3x3")
        det = float(np.linalg.det(rotation))
        if abs(det) < 1e-6:
            raise ValueError("mapping.rotation is singular")
        self.rotation = rotation
        self.scale = float(mapping.scale)
        self.anchor = np.asarray(mapping.anchor_position, dtype=np.float64)
        self.axes_sign = np.asarray(mapping.axes_sign, dtype=np.float64)
        self.workspace_min = np.asarray(safety.workspace_min, dtype=np.float64)
        self.workspace_max = np.asarray(safety.workspace_max, dtype=np.float64)
        self.reference: np.ndarray | None = None
        self.static_rotation = self.rotation.copy()

    def set_reference(self, camera_point: np.ndarray) -> None:
        """Fix the camera-frame origin; called once when the first frame arrives."""
        self.reference = np.asarray(camera_point, dtype=np.float64).copy()

    def use_body_frame(self, rotation: np.ndarray | None) -> None:
        """Follow the operator's body frame, falling back to the static one."""
        self.rotation = self.static_rotation if rotation is None else rotation

    def transform(self, camera_delta: np.ndarray) -> np.ndarray:
        """Rotate a camera-frame *displacement* into the base frame."""
        rotated = self.rotation @ (np.asarray(camera_delta, dtype=np.float64) * self.scale)
        return self.axes_sign * rotated

    def to_base(self, camera_point: np.ndarray) -> np.ndarray:
        point = np.asarray(camera_point, dtype=np.float64)
        if self.reference is None:
            self.set_reference(point)
        return self.anchor + self.transform(point - self.reference)

    def clamp(self, point: np.ndarray) -> np.ndarray:
        return np.clip(point, self.workspace_min, self.workspace_max)


class OneEuroFilter:
    """One Euro filter for a 3D point.

    Removes the frame-to-frame jitter of MediaPipe landmarks without adding the
    constant lag of a plain low-pass: the cutoff rises with speed.
    """

    def __init__(self, config: SmoothingConfig):
        self.enabled = bool(config.enabled)
        self.min_cutoff = float(config.min_cutoff)
        self.beta = float(config.beta)
        self.derivative_cutoff = float(config.derivative_cutoff)
        self._value: np.ndarray | None = None
        self._derivative = np.zeros(3)

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * max(cutoff, 1e-6))
        return 1.0 / (1.0 + tau / max(dt, 1e-6))

    def reset(self) -> None:
        """Drop history so the next sample is taken as-is."""
        self._value = None
        self._derivative = np.zeros(3)

    def __call__(self, point: np.ndarray, dt: float) -> np.ndarray:
        value = np.asarray(point, dtype=np.float64)
        if not self.enabled:
            return value
        if self._value is None or dt <= 0.0:
            self._value = value.copy()
            self._derivative = np.zeros(3)
            return self._value.copy()
        derivative = (value - self._value) / dt
        alpha_d = self._alpha(self.derivative_cutoff, dt)
        self._derivative = alpha_d * derivative + (1.0 - alpha_d) * self._derivative
        cutoff = self.min_cutoff + self.beta * float(np.linalg.norm(self._derivative))
        alpha = self._alpha(cutoff, dt)
        self._value = alpha * value + (1.0 - alpha) * self._value
        return self._value.copy()


class BodyFrameEstimator:
    """Derive the camera->base rotation from the operator's own body.

    The shoulder line gives the operator's left/right axis in camera
    coordinates; the shoulder-to-hip direction (or the camera's up axis when the
    hips are out of frame) gives up. From those we rebuild the same
    "forward / left / up" frame the robot base uses.

    This is what replaces a one-off calibration when the camera is moved: the
    mapping follows wherever the camera now looks, as long as the operator is
    facing it.
    """

    def __init__(self, alpha: float = 0.05):
        self.alpha = float(alpha)
        self.rotation: np.ndarray | None = None
        self._samples = 0

    @staticmethod
    def _orthonormal(forward: np.ndarray, left: np.ndarray, up: np.ndarray) -> np.ndarray:
        # Re-orthogonalise with SVD so the matrix stays a rotation.
        basis = np.vstack([forward, left, up])
        u, _s, vt = np.linalg.svd(basis)
        rotation = u @ vt
        if np.linalg.det(rotation) < 0:
            u[:, -1] *= -1.0
            rotation = u @ vt
        return rotation

    def update(
        self,
        shoulder_left: np.ndarray,
        shoulder_right: np.ndarray,
        hip_left: np.ndarray | None = None,
        hip_right: np.ndarray | None = None,
    ) -> np.ndarray | None:
        right = np.asarray(shoulder_right, dtype=np.float64) - np.asarray(
            shoulder_left, dtype=np.float64
        )
        norm = float(np.linalg.norm(right))
        if norm < 1e-6:
            return self.rotation
        right /= norm
        left = -right

        if hip_left is not None and hip_right is not None:
            shoulders = 0.5 * (np.asarray(shoulder_left) + np.asarray(shoulder_right))
            hips = 0.5 * (np.asarray(hip_left) + np.asarray(hip_right))
            up = shoulders - hips
        else:
            # Half-body view: assume the camera is roughly upright.
            up = np.array([0.0, -1.0, 0.0])
        up = up - left * float(np.dot(up, left))
        up_norm = float(np.linalg.norm(up))
        if up_norm < 1e-6:
            return self.rotation
        up /= up_norm
        forward = np.cross(left, up)
        if float(np.dot(forward, np.array([0.0, 0.0, -1.0]))) < 0.0:
            # Operator would be facing away; flip the forward/left pair so the
            # frame still points at the camera.
            forward = -forward
            left = -left
        rotation = self._orthonormal(forward, left, up)

        if self.rotation is None:
            self.rotation = rotation
        else:
            blended = (1.0 - self.alpha) * self.rotation + self.alpha * rotation
            self.rotation = self._orthonormal(
                blended[0], blended[1], blended[2]
            )
        self._samples += 1
        return self.rotation
