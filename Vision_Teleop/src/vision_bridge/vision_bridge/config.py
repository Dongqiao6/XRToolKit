"""Configuration loading for the vision teleoperation bridge.

Three YAML files live under ``Vision_Teleop/config``:

* ``vision.yaml``  - camera, model and window settings
* ``mapping.yaml`` - camera-frame to robot-base-frame transform
* ``safety.yaml``  - gesture thresholds, limits and workspace

Every field has a default so the node still starts when a file is missing,
which keeps first-run debugging cheap.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


def project_root() -> Path:
    """Return ``Vision_Teleop/`` regardless of how the module is imported."""
    override = os.environ.get("VISION_TELEOP_ROOT")
    if override:
        return Path(override).expanduser().resolve()
    return Path(__file__).resolve().parents[3]


@dataclass
class StreamConfig:
    width: int = 1280
    height: int = 720
    fps: int = 30


@dataclass
class CameraConfig:
    serial: str = ""
    color: StreamConfig = field(default_factory=lambda: StreamConfig(1280, 720, 30))
    depth: StreamConfig = field(default_factory=lambda: StreamConfig(848, 480, 30))
    # librealsense console verbosity: none / error / warning / info / debug
    log_level: str = "none"
    # Median window (pixels) used when sampling depth around a landmark.
    depth_window: int = 4


@dataclass
class PoseConfig:
    model: str = "models/pose_landmarker_full.task"
    min_detection: float = 0.5
    min_tracking: float = 0.5
    min_visibility: float = 0.5


@dataclass
class HandConfig:
    model: str = "models/hand_landmarker.task"
    crop_size: int = 256
    min_detection: float = 0.4
    min_presence: float = 0.4
    # Minimum seconds between hand inferences. Gestures only drive the clutch,
    # so running them slower frees CPU for pose and raises the position rate.
    interval_s: float = 0.12


@dataclass
class PublishConfig:
    rate: float = 125.0
    frame_id: str = "world"


@dataclass
class WindowConfig:
    enabled: bool = True
    scale: float = 1.0


@dataclass
class SmoothingConfig:
    enabled: bool = False
    # One Euro filter: lower min_cutoff = smoother but laggier.
    min_cutoff: float = 1.2
    beta: float = 1.5
    derivative_cutoff: float = 1.0


@dataclass
class VisionConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    pose: PoseConfig = field(default_factory=PoseConfig)
    hand: HandConfig = field(default_factory=HandConfig)
    publish: PublishConfig = field(default_factory=PublishConfig)
    window: WindowConfig = field(default_factory=WindowConfig)
    smoothing: SmoothingConfig = field(default_factory=SmoothingConfig)
    # Swallow the native LiteRT/librealsense banner printed during model load.
    quiet_native: bool = True


@dataclass
class MappingConfig:
    # Row-major 3x3 rotation from the camera frame to the robot base frame.
    rotation: tuple[tuple[float, float, float], ...] = (
        (0.0, 0.0, 1.0),
        (-1.0, 0.0, 0.0),
        (0.0, -1.0, 0.0),
    )
    scale: float = 1.0
    # Distance from the human wrist to the point that should track the robot TCP.
    tool_offset_m: float = 0.08
    # Pose published for a zero displacement. The downstream rc_ctrl node takes
    # its own origin when the grip is pressed, so only deltas matter.
    anchor_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Derive the camera->base rotation from the operator's shoulders/hips every
    # frame, so the system keeps working when the camera is moved.
    # Per-axis sign flips applied after the rotation, e.g. [1, -1, 1] if the
    # robot's y axis runs the other way. Cheaper than editing the matrix.
    axes_sign: tuple[float, float, float] = (1.0, 1.0, 1.0)
    use_body_frame: bool = True
    # EMA weight for the estimated rotation (smaller = slower, steadier).
    body_frame_alpha: float = 0.05


@dataclass
class SafetyConfig:
    # Pinch distance normalised by hand size: close below the first value,
    # release above the second one (Schmitt trigger).
    pinch_close: float = 0.45
    pinch_open: float = 0.60
    # Absolute thumb-index distance (metres) used when 3D hand landmarks are
    # available: "fingers nearly touching" is a physical distance, not a ratio.
    pinch_close_m: float = 0.030
    pinch_open_m: float = 0.050
    # Ignore target changes smaller than this: static landmark jitter must not
    # move the arm.
    target_deadband_m: float = 0.008
    # Fingertip-curl threshold for "clenched fist", with hysteresis.
    fist_curl_enter: float = 1.039
    fist_curl_exit: float = 1.376
    # Seconds a new gesture must persist before it is accepted.
    gesture_debounce_s: float = 0.15
    # A hand curled tighter than this is never treated as a pinch, even if the
    # thumb/index tips are close (that is what a fist looks like).
    pinch_min_curl: float = 1.2
    # Seconds both hands must stay clenched to arm the system.
    fist_hold_s: float = 1.0
    # Reject a depth sample that jumps more than this between frames.
    depth_gate_m: float = 0.06
    # Seconds without a valid hand before the arm is treated as lost.
    lost_timeout_s: float = 0.35
    # Time constant of the exponential follower between the 125 Hz publisher
    # and the ~14 Hz target. Larger = smoother, at the cost of follow lag.
    follow_tau_s: float = 0.05
    # Hard cap on a single publish step, in metres (safety / speed limit).
    max_step_m: float = 0.004
    # Hard cap on how far the *target* may jump between camera frames. A depth
    # glitch must never send the arm chasing a point half a metre away.
    max_target_step_m: float = 0.03
    # Workspace box in the robot base frame, metres.
    workspace_min: tuple[float, float, float] = (-0.6, -0.4, -0.3)
    workspace_max: tuple[float, float, float] = (0.6, 0.4, 0.6)


@dataclass
class AppConfig:
    vision: VisionConfig = field(default_factory=VisionConfig)
    mapping: MappingConfig = field(default_factory=MappingConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    root: Path = field(default_factory=project_root)

    def path(self, relative: str) -> Path:
        candidate = Path(relative).expanduser()
        return candidate if candidate.is_absolute() else self.root / candidate


def _merge(defaults: Any, data: Any) -> Any:
    """Recursively overlay ``data`` on ``defaults`` and coerce tuples."""
    if isinstance(defaults, dict) and isinstance(data, dict):
        merged = dict(defaults)
        for key, value in data.items():
            merged[key] = _merge(defaults.get(key), value)
        return merged
    if isinstance(defaults, (list, tuple)) and isinstance(data, (list, tuple)):
        return tuple(data)
    return data


def _as_dict(obj: Any) -> dict:
    out: dict = {}
    for name, value in obj.__dict__.items():
        if hasattr(value, "__dict__"):
            out[name] = _as_dict(value)
        else:
            out[name] = value
    return out


def _apply(obj: Any, data: dict) -> Any:
    for key, value in data.items():
        if not hasattr(obj, key):
            continue
        current = getattr(obj, key)
        if hasattr(current, "__dict__") and isinstance(value, dict):
            _apply(current, value)
        elif isinstance(current, tuple) and isinstance(value, list):
            setattr(obj, key, tuple(tuple(item) if isinstance(item, list) else item for item in value))
        elif isinstance(current, tuple) and isinstance(value, tuple):
            setattr(obj, key, value)
        else:
            setattr(obj, key, value)
    return obj


def _read(path: Path) -> dict:
    try:
        raw = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return {}
    loaded = yaml.safe_load(raw) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"{path}: expected a mapping at the top level")
    return loaded


def load_config(root: Path | None = None) -> AppConfig:
    """Load the three YAML files, applying them over built-in defaults."""
    base = (root or project_root()).resolve()
    config = AppConfig(root=base)

    vision_data = _read(base / "config" / "vision.yaml")
    mapping_data = _read(base / "config" / "mapping.yaml")
    safety_data = _read(base / "config" / "safety.yaml")

    _apply(config.vision, _merge(_as_dict(config.vision), vision_data))
    _apply(config.mapping, _merge(_as_dict(config.mapping), mapping_data))
    _apply(config.safety, _merge(_as_dict(config.safety), safety_data))
    return config
