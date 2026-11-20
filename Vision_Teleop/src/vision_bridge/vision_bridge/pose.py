"""MediaPipe Tasks pose tracking (33 landmarks, VIDEO mode)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

try:  # pragma: no cover - heavy import
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision
except ImportError:  # pragma: no cover
    mp = None


@dataclass
class PoseResult:
    present: bool
    landmarks: np.ndarray | None = None  # (33, 2) normalised image coordinates
    world: np.ndarray | None = None  # (33, 3) metres, hip centred
    visibility: np.ndarray | None = None  # (33,)

    def point(self, index: int, width: int, height: int) -> np.ndarray | None:
        """Landmark as pixel coordinates (x, y)."""
        if self.landmarks is None:
            return None
        x, y = self.landmarks[index]
        return np.array([x * width, y * height], dtype=np.float64)

    def visible(self, index: int, threshold: float) -> bool:
        if self.visibility is None:
            return False
        return bool(self.visibility[index] >= threshold)


class PoseTracker:
    """Wraps ``PoseLandmarker`` in VIDEO mode, enforcing monotonic timestamps."""

    def __init__(self, model_path: Path, min_detection: float, min_tracking: float):
        if mp is None:
            raise RuntimeError("mediapipe is not installed")
        if not Path(model_path).is_file():
            raise FileNotFoundError(
                f"pose model not found: {model_path}. Run scripts/download_models.sh"
            )
        options = vision.PoseLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=float(min_detection),
            min_pose_presence_confidence=float(min_tracking),
            min_tracking_confidence=float(min_tracking),
        )
        self._landmarker = vision.PoseLandmarker.create_from_options(options)
        self._last_ts = -1

    def process(self, bgr: np.ndarray, timestamp_ms: int) -> PoseResult:
        if timestamp_ms <= self._last_ts:
            timestamp_ms = self._last_ts + 1
        self._last_ts = timestamp_ms
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
        result = self._landmarker.detect_for_video(image, timestamp_ms)
        if not result.pose_landmarks:
            return PoseResult(present=False)
        landmarks = np.array(
            [[lm.x, lm.y] for lm in result.pose_landmarks[0]], dtype=np.float64
        )
        visibility = np.array(
            [getattr(lm, "visibility", 0.0) for lm in result.pose_landmarks[0]],
            dtype=np.float64,
        )
        world = None
        if getattr(result, "pose_world_landmarks", None):
            world = np.array(
                [[lm.x, lm.y, lm.z] for lm in result.pose_world_landmarks[0]],
                dtype=np.float64,
            )
        return PoseResult(present=True, landmarks=landmarks, world=world, visibility=visibility)

    def close(self) -> None:
        self._landmarker.close()
