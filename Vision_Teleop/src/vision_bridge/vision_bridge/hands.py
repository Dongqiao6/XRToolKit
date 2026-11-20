"""Wrist-anchored crop + MediaPipe hand landmarks, used for the pinch clutch.

Running the hand model on a 256x256 crop around the pose wrist keeps the hand
large in the model's input even when the full frame is 1280x720 and the
operator stands two metres away.
"""

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
class HandSample:
    present: bool = False
    landmarks_px: np.ndarray | None = None  # (21, 2) full-frame pixels
    palm_px: np.ndarray | None = None
    pinch_ratio: float = float("nan")
    pinch_m: float = float("nan")
    curl: float = float("nan")
    span_px: float = 0.0
    crop_box: tuple[int, int, int, int] | None = None


class HandTracker:
    def __init__(
        self,
        model_path: Path,
        crop_size: int,
        min_detection: float,
        min_presence: float,
    ):
        if mp is None:
            raise RuntimeError("mediapipe is not installed")
        if not Path(model_path).is_file():
            raise FileNotFoundError(
                f"hand model not found: {model_path}. Run scripts/download_models.sh"
            )
        options = vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.IMAGE,
            num_hands=1,
            min_hand_detection_confidence=float(min_detection),
            min_hand_presence_confidence=float(min_presence),
            min_tracking_confidence=0.4,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)
        self.crop_size = int(crop_size)
        # The crop is resized to the model's fixed input, so a tight crop is the
        # only way to give the model enough pixels at 2 m. Start at the
        # configured size and adapt to the hand actually seen.
        self._adaptive_size = float(crop_size)
        self._ratio_ema: float | None = None
        self.min_span_px = 12.0
        self.crop_gain = 3.5
        self.min_crop = 96
        self.max_crop = 320

    def _crop(
        self, bgr: np.ndarray, centre: np.ndarray, size: int | None = None
    ) -> tuple[np.ndarray, tuple[int, int, int, int]]:
        height, width = bgr.shape[:2]
        size = min(int(size or self._adaptive_size), min(height, width))
        cx, cy = float(centre[0]), float(centre[1])
        x0 = int(round(cx - size / 2.0))
        y0 = int(round(cy - size / 2.0))
        x0 = max(0, min(width - size, x0))
        y0 = max(0, min(height - size, y0))
        return bgr[y0 : y0 + size, x0 : x0 + size], (x0, y0, size, size)

    def process(
        self, bgr: np.ndarray, wrist_px: np.ndarray, centre_px: np.ndarray | None = None
    ) -> HandSample:
        centre = wrist_px if centre_px is None else centre_px
        crop, (x0, y0, size, _size) = self._crop(bgr, centre)
        rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
        result = self._landmarker.detect(image)
        if not result.hand_landmarks:
            return HandSample(present=False, crop_box=(x0, y0, size, size))

        points = np.array(
            [[lm.x * size + x0, lm.y * size + y0] for lm in result.hand_landmarks[0]],
            dtype=np.float64,
        )
        hand_size = float(np.linalg.norm(points[0] - points[9]))
        if hand_size < self.min_span_px:
            # Too few pixels for a trustworthy pinch: report nothing rather than
            # a random gesture.
            return HandSample(present=False, landmarks_px=points, crop_box=(x0, y0, size, size))

        # Prefer the metric world landmarks: their ratios do not wobble with
        # pixel quantisation when the hand is small.
        ratio_source = None
        pinch_m = float("nan")
        if getattr(result, "hand_world_landmarks", None):
            ratio_source = np.array(
                [[lm.x, lm.y, lm.z] for lm in result.hand_world_landmarks[0]], dtype=np.float64
            )
        if ratio_source is not None:
            world_span = float(np.linalg.norm(ratio_source[0] - ratio_source[9]))
            if world_span > 1e-6:
                pinch_m = float(np.linalg.norm(ratio_source[4] - ratio_source[8]))
                pinch = pinch_m / world_span
                curl = float(
                    np.mean(
                        [np.linalg.norm(ratio_source[t] - ratio_source[0]) for t in (8, 12, 16, 20)]
                    )
                ) / world_span
            else:
                ratio_source = None
        if ratio_source is None:
            pinch = float(np.linalg.norm(points[4] - points[8])) / hand_size
            curl = float(
                np.mean([np.linalg.norm(points[tip] - points[0]) for tip in (8, 12, 16, 20)])
            ) / hand_size

        self._ratio_ema = pinch if self._ratio_ema is None else 0.6 * pinch + 0.4 * self._ratio_ema
        pinch = self._ratio_ema
        target = np.clip(self.crop_gain * hand_size, self.min_crop, self.max_crop)
        self._adaptive_size = 0.7 * self._adaptive_size + 0.3 * target
        palm = np.mean(points[[0, 5, 9, 13, 17]], axis=0)
        return HandSample(
            present=True,
            landmarks_px=points,
            palm_px=palm,
            pinch_ratio=pinch,
            pinch_m=pinch_m,
            curl=curl,
            span_px=hand_size,
            crop_box=(x0, y0, size, size),
        )

    def close(self) -> None:
        self._landmarker.close()
