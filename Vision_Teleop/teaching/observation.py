"""Use upstream detection and OpenCV geometry; no custom vision algorithm."""
from __future__ import annotations

from collections import deque
import os
import sys
from pathlib import Path
import cv2
import numpy as np


def validate_board(board):
    for name in ("cols", "rows"):
        if not isinstance(board.get(name), int) or isinstance(board[name], bool) or board[name] < 3:
            raise ValueError(f"board.{name}: enter the number of INTERNAL corners (at least 3)")
    size = board.get("square_size_m")
    if not isinstance(size, (int, float)) or isinstance(size, bool) or not np.isfinite(size) or size <= 0:
        raise ValueError("board.square_size_m: enter measured square edge in metres")


def board_points(board):
    validate_board(board)
    points = np.zeros((board["rows"] * board["cols"], 3), np.float32)
    points[:, :2] = np.mgrid[0:board["cols"], 0:board["rows"]].T.reshape(-1, 2) * board["square_size_m"]
    return points


def detect_board(image, board):
    validate_board(board)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(gray, (board["cols"], board["rows"]),
                                               flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    return corners.astype(np.float32) if found else None


class BoardStability:
    def __init__(self, count=3, max_motion=1.5):
        self.history = deque(maxlen=count)
        self.count, self.max_motion = count, max_motion

    def update(self, corners, frame_numbers):
        if corners[0] is None or corners[1] is None:
            self.history.clear()
            return False, None
        numbers = np.asarray(frame_numbers)
        joined = np.concatenate(corners).reshape(2, -1, 2)
        motion = None
        if self.history:
            previous, prev_numbers = self.history[-1]
            gaps = numbers - prev_numbers
            if np.any(gaps <= 0) or np.any(gaps > 15):
                self.history.clear()
            else:
                motion = float(np.max(np.linalg.norm(joined - previous, axis=2).mean(axis=1) / gaps))
                if motion > self.max_motion:
                    self.history.clear()
        self.history.append((joined.copy(), numbers.copy()))
        return len(self.history) == self.count and motion is not None, motion


def calibrate(corners, size, board):
    if len(corners) < board["min_pairs"]:
        raise ValueError(f"Need at least {board['min_pairs']} valid pairs; have {len(corners)}")
    objects = [board_points(board) for _ in corners]
    left, right = [x[0] for x in corners], [x[1] for x in corners]
    lrms, k1, d1, *_ = cv2.calibrateCamera(objects, left, size, None, None)
    rrms, k2, d2, *_ = cv2.calibrateCamera(objects, right, size, None, None)
    rms, k1, d1, k2, d2, r, t, e, f = cv2.stereoCalibrate(objects, left, right, k1, d1, k2, d2, size,
                                                        flags=cv2.CALIB_FIX_INTRINSIC)
    baseline = float(np.linalg.norm(t))
    if not np.isfinite(rms) or not 0.001 < baseline < 10:
        raise ValueError("Invalid stereo calibration; check board geometry and pair ordering")
    result = dict(left_rms=float(lrms), right_rms=float(rrms), stereo_rms=float(rms), baseline_m=baseline,
                  image_size=list(size), board=board, valid_pairs=len(corners),
                  quality_status="NEEDS_GEOMETRY_REVIEW_NOT_AUTOMATIC_PASS")
    for name, value in dict(K1=k1, D1=d1, K2=k2, D2=d2, R=r, T=t, E=e, F=f).items():
        result[name] = value.tolist()
    return result


def triangulate(left, right, calibration):
    c = {k: np.asarray(calibration[k], dtype=float) for k in ("K1", "D1", "K2", "D2", "R", "T")}
    l = cv2.undistortPoints(np.asarray(left, dtype=float).reshape(-1, 1, 2), c["K1"], c["D1"])
    r = cv2.undistortPoints(np.asarray(right, dtype=float).reshape(-1, 1, 2), c["K2"], c["D2"])
    p1, p2 = np.eye(3, 4), np.column_stack((c["R"], c["T"].reshape(3)))
    homogeneous = cv2.triangulatePoints(p1, p2, l.reshape(-1, 2).T, r.reshape(-1, 2).T)
    with np.errstate(divide="ignore", invalid="ignore"):
        xyz = (homogeneous[:3] / homogeneous[3]).T
    valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0) & ((xyz @ c["R"].T + c["T"].reshape(3))[:, 2] > 0)
    xyz[~valid] = np.nan
    return xyz, valid


class Detector:
    def __init__(self, device="cuda", mode="lightweight", single_subject=False, target="body", min_palm_score=0.5):
        os.environ["TORCH_HOME"] = str(Path(__file__).resolve().parents[1] / "models/rtmlib")
        import onnxruntime as ort
        from rtmlib import Wholebody, Hand
        if target not in ("body", "hand"):
            raise ValueError("Unknown detector target")
        if device == "cuda":
            self.dll_directories = []
            if os.name == "nt":
                for directory in (Path(sys.prefix) / "Lib/site-packages/nvidia").glob("*/bin"):
                    self.dll_directories.append(os.add_dll_directory(str(directory)))
                import ctypes
                nvrtc = Path(sys.prefix) / "Lib/site-packages/nvidia/cuda_nvrtc/bin/nvrtc64_120_0.dll"
                if nvrtc.exists():
                    self.nvrtc = ctypes.WinDLL(str(nvrtc))
            ort.preload_dlls(directory="")
        self.model = (Hand if target == "hand" else Wholebody)(mode=mode, backend="onnxruntime", device=device)
        self.target = target
        self.keypoint_count = 21 if target == "hand" else 133
        self.min_palm_score = min_palm_score
        self.providers = [self.model.det_model.session.get_providers(), self.model.pose_model.session.get_providers()]
        if device == "cuda" and any("CUDAExecutionProvider" not in p for p in self.providers):
            raise RuntimeError("Requested GPU detector fell back to CPU; refusing silent slowdown")
        self.device = device
        self.single_subject = single_subject
        self.person_count = 0
        if device == "cuda":
            for model in (self.model.det_model, self.model.pose_model):
                model.session.disable_fallback()

    def __call__(self, image):
        if self.single_subject or self.target == "hand":
            boxes = self.model.det_model(image)
            self.person_count = len(boxes)
            if not len(boxes):
                return np.empty((0, self.keypoint_count, 2)), np.empty((0, self.keypoint_count))
            areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
            chosen = boxes if self.target == "hand" else boxes[[int(np.argmax(areas))]]
            # Preview the largest subject, but publish ambiguity separately.
            # Future control must reject person_count != 1, not silently switch.
            result = self.model.pose_model(image, bboxes=chosen)
        else:
            result = self.model(image)
            self.person_count = len(result[0])
        if self.device == "cuda":
            for model in (self.model.det_model, self.model.pose_model):
                if "CUDAExecutionProvider" not in model.session.get_providers():
                    raise RuntimeError("GPU provider was lost during inference")
        return result


def palm_observation_clear(result, min_score=0.5):
    points, scores = result
    if points.shape != (1, 21, 2) or scores.shape != (1, 21):
        return False
    indices = [0, 5, 9, 17]
    return bool(np.isfinite(points[0, indices]).all() and
                np.isfinite(scores[0, indices]).all() and (scores[0, indices] >= min_score).all())
