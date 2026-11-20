#!/usr/bin/env python3
"""Interactive camera -> robot-base calibration.

The operator performs three moves in front of the D435: toward the camera, to
their own right, and up. From those three camera-frame vectors we build the
rotation that maps camera deltas into robot-base deltas, assuming the operator
faces the camera and the robot base axes are "forward / left / up".

Usage:
    source env.sh
    python scripts/calibrate.py [--hand right|left] [--no-window]
"""

from __future__ import annotations

import argparse
import datetime as dt
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "vision_bridge"))

from vision_bridge.camera import RealSenseSource  # noqa: E402
from vision_bridge.config import load_config  # noqa: E402
from vision_bridge.mapping import depth_at, deproject  # noqa: E402
from vision_bridge.overlay import (  # noqa: E402
    LEFT_ELBOW,
    LEFT_WRIST,
    RIGHT_ELBOW,
    RIGHT_WRIST,
    draw_overlay,
    Hud,
)
from vision_bridge.pose import PoseTracker  # noqa: E402

MOVES = (
    ("forward", "move your hand TOWARD the camera", "hand away from the camera"),
    ("right", "move your hand to YOUR RIGHT", "hand to your left"),
    ("up", "move your hand UP", "hand down"),
)


class Sample:
    def __init__(self) -> None:
        self.vectors: dict[str, np.ndarray] = {}
        self.stage = 0  # index into MOVES * 2
        self.start_point: np.ndarray | None = None


def wrist_point(pose, width: int, height: int, hand: str, depth: np.ndarray, k) -> np.ndarray | None:
    wrist = LEFT_WRIST if hand == "left" else RIGHT_WRIST
    if not pose.present or not pose.visible(wrist, 0.5):
        return None
    pixel = pose.point(wrist, width, height)
    if pixel is None:
        return None
    d = depth_at(depth, int(pixel[0]), int(pixel[1]))
    if d <= 0.0:
        return None
    return deproject(pixel[0], pixel[1], d, k)


def build_rotation(vectors: dict[str, np.ndarray]) -> tuple[np.ndarray, dict]:
    forward = vectors["forward"] / np.linalg.norm(vectors["forward"])
    right = vectors["right"] / np.linalg.norm(vectors["right"])
    up_raw = vectors["up"] / np.linalg.norm(vectors["up"])

    right = right - forward * float(np.dot(right, forward))
    right /= np.linalg.norm(right)
    up = up_raw - forward * float(np.dot(up_raw, forward)) - right * float(np.dot(up_raw, right))
    up /= np.linalg.norm(up)

    left = -right
    forward = np.cross(left, up)
    forward /= np.linalg.norm(forward)

    rotation = np.column_stack([forward, left, up])
    diagnostics = {
        "lengths_m": {name: float(np.linalg.norm(vec)) for name, vec in vectors.items()},
        "angle_forward_right_deg": float(
            np.degrees(
                np.arccos(
                    np.clip(
                        np.dot(vectors["forward"], vectors["right"])
                        / (np.linalg.norm(vectors["forward"]) * np.linalg.norm(vectors["right"])),
                        -1.0,
                        1.0,
                    )
                )
            )
        ),
        "angle_forward_up_deg": float(
            np.degrees(
                np.arccos(
                    np.clip(
                        np.dot(vectors["forward"], vectors["up"])
                        / (np.linalg.norm(vectors["forward"]) * np.linalg.norm(vectors["up"])),
                        -1.0,
                        1.0,
                    )
                )
            )
        ),
        "determinant": float(np.linalg.det(rotation)),
    }
    return rotation, diagnostics


def save_mapping(root: Path, rotation: np.ndarray, diagnostics: dict, hand: str) -> Path:
    path = root / "config" / "mapping.yaml"
    data = {}
    if path.is_file():
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    data["rotation"] = [[round(float(v), 6) for v in row] for row in rotation]
    data.setdefault("scale", 1.0)
    data.setdefault("tool_offset_m", 0.08)
    data.setdefault("anchor_position", [0.0, 0.0, 0.0])
    data["calibrated_at"] = dt.datetime.now().isoformat(timespec="seconds")
    data["calibrated_hand"] = hand
    diagnostics_out = {
        "lengths_m": {k: round(v, 4) for k, v in diagnostics["lengths_m"].items()},
        "angle_forward_right_deg": round(diagnostics["angle_forward_right_deg"], 1),
        "angle_forward_up_deg": round(diagnostics["angle_forward_up_deg"], 1),
        "determinant": round(diagnostics["determinant"], 4),
    }
    data["calibration_diagnostics"] = diagnostics_out
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hand", choices=("right", "left"), default="right")
    parser.add_argument("--no-window", action="store_true", help="terminal prompts only")
    args = parser.parse_args()

    config = load_config()
    camera = RealSenseSource(config.vision.camera)
    pose_tracker = PoseTracker(
        config.path(config.vision.pose.model),
        config.vision.pose.min_detection,
        config.vision.pose.min_tracking,
    )
    sample = Sample()
    camera.start()
    print("Calibration: three moves, capture start and end of each.")
    print("Keys: SPACE capture, r restart, q abort")

    try:
        while True:
            frame = camera.read()
            if frame is None:
                continue
            height, width = frame.bgr.shape[:2]
            pose = pose_tracker.process(frame.bgr, int(frame.stamp * 1000.0))
            point = wrist_point(pose, width, height, args.hand, frame.depth_m, frame.intrinsics)

            stage = sample.stage
            if stage >= len(MOVES) * 2:
                break
            move_index = stage // 2
            capturing_end = stage % 2 == 1
            name = MOVES[move_index][0]
            if capturing_end:
                prompt = f"{MOVES[move_index][1]}, then SPACE"
            else:
                prompt = f"Hold your {args.hand} hand still at the start, SPACE"

            hud = Hud(
                prompt=prompt,
                mode="calibration",
                left_gesture=name,
                right_gesture=f"{stage + 1}/{len(MOVES) * 2}",
                fps=0.0,
            )
            canvas = draw_overlay(
                frame.bgr, pose.landmarks if pose.present else None, None, None, None, None, hud
            )
            cv2.putText(
                canvas,
                "point visible" if point is not None else "NO DEPTH at wrist",
                (14, height - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0) if point is not None else (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("Calibration", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("aborted")
                return 1
            if key == ord("r"):
                sample = Sample()
                continue
            if key == 32:  # SPACE
                if point is None:
                    print("no depth at wrist, move your hand into view")
                    continue
                if not capturing_end:
                    sample.start_point = point
                    sample.stage += 1
                else:
                    assert sample.start_point is not None
                    sample.vectors[name] = point - sample.start_point
                    print(f"{name}: {np.round(sample.vectors[name], 4)} m")
                    sample.stage += 1
    finally:
        camera.stop()
        pose_tracker.close()
        cv2.destroyAllWindows()

    rotation, diagnostics = build_rotation(sample.vectors)
    print("rotation (camera -> robot base):")
    print(np.round(rotation, 4))
    print("diagnostics:", diagnostics)
    if diagnostics["determinant"] < 0:
        print("ERROR: rotation is left handed, calibration moves were inconsistent")
        return 1
    if min(diagnostics["lengths_m"].values()) < 0.05:
        print("WARNING: one move was shorter than 5 cm, accuracy will suffer")
    path = save_mapping(config.root, rotation, diagnostics, args.hand)
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
