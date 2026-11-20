#!/usr/bin/env python3
"""Offline self-test: configuration, geometry and clutch logic.

Runs without a camera, without mediapipe and without ROS:

    .venv/bin/python scripts/selftest.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "vision_bridge"))

from vision_bridge.clutch import (  # noqa: E402
    Clutch,
    Gesture,
    HandObservation,
    Mode,
)
from vision_bridge.config import load_config  # noqa: E402
from vision_bridge.mapping import Intrinsics, Mapper, deproject, tool_point  # noqa: E402


def check(name: str, condition: bool, detail: str = "") -> None:
    status = "ok  " if condition else "FAIL"
    print(f"[{status}] {name}{(' - ' + detail) if detail and not condition else ''}")
    if not condition:
        raise SystemExit(1)


def test_config() -> None:
    cfg = load_config(ROOT)
    check("config: camera width", cfg.vision.camera.color.width == 1280, str(cfg.vision.camera.color.width))
    check("config: publish rate", cfg.vision.publish.rate == 125.0)
    check("config: rotation 3x3", np.asarray(cfg.mapping.rotation).shape == (3, 3))
    check("config: pinch hysteresis", cfg.safety.pinch_close < cfg.safety.pinch_open)


def test_mapping() -> None:
    cfg = load_config(ROOT)
    mapper = Mapper(cfg.mapping, cfg.safety)
    forward = mapper.transform(np.array([0.0, 0.0, 1.0]))
    check("mapping: camera +z is base +x", np.allclose(forward, [1.0, 0.0, 0.0]), str(forward))
    right = mapper.transform(np.array([1.0, 0.0, 0.0]))
    check("mapping: camera +x is base -y", np.allclose(right, [0.0, -1.0, 0.0]), str(right))
    down = mapper.transform(np.array([0.0, 1.0, 0.0]))
    check("mapping: camera +y is base -z", np.allclose(down, [0.0, 0.0, -1.0]), str(down))

    k = Intrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480)
    point = deproject(320.0, 240.0, 2.0, k)
    check("mapping: centre deprojects to optical axis", np.allclose(point, [0.0, 0.0, 2.0]))
    off = deproject(620.0, 240.0, 2.0, k)
    check("mapping: pixel offset scales with depth", np.isclose(off[0], 1.0), str(off[0]))

    wrist = np.array([0.0, 0.0, 1.0])
    elbow = np.array([0.0, 0.0, 0.7])
    tool = tool_point(wrist, elbow, 0.08)
    check("mapping: tool offset follows the forearm", np.allclose(tool, [0.0, 0.0, 1.08]), str(tool))
    check("mapping: tool offset without elbow is a no-op", np.allclose(tool_point(wrist, None, 0.08), wrist))

    clipped = mapper.clamp(np.array([5.0, 0.0, 0.0]))
    check("mapping: workspace clamp", np.isclose(clipped[0], cfg.safety.workspace_max[0]))


def test_body_frame() -> None:
    from vision_bridge.mapping import BodyFrameEstimator

    estimator = BodyFrameEstimator(alpha=1.0)
    rotation = estimator.update(
        np.array([0.30, 0.0, 2.0]),   # left shoulder (image right)
        np.array([-0.30, 0.0, 2.0]),  # right shoulder (image left)
        np.array([0.20, 0.90, 2.0]),
        np.array([-0.20, 0.90, 2.0]),
    )
    check("body frame: rotation returned", rotation is not None)
    # Camera right (+x) is the operator's left -> robot +y.
    mapped = rotation @ np.array([1.0, 0.0, 0.0])
    check("body frame: camera right maps to robot left", np.allclose(mapped, [0.0, 1.0, 0.0], atol=1e-6), str(mapped))
    # Camera forward (+z) points at the operator -> robot -x.
    mapped = rotation @ np.array([0.0, 0.0, 1.0])
    check("body frame: camera forward maps to robot backward", np.allclose(mapped, [-1.0, 0.0, 0.0], atol=1e-6), str(mapped))
    check("body frame: determinant +1", abs(np.linalg.det(rotation) - 1.0) < 1e-6)

    half = BodyFrameEstimator(alpha=1.0)
    rotation = half.update(
        np.array([0.30, 0.0, 2.0]),
        np.array([-0.30, 0.0, 2.0]),
        None,
        None,
    )
    check("body frame: works without hips", rotation is not None and abs(np.linalg.det(rotation) - 1.0) < 1e-6)


def test_clutch() -> None:
    cfg = load_config(ROOT)
    clutch = Clutch(cfg.safety)
    fist = HandObservation(gesture=Gesture.FIST, pinch_ratio=0.2, present=True)
    pinch = HandObservation(gesture=Gesture.PINCH, pinch_ratio=0.2, present=True)
    open_hand = HandObservation(gesture=Gesture.OPEN, pinch_ratio=0.9, present=True)
    lost = HandObservation(gesture=Gesture.NONE, present=False)

    out = clutch.update(0.1, fist, fist)
    check("clutch: fists do not arm instantly", out.mode is Mode.STANDBY)
    for _ in range(12):
        out = clutch.update(0.1, fist, fist)
    check("clutch: held fists arm the session", out.mode is Mode.READY)
    check("clutch: armed but not engaged", not out.left_engaged and not out.right_engaged)

    out = clutch.update(0.1, pinch, open_hand)
    check("clutch: pinch engages left only", out.left_engaged and not out.right_engaged)

    out = clutch.update(0.1, pinch, pinch)
    check("clutch: both pinched engages both", out.left_engaged and out.right_engaged)

    out = clutch.update(0.1, lost, pinch)
    check("clutch: lost hand releases its arm", not out.left_engaged and out.right_engaged)

    clutch.toggle_emergency()
    out = clutch.update(0.1, pinch, pinch)
    check("clutch: ESC drops to standby", out.mode is Mode.STANDBY and not out.left_engaged)
    check("clutch: prompt asks to re-arm", "clench" in out.prompt.lower())

    for _ in range(12):
        out = clutch.update(0.1, fist, fist)
    check("clutch: re-arm after ESC", out.mode is Mode.READY)


def test_smoothing() -> None:
    from vision_bridge.mapping import OneEuroFilter

    from vision_bridge.config import SmoothingConfig

    smoothed = SmoothingConfig(enabled=True, min_cutoff=3.0, beta=4.0, derivative_cutoff=1.0)
    smoother = OneEuroFilter(smoothed)
    for _ in range(5):
        out = smoother(np.zeros(3), 0.08)
    check("smoothing: steady point stays put", np.allclose(out, 0.0))

    noisy = OneEuroFilter(smoothed)
    samples = []
    for i in range(40):
        noisy_point = np.array([0.01 if i % 2 else -0.01, 0.0, 0.0])
        samples.append(noisy(noisy_point, 0.08))
    amplitude = float(np.max(np.abs(np.array(samples)[15:, 0])))
    check("smoothing: alternating jitter attenuated", amplitude < 0.008, f"amplitude={amplitude:.4f}")

    tracked = OneEuroFilter(smoothed)
    for i in range(20):
        tracked(np.array([0.001 * i, 0.0, 0.0]), 0.08)
    final = tracked(np.array([0.05, 0.0, 0.0]), 0.08)
    check("smoothing: follows a real move", final[0] > 0.02, f"x={final[0]:.4f}")

    off = OneEuroFilter(SmoothingConfig(enabled=False))
    raw = np.array([0.01, 0.02, 0.03])
    check("smoothing: disabled is pass-through", np.allclose(off(raw, 0.08), raw))


def test_gesture_filter() -> None:
    from vision_bridge.clutch import GestureFilter

    filt = GestureFilter(pinch_close=0.35, pinch_open=0.55)

    check("gesture: open hand", filt.update(True, 0.9, 2.0) is Gesture.OPEN)
    check("gesture: enters pinch", filt.update(True, 0.30, 2.0) is Gesture.PINCH)
    check("gesture: pinch hysteresis holds", filt.update(True, 0.45, 2.0) is Gesture.PINCH)
    check("gesture: releases above open threshold", filt.update(True, 0.60, 2.0) is Gesture.OPEN)
    check("gesture: fist wins over pinch", filt.update(True, 0.20, 1.0) is Gesture.FIST)
    check("gesture: fist hysteresis holds", filt.update(True, 0.20, 1.25) is Gesture.FIST)
    check("gesture: unfist", filt.update(True, 0.90, 1.5) is Gesture.OPEN)
    check("gesture: missing hand", filt.update(False, 0.0, 0.0) is Gesture.NONE)

    absolute = GestureFilter(pinch_close=0.525, pinch_open=0.619, pinch_close_m=0.020, pinch_open_m=0.035)
    check("pinch: 5 cm apart is open", absolute.update(True, 0.9, 1.7, pinch_m=0.050) is Gesture.OPEN)
    check("pinch: small distance but open ratio is vetoed",
          absolute.update(True, 0.9, 1.7, pinch_m=0.015) is Gesture.OPEN)
    check("pinch: 1.5 cm with a closed ratio engages",
          absolute.update(True, 0.20, 1.7, pinch_m=0.015) is Gesture.PINCH)
    check("pinch: hysteresis holds at 3 cm",
          absolute.update(True, 0.30, 1.7, pinch_m=0.030) is Gesture.PINCH)
    check("pinch: releases at 4 cm", absolute.update(True, 0.30, 1.7, pinch_m=0.040) is Gesture.OPEN)

    debounced = GestureFilter(pinch_close=0.525, pinch_open=0.619, debounce_s=0.15)
    check("debounce: first reading is still unknown",
          debounced.update(True, 0.9, 1.7, now=0.0) is Gesture.NONE)
    check("debounce: not accepted before the window elapses",
          debounced.update(True, 0.9, 1.7, now=0.05) is Gesture.NONE)
    check("debounce: accepted after the window",
          debounced.update(True, 0.9, 1.7, now=0.20) is Gesture.OPEN)
    check("debounce: one pinch reading does not flip",
          debounced.update(True, 0.20, 1.7, now=0.25) is Gesture.OPEN)
    check("debounce: held pinch flips",
          debounced.update(True, 0.20, 1.7, now=0.45) is Gesture.PINCH)


def main() -> int:
    test_config()
    test_mapping()
    test_body_frame()
    test_clutch()
    test_smoothing()
    test_gesture_filter()
    print("\nall checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
