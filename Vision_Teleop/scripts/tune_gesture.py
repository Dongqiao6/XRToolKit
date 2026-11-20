#!/usr/bin/env python3
"""Measure your own pinch/open/fist numbers and derive thresholds.

Run it, then follow the prompts: hold the gesture, press the key, repeat a few
times per gesture. On quit it prints suggested values for config/safety.yaml,
and with ``--write`` it updates the file for you.

    source env.sh
    python scripts/tune_gesture.py            # measure only
    python scripts/tune_gesture.py --write    # measure and save
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "vision_bridge"))

from vision_bridge.camera import RealSenseSource  # noqa: E402
from vision_bridge.config import load_config  # noqa: E402
from vision_bridge.hands import HandTracker  # noqa: E402
from vision_bridge.overlay import LEFT_INDEX, LEFT_WRIST, RIGHT_INDEX, RIGHT_WRIST  # noqa: E402
from vision_bridge.pose import PoseTracker  # noqa: E402

INSTRUCTIONS = [
    "1 = PINCH both hands (thumb+index touching)",
    "2 = OPEN both hands (fingers spread)",
    "3 = FIST both hands (clenched)",
    "q = finish and report",
]


def palm_centre(pose, wrist_index, index_index, width, height):
    wrist = pose.point(wrist_index, width, height)
    if wrist is None:
        return None, None
    tip = pose.point(index_index, width, height) if pose.visible(index_index, 0.5) else None
    centre = wrist if tip is None else wrist + 0.5 * (tip - wrist)
    return wrist, centre


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true", help="update config/safety.yaml")
    args = parser.parse_args()

    config = load_config(ROOT)
    camera = RealSenseSource(config.vision.camera)
    pose_tracker = PoseTracker(
        config.path(config.vision.pose.model),
        config.vision.pose.min_detection,
        config.vision.pose.min_tracking,
    )
    left = HandTracker(
        config.path(config.vision.hand.model),
        config.vision.hand.crop_size,
        config.vision.hand.min_detection,
        config.vision.hand.min_presence,
    )
    right = HandTracker(
        config.path(config.vision.hand.model),
        config.vision.hand.crop_size,
        config.vision.hand.min_detection,
        config.vision.hand.min_presence,
    )

    samples: dict[str, dict[str, list[float]]] = {
        "pinch": {"ratio": [], "curl": []},
        "open": {"ratio": [], "curl": []},
        "fist": {"ratio": [], "curl": []},
    }
    current = (float("nan"), float("nan"), float("nan"), float("nan"))
    camera.start()
    print("\n".join(INSTRUCTIONS))
    try:
        while True:
            frame = camera.read()
            if frame is None:
                continue
            height, width = frame.bgr.shape[:2]
            pose = pose_tracker.process(frame.bgr, int(frame.stamp * 1000.0))
            values = []
            boxes = []
            for tracker, wrist_index, index_index in (
                (left, LEFT_WRIST, LEFT_INDEX),
                (right, RIGHT_WRIST, RIGHT_INDEX),
            ):
                wrist = centre = None
                if pose.present:
                    wrist, centre = palm_centre(pose, wrist_index, index_index, width, height)
                if wrist is None:
                    values.append((float("nan"), float("nan")))
                    boxes.append(None)
                    continue
                sample = tracker.process(frame.bgr, wrist, centre)
                values.append((sample.pinch_ratio, sample.curl))
                boxes.append(sample.crop_box if sample.present else None)

            current = (values[0][0], values[0][1], values[1][0], values[1][1])
            canvas = frame.bgr.copy()
            for box, colour in zip(boxes, ((255, 128, 0), (0, 128, 255))):
                if box:
                    x, y, s, _ = box
                    cv2.rectangle(canvas, (x, y), (x + s, y + s), colour, 1)
            lines = [
                f"L pinch {current[0]:.3f}  curl {current[1]:.3f}",
                f"R pinch {current[2]:.3f}  curl {current[3]:.3f}",
                " | ".join(INSTRUCTIONS),
            ]
            for i, line in enumerate(lines):
                cv2.putText(canvas, line, (14, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("Gesture tuner", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            labels = {ord("1"): "pinch", ord("2"): "open", ord("3"): "fist"}
            if key in labels:
                name = labels[key]
                for ratio, curl in values:
                    if ratio == ratio and curl == curl:
                        samples[name]["ratio"].append(float(ratio))
                        samples[name]["curl"].append(float(curl))
                print(f"captured {name}: {len(samples[name]['ratio'])} samples")
    finally:
        camera.stop()
        pose_tracker.close()
        left.close()
        right.close()
        cv2.destroyAllWindows()

    return report(samples, config, args.write)


def _percentile(values: list[float], q: float) -> float:
    return float(np.percentile(values, q)) if values else float("nan")


def report(samples: dict, config, write: bool) -> int:
    pinch = samples["pinch"]["ratio"]
    open_ = samples["open"]["ratio"]
    fist_curl = samples["fist"]["curl"]
    other_curl = samples["pinch"]["curl"] + samples["open"]["curl"]

    print("\n=== measurements ===")
    for name in ("pinch", "open", "fist"):
        ratio = samples[name]["ratio"]
        curl = samples[name]["curl"]
        print(
            f"{name:5s}: n={len(ratio):3d}  ratio p5={_percentile(ratio,5):.3f} "
            f"p95={_percentile(ratio,95):.3f}  curl p5={_percentile(curl,5):.3f} "
            f"p95={_percentile(curl,95):.3f}"
        )

    if len(pinch) < 3 or len(open_) < 3:
        print("\nnot enough samples for pinch/open; capture at least 3 each")
        return 1

    pinch_hi = max(_percentile(pinch, 95), max(pinch))
    open_lo = min(_percentile(open_, 5), min(open_))
    if pinch_hi >= open_lo:
        print(f"\nWARNING: pinch/open overlap (pinch p95 {pinch_hi:.3f} >= open min {open_lo:.3f})")
        gap = 0.02
        close = pinch_hi
        release = pinch_hi + gap
    else:
        close = pinch_hi + 0.25 * (open_lo - pinch_hi)
        release = pinch_hi + 0.65 * (open_lo - pinch_hi)

    suggested = {
        "pinch_close": round(close, 3),
        "pinch_open": round(release, 3),
    }
    if len(fist_curl) >= 3 and len(other_curl) >= 3:
        fist_hi = max(_percentile(fist_curl, 95), max(fist_curl))
        other_lo = min(_percentile(other_curl, 5), min(other_curl))
        if fist_hi < other_lo:
            suggested["fist_curl_enter"] = round(fist_hi + 0.25 * (other_lo - fist_hi), 3)
            suggested["fist_curl_exit"] = round(fist_hi + 0.65 * (other_lo - fist_hi), 3)
        else:
            print(f"WARNING: fist overlaps other gestures (fist {fist_hi:.3f} >= other {other_lo:.3f})")

    print("\n=== suggested config/safety.yaml ===")
    for key, value in suggested.items():
        print(f"{key}: {value}")

    if write:
        path = config.root / "config" / "safety.yaml"
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        data.update(suggested)
        path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
        print(f"\nwrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
