#!/usr/bin/env python3
"""Check that the configured D435 can actually be opened.

Prints the devices the SDK can see, then tries to start the exact streams from
config/vision.yaml and reports the intrinsics. If this passes, vision_bridge
can open the camera; if it fails, the message says why.

    source env.sh
    python scripts/check_camera.py [--serial SERIAL]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "vision_bridge"))

from vision_bridge.camera import RealSenseSource, list_devices  # noqa: E402
from vision_bridge.config import load_config  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default=None, help="override camera.serial")
    args = parser.parse_args()

    config = load_config(ROOT)
    devices = list_devices()
    print(f"visible devices: {devices or 'none'}")
    if not devices:
        print("no RealSense device is visible; check USB and udev rules")
        return 1

    serial = args.serial if args.serial is not None else config.vision.camera.serial
    if serial and serial not in devices:
        print(f"WARNING: configured serial {serial} is not in the visible list")

    camera = RealSenseSource(config.vision.camera, serial_override=args.serial)
    try:
        camera.start()
    except Exception as exc:
        print(f"FAILED to start: {exc}")
        if serial in devices:
            print(
                "the camera is visible but cannot be opened; most often another "
                "process still holds it: pkill -f vision_bridge.node"
            )
        return 1

    intr = camera._intrinsics
    print(f"opened serial {camera.serial}")
    print(f"colour {intr.width}x{intr.height}  fx={intr.fx:.1f} fy={intr.fy:.1f} "
          f"cx={intr.cx:.1f} cy={intr.cy:.1f}")
    for _ in range(5):
        frame = camera.read()
    if frame is None:
        print("FAILED: no frames returned")
        camera.stop()
        return 1
    print(f"frame {frame.bgr.shape[1]}x{frame.bgr.shape[0]}  "
          f"depth {frame.depth_m.shape[1]}x{frame.depth_m.shape[0]}  "
          f"valid depth ratio {float((frame.depth_m > 0).mean()):.2f}")
    camera.stop()
    print("camera ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
