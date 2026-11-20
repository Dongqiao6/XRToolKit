#!/usr/bin/env python3
"""录一段原始彩色+深度帧，供离线回放和感知改动前后对照。

    ros2 run vr_safe_perception record_clip --seconds 20
    ros2 run vr_safe_perception record_clip --seconds 30 --out out/clips/legs

每帧写两个文件：00000.jpg（彩色）和 00000.png（深度，毫米），外加
meta.json（内参和时间戳），格式与 camera_zone 的 clip_io 完全一致。
"""

import argparse
import os
import sys
import time

from ..camera_backend import open_camera
from ..clip_io import ClipWriter
from ..config import ConfigError, load_config
from ..paths import default_output_dir, default_perception_config


def main(argv=None):
    ap = argparse.ArgumentParser(description="录 VR_safe 相机片段")
    ap.add_argument("--config", default=default_perception_config())
    ap.add_argument("--out", default="", help="片段目录（默认 out/clips/clip_<时间>）")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--backend", choices=["orbbec", "realsense", "auto"], default=None)
    args = ap.parse_args(argv)
    if args.seconds <= 0:
        ap.error("--seconds 必须大于 0")

    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        print(f"✗ perception 配置不合法：{exc}", file=sys.stderr)
        return 2
    backend = args.backend or cfg["camera"]["backend"]
    out = args.out or os.path.join(
        default_output_dir(), "clips", time.strftime("clip_%m%d_%H%M%S"))

    try:
        cam = open_camera(backend, cfg["camera"]["width"],
                          cfg["camera"]["height"], cfg["camera"]["fps"])
    except Exception as exc:
        print(f"✗ 相机打不开：{exc}", file=sys.stderr)
        return 1

    writer = ClipWriter(out, cam.intr)
    t0 = time.monotonic()
    n = 0
    try:
        print(f"开始录 {args.seconds:.0f} 秒 → {writer.dir}，站到相机前走动一下…")
        for frame in cam.frames():
            elapsed = time.monotonic() - t0
            if elapsed > args.seconds:
                break
            writer.write(frame.color, frame.depth, elapsed)
            n += 1
            if n % 60 == 0:
                print(f"  {elapsed:.0f}s / {n} 帧", flush=True)
    except KeyboardInterrupt:
        print("  （收到中断，正常收尾）")
    finally:
        cam.close()
        count = writer.close()

    actual = max(time.monotonic() - t0, 1e-6)
    print(f"✓ {count} 帧 → {writer.dir}/（{count / actual:.1f} FPS）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
