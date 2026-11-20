#!/usr/bin/env python3
"""区域预览：开相机取若干帧，把 polygon/reach 保护区画上去，存 JPG。

不依赖 rerun、table_zones 的 UI。默认只存一张预览图；加 --show 才开
OpenCV 窗口（现场调试用）。

    ros2 run vr_safe_perception preview_zone
    ros2 run vr_safe_perception preview_zone --show --allow-uncalibrated
"""

import argparse
import os
import sys

import cv2

from ..config import ConfigError, load_config
from ..paths import default_output_dir, default_perception_config


def _draw_reach(vis, zone, intr, color):
    for origin, radius in zone.spheres:
        q = intr.project(origin)
        if q is None:
            continue
        x, y = int(round(q[0])), int(round(q[1]))
        if origin[2] <= 0.05:
            continue
        r_px = max(2, int(radius * intr.fx / max(origin[2], 1e-3)))
        cv2.circle(vis, (x, y), r_px, color, 2, cv2.LINE_AA)
        cv2.putText(vis, f"{radius:.2f}m", (x + 6, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def main(argv=None):
    ap = argparse.ArgumentParser(description="预览 VR_safe 保护区")
    ap.add_argument("--config", default=default_perception_config())
    ap.add_argument("--backend", choices=["orbbec", "realsense", "auto"], default=None)
    ap.add_argument("--zone-mode", choices=["polygon", "reach"], default=None)
    ap.add_argument("--allow-uncalibrated", action="store_true")
    ap.add_argument("--frames", type=int, default=20, help="取多少帧后存图")
    ap.add_argument("--out", default=os.path.join(default_output_dir(),
                                                  "zone_preview.jpg"))
    ap.add_argument("--show", action="store_true", help="开窗口实时看（q 退出）")
    args = ap.parse_args(argv)

    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        print(f"✗ perception 配置不合法：{exc}", file=sys.stderr)
        return 2
    if args.backend:
        cfg["camera"]["backend"] = args.backend
    if args.zone_mode:
        cfg["calibration"]["zone_mode"] = args.zone_mode

    # 重依赖（mediapipe/相机 SDK）放到参数解析之后 import，--help 不要求部署齐全。
    from ..runtime import prepare_zone
    try:
        _paths, _calib, zone_result = prepare_zone(
            cfg, args.config, allow_uncalibrated=args.allow_uncalibrated,
            log=lambda m: print(m, flush=True))
    except Exception as exc:
        print(f"✗ 保护区准备失败：{exc}", file=sys.stderr)
        return 2

    from .. import zone_monitor as zm
    from ..camera_backend import open_camera
    try:
        cam = open_camera(cfg["camera"]["backend"],
                          cfg["camera"]["width"], cfg["camera"]["height"],
                          cfg["camera"]["fps"])
    except Exception as exc:
        print(f"✗ 相机打不开：{exc}", file=sys.stderr)
        return 1

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    color = (0, 220, 0)
    vis, n = None, 0
    try:
        for frame in cam.frames():
            n += 1
            vis = frame.color.copy()
            if zone_result.mode == "polygon":
                zm.draw_zone(vis, zone_result.zone, frame.intr, color)
            else:
                _draw_reach(vis, zone_result.zone, frame.intr, color)
            text = (f"{zone_result.mode}  "
                    f"h={cfg['zone']['height_m']:.1f}m  "
                    f"near={cfg['zone']['near_m']:.1f}m  "
                    f"source={cfg['node']['source_id']}")
            cv2.putText(vis, text, (8, 26), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, color, 2, cv2.LINE_AA)
            if args.show:
                cv2.imshow("VR_safe preview_zone（q 退出）", vis)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
            if not args.show and n >= max(1, args.frames):
                break
    finally:
        cam.close()
        if args.show:
            cv2.destroyAllWindows()

    if vis is None:
        print("✗ 一帧都没取到，没写预览图", file=sys.stderr)
        return 1
    ok = cv2.imwrite(args.out, vis)
    if not ok:
        print(f"✗ 预览图写不进去：{args.out}", file=sys.stderr)
        return 1
    print(f"✓ 预览图 → {args.out}（{n} 帧，区域 {zone_result.mode}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
