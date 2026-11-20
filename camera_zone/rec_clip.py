#!/usr/bin/env python3
"""录一段原始彩色+深度帧存盘。用来给算法改动做离线对照。

为什么要这个：人站的位置、动作每次都不一样，分两次录像跑 A/B 对照
得不出结论（11-11 踩过：两段 20 秒里一段人腿压根没进画面）。
存下原始帧，之后所有改动都在同一段数据上重放，差异才是算法带来的。

    python3 rec_clip.py --seconds 20 --out clips/legs_1111
    python3 replay_ab.py clips/legs_1111        # 开/关骨架反推对照
"""
import argparse, os, time
from camera_backend import open_camera
from clip_io import ClipWriter
from zone_monitor import W, H, FPS

ap = argparse.ArgumentParser()
ap.add_argument("--seconds", type=float, default=20.0)
ap.add_argument("--out", default="clips/clip")
ap.add_argument("--backend", default=None, choices=["orbbec", "realsense"])
a = ap.parse_args()

os.makedirs(os.path.dirname(os.path.abspath(a.out)) or ".", exist_ok=True)
cam = open_camera(a.backend, W, H, FPS)
w = ClipWriter(a.out, cam.intr)
t0 = time.time()
print(f"开始录 {a.seconds:.0f} 秒，站到相机前走动一下…")
try:
    for fr in cam.frames():
        el = time.time() - t0
        if el > a.seconds:
            break
        w.write(fr.color, fr.depth, el)      # 边录边落盘，内存不随时长涨
        if len(w.ts) % 60 == 0:
            print(f"  {el:.0f}s / {len(w.ts)} 帧")
finally:
    cam.close()

n = w.close()
mb = sum(os.path.getsize(os.path.join(w.dir, f))
         for f in os.listdir(w.dir)) / 1e6
print(f"✓ {n} 帧 → {w.dir}/（{mb:.0f} MB，{n/a.seconds:.1f} FPS）")
