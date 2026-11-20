#!/usr/bin/env python3
"""录像片段的读写。彩色存 JPEG、深度存 16 位 PNG（毫米），一帧两个文件。

为什么不用一个 npz 装：npz 要先把整段堆在内存里再压缩。586 帧原始数据
1.2 GB，leju 上只剩 4 GB 可用内存时写出来的文件 CRC 就坏了（11-11 踩过，
报 Bad CRC-32 for file 'color.npy'）。边录边写，内存占用和录多久无关。

深度存毫米整数：精度 1 mm，比深度相机本身的误差小一个量级，不损失信息。
"""
import json
import os

import cv2
import numpy as np

from camera_backend import Intrinsics


class ClipWriter:
    def __init__(self, path, intr, jpeg_q=92):
        self.dir = path[:-4] if path.endswith(".npz") else path
        os.makedirs(self.dir, exist_ok=True)
        self.intr, self.q, self.ts = intr, jpeg_q, []

    def write(self, color, depth_m, t):
        n = len(self.ts)
        cv2.imwrite(f"{self.dir}/{n:05d}.jpg", color,
                    [cv2.IMWRITE_JPEG_QUALITY, self.q])
        mm = np.clip(np.nan_to_num(depth_m) * 1000.0, 0, 65535).astype(np.uint16)
        cv2.imwrite(f"{self.dir}/{n:05d}.png", mm)
        self.ts.append(float(t))

    def close(self):
        i = self.intr
        with open(f"{self.dir}/meta.json", "w") as f:
            json.dump({"fx": i.fx, "fy": i.fy, "ppx": i.ppx, "ppy": i.ppy,
                       "width": i.width, "height": i.height,
                       "coeffs": list(i.coeffs), "ts": self.ts,
                       "depth_unit": "mm(uint16 png)"}, f)
        return len(self.ts)


class ClipReader:
    """逐帧读。也认旧的 .npz（整段读进内存）。"""

    def __init__(self, path):
        self.npz = None
        if path.endswith(".npz") and os.path.isfile(path):
            d = np.load(path)
            fx, fy, ppx, ppy, w, h = d["intr"]
            self.intr = Intrinsics(fx, fy, ppx, ppy, int(w), int(h),
                                   tuple(d["coeffs"]) if "coeffs" in d else (0.,)*5)
            self.ts = list(d["ts"])
            self.npz = d
            return
        self.dir = path[:-4] if path.endswith(".npz") else path
        with open(f"{self.dir}/meta.json") as f:
            m = json.load(f)
        self.intr = Intrinsics(m["fx"], m["fy"], m["ppx"], m["ppy"],
                               int(m["width"]), int(m["height"]),
                               tuple(m["coeffs"]))
        self.ts = m["ts"]

    def __len__(self):
        return len(self.ts)

    def __iter__(self):
        if self.npz is not None:
            for c, d, t in zip(self.npz["color"], self.npz["depth"], self.ts):
                yield c, d, float(t)
            return
        for n, t in enumerate(self.ts):
            c = cv2.imread(f"{self.dir}/{n:05d}.jpg", cv2.IMREAD_COLOR)
            d = cv2.imread(f"{self.dir}/{n:05d}.png",
                           cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0
            yield c, d, float(t)
