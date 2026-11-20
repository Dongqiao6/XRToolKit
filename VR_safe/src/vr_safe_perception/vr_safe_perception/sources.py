"""帧源：实时相机和录像回放对上层暴露同一种接口。

相机打不开、或者连续读帧失败时抛 SourceError，由 observation_node 切到
「持续发 UNKNOWN」的兜底循环；不要把异常吞掉然后退出——静默退出在安全
系统里等于把故障留给 supervisor 的观察超时去猜。
"""

import time

import numpy as np

from .camera_backend import Frame
from .clip_io import ClipReader


class SourceError(RuntimeError):
    """帧源不可用。"""


class CameraSource:
    """实时相机帧源。open() 才真正打开相机，启动失败可以由上层重试。"""

    def __init__(self, backend=None, width=848, height=480, fps=30,
                 max_consecutive_errors=5):
        self.backend = None if backend in (None, "auto") else backend
        self.width, self.height, self.fps = int(width), int(height), int(fps)
        self.max_consecutive_errors = int(max_consecutive_errors)
        self.cam = None

    def open(self):
        from .camera_backend import open_camera
        try:
            self.cam = open_camera(self.backend, self.width, self.height, self.fps)
        except Exception as exc:
            raise SourceError(f"相机打不开：{exc}") from exc
        return self

    def frames(self):
        """逐帧产出。连续失败到上限时抛 SourceError。"""
        if self.cam is None:
            self.open()
        failures = 0
        while True:
            try:
                frame = self.cam.read()
            except Exception as exc:
                failures += 1
                if failures >= self.max_consecutive_errors:
                    raise SourceError(f"相机连续读帧失败：{exc}") from exc
                time.sleep(0.02)
                continue
            if frame is None:
                failures += 1
                if failures >= self.max_consecutive_errors:
                    raise SourceError("相机连续返回空帧")
                time.sleep(0.02)
                continue
            failures = 0
            yield frame

    def close(self):
        if self.cam is not None:
            try:
                self.cam.close()
            except Exception:
                pass
            self.cam = None


class ClipSource:
    """录像回放帧源。

    保持片的原始时间戳节奏（--replay-fast 可关掉），循环回放时把时间戳
    整体后移，保证传给 MediaPipe 的时间单调递增。
    """

    def __init__(self, path, loop=False, realtime=True, speed=1.0,
                 max_frames=0, sleep=time.sleep):
        try:
            self.reader = ClipReader(path)
        except Exception as exc:
            raise SourceError(f"录像打不开：{path!r}：{exc}") from exc
        if len(self.reader) <= 0:
            raise SourceError(f"录像里没有帧：{path!r}")
        self.path = path
        self.loop = bool(loop)
        self.realtime = bool(realtime)
        self.speed = max(float(speed), 1e-3)
        self.max_frames = int(max_frames)
        self._sleep = sleep

    def frames(self):
        sent = 0
        offset = 0.0
        last_ts = None
        prev_ts = None
        while True:
            for color, depth, ts in self.reader:
                if self.max_frames and sent >= self.max_frames:
                    return
                ts = float(ts)
                if last_ts is not None and ts < last_ts:
                    # 新的一轮：把上一轮最后一个时间戳当作起点继续往后走
                    offset += last_ts + 1.0 / 30.0
                last_ts = ts
                t = offset + ts
                if self.realtime and sent and prev_ts is not None:
                    # 用原始时间差睡眠，不依赖上一帧处理花了多久；
                    # 处理比录像慢时自然降速，不会把时间轴压短。
                    self._sleep(max(0.0, (ts - prev_ts) / self.speed))
                prev_ts = ts
                yield Frame(np.asarray(color), np.asarray(depth, dtype=np.float32),
                            self.reader.intr, t)
                sent += 1
            if not self.loop:
                return

    def close(self):
        pass
