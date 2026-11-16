"""Independent palm reference in the calibrated camera's proper rotation frame."""
from collections import deque

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def proper_rotation(value):
    r = np.asarray(value, float)
    return (r.shape == (3, 3) and np.isfinite(r).all() and
            np.allclose(r.T @ r, np.eye(3), atol=1e-6) and np.linalg.det(r) > .999999)


class PalmReference:
    def __init__(self, settings, duration=.6):
        self.settings, self.duration = settings, duration
        if not np.isfinite(duration) or duration <= 0:
            raise ValueError("Invalid palm reference duration")
        self.history = deque(maxlen=180)
        self.reference = None
        self.timestamp = -1e9
        self.points = self.rotation = None
        self.reason = "等待双侧完整掌部观测"
        self.sample_count = 0

    def observe(self, geometry, timestamp):
        if timestamp <= self.timestamp:
            return
        if timestamp-self.timestamp > self.settings["stale_after_s"]:
            self.history.clear()
        self.timestamp = timestamp
        g = geometry
        valid = (g.get("valid", False) and g.get("rotation") is not None and
                 np.shape(g.get("points")) == (4, 3) and np.isfinite(g["points"]).all() and
                 proper_rotation(g["rotation"]) and np.all(g["positive_depth"]) and
                 np.isfinite(g["ray_gap_m"]).all() and np.isfinite(g["ray_angle_sine"]).all() and
                 max(g["ray_gap_m"]) <= self.settings["orientation_ray_gap_m"] and
                 min(g["ray_angle_sine"]) >= self.settings["min_ray_angle_sine"])
        if not valid:
            self.history.clear()
            self.points = self.rotation = None
            self.reason = "掌部点缺失或几何不一致；姿态不可用"
            return
        self.points = np.array(g["points"], float, copy=True)
        self.rotation = np.array(g["rotation"], float, copy=True)
        self.history.append((timestamp, self.points[1:].mean(axis=0), self.rotation.copy()))
        while self.history and timestamp-self.history[0][0] > self.duration+.15:
            self.history.popleft()
        self.reason = ""

    def current(self, now):
        return self.rotation is not None and 0 <= now-self.timestamp <= self.settings["stale_after_s"]

    def candidate(self, now):
        if not self.current(now):
            raise ValueError(self.reason or "掌部观测过期；基准仍保留")
        rows = list(self.history)
        if len(rows) < 6 or rows[-1][0]-rows[0][0] < self.duration:
            raise ValueError(f"保持舒服手姿约{self.duration:.1f}秒，再设基准")
        centers = np.array([r[1] for r in rows])
        rotations = Rotation.from_matrix(np.array([r[2] for r in rows]))
        mean = rotations.mean()
        spread = np.degrees((rotations*mean.inv()).magnitude())
        if np.max(np.linalg.norm(centers-centers.mean(axis=0), axis=1)) > self.settings["init_position_range_m"]:
            raise ValueError("手还在平移；停稳后可设姿态基准")
        if spread.max() > self.settings["init_angle_deg"]:
            raise ValueError("手还在转动；停稳后可设姿态基准")
        return mean.as_matrix(), len(rows)

    def capture(self, now):
        reference, count = self.candidate(now)
        self.reference, self.sample_count = reference, count

    def angle_deg(self, now):
        if not self.current(now) or self.reference is None:
            return None
        # No display Z reflection enters Rotation: both operands are in camera SO(3).
        return float(np.degrees(Rotation.from_matrix(self.rotation @ self.reference.T).magnitude()))


def overlay_palm(image, palm, calibration, side, now):
    if not palm.current(now):
        return
    center = palm.points[1:].mean(axis=0)
    points = np.vstack((center, center+palm.rotation.T*.065))
    if side:
        points = points @ np.asarray(calibration["R"]).T + np.asarray(calibration["T"]).reshape(3)
    if not np.isfinite(points).all() or np.any(points[:, 2] <= 0):
        return
    xy = cv2.projectPoints(points, np.zeros(3), np.zeros(3),
                           np.asarray(calibration["K2" if side else "K1"]),
                           np.asarray(calibration["D2" if side else "D1"]))[0][:, 0]
    xy *= np.array(image.shape[1::-1])/np.array(calibration["image_size"])
    if not np.isfinite(xy).all() or np.max(np.abs(xy)) > 10000:
        return
    xy = np.rint(xy).astype(int)
    for end, color, label in zip(xy[1:], ((80,80,240),(70,220,70),(255,175,65)), ("W","F","N")):
        cv2.arrowedLine(image, tuple(xy[0]), tuple(end), (0,0,0), 5, tipLength=.2)
        cv2.arrowedLine(image, tuple(xy[0]), tuple(end), color, 2, tipLength=.2)
        cv2.putText(image, label, tuple(end), 0, .5, (0,0,0), 3)
        cv2.putText(image, label, tuple(end), 0, .5, color, 1)
    cv2.circle(image, tuple(xy[0]), 6, (0,0,0), 3)
    cv2.circle(image, tuple(xy[0]), 5, (255,255,255), 2)
