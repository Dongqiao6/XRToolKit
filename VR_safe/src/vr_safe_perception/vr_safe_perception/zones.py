"""保护区构建：polygon（地面多边形）和 reach（机械臂可达空间）。

判定核心仍然用 zone_monitor 的 Ground / ReachZone，两者接口一致
（inside / height / dist_to_edge），judge 不需要知道当前是哪种区域。
"""

import os
from dataclasses import dataclass
from typing import Any, Dict

import numpy as np

from . import zone_monitor as zm
from .calibration import CalibrationResult
from .jaka_model import JakaModel


class ZoneError(RuntimeError):
    """保护区不可用。"""


@dataclass
class ZoneResult:
    zone: Any
    mode: str
    description: Dict[str, Any]


def build_polygon(zone_file: str) -> ZoneResult:
    if not os.path.isfile(zone_file):
        raise ZoneError(
            f"多边形区域文件不存在：{zone_file!r}（先跑 calibrate_zone --refit --save）")
    try:
        ground = zm.Ground.load(zone_file)
    except Exception as exc:
        raise ZoneError(f"多边形区域文件读不了：{zone_file!r}：{exc}") from exc
    if len(ground.corners3d) < 3:
        raise ZoneError(f"多边形区域至少要有 3 个角点：{zone_file!r}")
    return ZoneResult(
        zone=ground,
        mode="polygon",
        description={
            "zone_file": zone_file,
            "corners3d": ground.corners3d.tolist(),
            "normal": ground.normal.tolist(),
        },
    )


def build_reach(calib: CalibrationResult, margin_m: float, samples: int,
                camera_id: str = "top") -> ZoneResult:
    if calib.T_cam_base is None:
        raise ZoneError(
            "reach 区域需要相机↔基座外参，但外参不可用；"
            "请先标定，或不要用 --allow-uncalibrated 跑 reach")
    robot = JakaModel()
    spheres = robot.reach_spheres(margin=float(margin_m), samples=int(samples))
    if not spheres:
        raise ZoneError("JAKA 模型没有给出可达球，检查 URDF 是否完整")
    T = np.asarray(calib.T_cam_base, float)

    # 可达球在 base_link 系；judge 用的是相机光学系，逐球换算。
    cam_spheres = []
    for origin, radius, *_ in spheres:
        p = T[:3, :3] @ np.asarray(origin, float) + T[:3, 3]
        cam_spheres.append((p, float(radius)))
    zone = zm.ReachZone(cam_spheres)
    return ZoneResult(
        zone=zone,
        mode="reach",
        description={
            "camera_id": camera_id,
            "margin_m": float(margin_m),
            "samples": int(samples),
            "spheres": [
                {"origin_cam": [round(float(x), 4) for x in o],
                 "radius_m": round(float(r), 4)}
                for o, r in cam_spheres],
        },
    )


def build_zone(zone_mode: str, paths: Dict[str, str], calib: CalibrationResult,
               margin_m: float, samples: int, camera_id: str) -> ZoneResult:
    """按 zone_mode 建区域。默认 polygon。"""
    if zone_mode == "polygon":
        return build_polygon(paths["zone_file"])
    if zone_mode == "reach":
        return build_reach(calib, margin_m, samples, camera_id)
    raise ZoneError(f"未知的 zone_mode：{zone_mode!r}")
