"""外参标定有效性检查（生产拒绝、开发显式降级）。

规则来自 docs/design.md §8：

  - 外参文件缺失            → 生产拒绝启动
  - ok 不是 true            → 生产拒绝启动
  - 标定时的地面和当前地面
    差超过阈值（默认 0.5°、2 cm） → 生产拒绝启动
  - 显式 --allow-uncalibrated 才退回「设计值 + 地面摆正」，并在日志里说清楚

外参文件的格式沿用 camera_zone 的 extrinsic_calib.json：
    {"ok": bool, "T_cam_base": 4×4, "plane": {"origin": [...], "normal": [...]}, ...}
T_cam_base 是「基座系 → 相机光学系」。
"""

import json
import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np


class CalibrationError(RuntimeError):
    """标定不可用，且调用方没有允许降级。"""


@dataclass
class CalibrationResult:
    T_cam_base: Optional[np.ndarray]
    source: str
    used_calibrated: bool
    warnings: List[str] = field(default_factory=list)
    ground_angle_deg: Optional[float] = None
    ground_offset_m: Optional[float] = None


def read_plane(zone_config_path: str) -> Dict[str, Any]:
    """读 zone_config.json 里的地面平面（origin/normal）。"""
    try:
        with open(zone_config_path, encoding="utf-8") as f:
            data = json.load(f)
    except Exception as exc:
        raise CalibrationError(f"地面配置读不了：{zone_config_path!r}：{exc}") from exc
    plane = (data.get("plane") or {}) if isinstance(data, dict) else {}
    origin, normal = plane.get("origin"), plane.get("normal")
    if not origin or not normal:
        raise CalibrationError(
            f"地面配置里没有 plane.origin/plane.normal：{zone_config_path!r}"
            "（先跑 calibrate_zone 标地面）")
    return {"origin": [float(v) for v in origin],
            "normal": [float(v) for v in normal]}


def plane_diff(plane_a: Dict[str, Any], plane_b: Dict[str, Any]):
    """两份地面差多少：(法向夹角 度, 原点到对方平面的距离差 米)。"""
    na = np.asarray(plane_a["normal"], float)
    nb = np.asarray(plane_b["normal"], float)
    na = na / max(float(np.linalg.norm(na)), 1e-12)
    nb = nb / max(float(np.linalg.norm(nb)), 1e-12)
    if float(na @ nb) < 0:
        nb = -nb
    dot = float(np.clip(na @ nb, -1.0, 1.0))
    # 同一份法向做点积会得到 0.9999999999999999，直接 acos 会冒出 1e-4 度
    # 的假误差；在浮点精度内认为完全一致。
    angle = 0.0 if dot >= 1.0 - 1e-12 else math.degrees(math.acos(dot))
    da = float(np.asarray(plane_a["origin"], float) @ na)
    db = float(np.asarray(plane_b["origin"], float) @ nb)
    return angle, abs(da - db)


def _check_transform(T: np.ndarray) -> None:
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise CalibrationError("T_cam_base 不是有限的 4×4 矩阵")
    if float(np.abs(T[3] - np.array([0.0, 0.0, 0.0, 1.0])).max()) > 1e-6:
        raise CalibrationError("T_cam_base 最后一行必须是 [0, 0, 0, 1]")
    R = T[:3, :3]
    ortho = float(np.abs(R @ R.T - np.eye(3)).max())
    det = float(np.linalg.det(R))
    if ortho > 1e-4 or abs(det - 1.0) > 1e-4:
        raise CalibrationError(
            f"T_cam_base 的旋转部分不合法（正交误差 {ortho:.2e}，行列式 {det:+.6f}）")


def load_extrinsic(path: str) -> Dict[str, Any]:
    """读并检查 extrinsic_calib.json 的结构。不检查地面是否匹配。"""
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    except Exception as exc:
        raise CalibrationError(f"外参文件读不了：{path!r}：{exc}") from exc
    if not isinstance(data, dict):
        raise CalibrationError("外参文件顶层必须是映射")
    if data.get("ok") is not True:
        reasons = data.get("not_ok_reasons") or data.get("reason") or "未说明原因"
        if isinstance(reasons, (list, tuple)):
            reasons = "；".join(str(x) for x in reasons)
        raise CalibrationError(f"外参标定未通过（ok != true）：{reasons}")
    try:
        T = np.asarray(data["T_cam_base"], dtype=float).reshape(4, 4)
    except Exception as exc:
        raise CalibrationError(f"外参文件缺少合法的 T_cam_base：{exc}") from exc
    _check_transform(T)
    plane = data.get("plane") or {}
    if not plane.get("origin") or not plane.get("normal"):
        raise CalibrationError("外参文件里没有 plane（标定时的地面），无法核对地面是否匹配")
    return {"T_cam_base": T, "plane": plane, "raw": data}


def design_T_cam_base(camera_id: str, plane: Optional[Dict[str, Any]] = None):
    """开发降级用的「设计值 + 地面摆正」。

    设计值来自 JAKA 仿真包里的 sensors.json；有实测地面时按地面立直并落到地面。
    没有仿真资源就返回 T=None，让调用方决定是否还能继续（polygon 不需要外参，
    reach 需要）。
    """
    from .robot_frames import RobotFrames, align_to_floor

    T = np.linalg.inv(RobotFrames.load().T_base_optical(camera_id))
    if plane is not None:
        T, _lift, _deg = align_to_floor(T, plane["origin"], plane["normal"])
    return T


def validate_calibration(extrinsic_file: str, zone_config_file: str,
                         camera_id: str, require_valid: bool,
                         max_ground_angle_deg: float,
                         max_ground_offset_m: float,
                         allow_uncalibrated: bool = False) -> CalibrationResult:
    """检查外参；生产不通过就抛 CalibrationError，开发降级返回设计值。

    require_valid=False 与 allow_uncalibrated=True 等效，都表示调用方已经
    明确接受降级。两者只要有一个为真就不抛。
    """
    allow = (not bool(require_valid)) or bool(allow_uncalibrated)
    warnings: List[str] = []

    # 当前地面：生产必须能读出来，否则「地面不匹配」无从谈起。
    try:
        current_plane = read_plane(zone_config_file)
    except CalibrationError as exc:
        if not allow:
            raise
        warnings.append(f"开发降级：{exc}")
        current_plane = None

    try:
        ext = load_extrinsic(extrinsic_file)
        if current_plane is not None:
            angle, dist = plane_diff(current_plane, ext["plane"])
            if angle > max_ground_angle_deg or dist > max_ground_offset_m:
                raise CalibrationError(
                    "外参标定时的地面和当前地面不一致："
                    f"差 {angle:.2f}°、{dist * 100:.1f} cm"
                    f"（门槛 {max_ground_angle_deg:.2f}°、"
                    f"{max_ground_offset_m * 100:.1f} cm）。相机可能挪过，"
                    "需要重新标定")
            return CalibrationResult(
                T_cam_base=ext["T_cam_base"],
                source=f"标定值 {os.path.basename(extrinsic_file)}",
                used_calibrated=True,
                warnings=warnings,
                ground_angle_deg=angle,
                ground_offset_m=dist,
            )
        warnings.append("开发降级：没有当前地面，无法核对外参地面一致性")
        return CalibrationResult(
            T_cam_base=ext["T_cam_base"],
            source=f"标定值 {os.path.basename(extrinsic_file)}（地面未核对）",
            used_calibrated=True,
            warnings=warnings,
        )
    except CalibrationError as exc:
        if not allow:
            raise
        warnings.append(f"开发降级：{exc}")

    # 降级到设计值。没有仿真资源时 T=None：polygon 还能跑，reach 会在建区域时报错。
    try:
        T = design_T_cam_base(camera_id, current_plane)
        warnings.append("已退回设计值 + 地面摆正（不是实测外参，只用于开发）")
        return CalibrationResult(
            T_cam_base=T,
            source="设计值 + 地面摆正（开发降级）",
            used_calibrated=False,
            warnings=warnings,
        )
    except Exception as exc:
        warnings.append(f"设计外参也用不了：{exc}")
        return CalibrationResult(
            T_cam_base=None,
            source="无可用外参（开发降级）",
            used_calibrated=False,
            warnings=warnings,
        )
