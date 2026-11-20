"""运行期组装：校验标定 → 建保护区 → 配阈值 → 加载 MediaPipe 模型。

只有真正要跑相机/录像的入口才 import 这个模块（它会 import zone_monitor，
进而 import mediapipe）。配置校验、五档映射、回放流水线的单元测试不碰它。
"""

import os
from dataclasses import dataclass
from typing import Any, Callable, Dict

import cv2
import numpy as np

from . import camera_backend
from . import zone_monitor as zm
from .calibration import (CalibrationError, CalibrationResult,
                          read_plane, validate_calibration)
from .config import resolved_paths
from .pipeline import ObservationPipeline
from .zones import ZoneResult, build_zone


def apply_config(cfg: Dict[str, Any], paths: Dict[str, str]) -> None:
    """把 perception.yaml 的阈值写进 zone_monitor 的模块级参数。

    zone_monitor 原先是靠环境变量读这些值的（camera_zone 的历史做法）；
    这里在启动时统一覆盖，避免运行中改环境变量导致前后帧不一致。
    """
    zm.ZONE_H = float(cfg["zone"]["height_m"])
    zm.NEAR_M = float(cfg["zone"]["near_m"])
    zm.MIN_DEPTH_VALID = float(cfg["quality"]["min_depth_valid"])
    zm.MAX_NEAR_COVER = float(cfg["quality"]["max_near_cover"])
    zm.MIN_TEXTURE = float(cfg["quality"]["min_texture"])
    zm.QualityGate.BAD_S = float(cfg["quality"]["bad_s"])
    zm.QualityGate.GOOD_S = float(cfg["quality"]["good_s"])
    zm.MIN_GOOD_LMS_UPPER = int(cfg["pose"]["min_good_lms_upper"])
    zm.POSE_CONF = float(cfg["pose"]["pose_conf"])
    zm.MAX_PERSONS = int(cfg["models"]["max_persons"])
    zm.POSE_BACKEND = str(cfg["models"]["pose_backend"])
    zm.RELAX_N = int(cfg["grader"]["relax_n"])

    zm.BODY_PARTS = str(cfg["pose"]["body_parts"])
    zm.LIMB = zm.LIMB_UPPER if zm.BODY_PARTS == "upper" else zm.LIMB_FULL

    zm.POSE_MODEL = paths["pose_model"]
    zm.HAND_MODEL = paths["hand_model"]
    zm.ZONE_CFG_PATH = paths["zone_config"]
    os.environ["ZONE_CFG"] = paths["zone_config"]

    camera_backend.LATEST_ONLY = bool(cfg["camera"]["latest_only"])

    # 髋部可信检查需要相机系里的「上」。它来自实测地面；没有地面就退回
    # judge 里的光轴深度比较。
    try:
        plane = read_plane(paths["zone_config"])
        n = np.asarray(plane["normal"], float)
        n = n / (np.linalg.norm(n) + 1e-12)
        zm.UP_CAM = n if n[1] < 0 else -n
    except CalibrationError:
        zm.UP_CAM = None


def prepare_zone(cfg: Dict[str, Any], config_path: str,
                 allow_uncalibrated: bool = False,
                 log: Callable[[str], None] = print):
    """校验标定并建区域。返回 (paths, calibration, zone_result)。"""
    paths = resolved_paths(cfg, config_path)
    cal = cfg["calibration"]
    result = validate_calibration(
        extrinsic_file=paths["extrinsic_file"],
        zone_config_file=paths["zone_config"],
        camera_id=str(cal["camera_id"]),
        require_valid=bool(cal["require_valid"]),
        max_ground_angle_deg=float(cal["max_ground_angle_deg"]),
        max_ground_offset_m=float(cal["max_ground_offset_m"]),
        allow_uncalibrated=bool(allow_uncalibrated),
    )
    for warning in result.warnings:
        log(f"⚠ {warning}")
    log(f"外参：{result.source}")
    if result.ground_angle_deg is not None:
        log(f"外参地面核对：差 {result.ground_angle_deg:.2f}°、"
            f"{result.ground_offset_m * 100:.1f} cm")

    zone = build_zone(
        zone_mode=str(cal["zone_mode"]),
        paths=paths,
        calib=result,
        margin_m=float(cfg["reach"]["margin_m"]),
        samples=int(cfg["reach"]["samples"]),
        camera_id=str(cal["camera_id"]),
    )
    apply_config(cfg, paths)
    if zone.mode == "polygon":
        log(f"保护区：polygon {paths['zone_file']} "
            f"（{len(zone.zone.corners3d)} 个角点）")
    else:
        log(f"保护区：reach，可达球 {len(zone.description.get('spheres', []))} 个，"
            f"余量 {cfg['reach']['margin_m']} m")
    return paths, result, zone


class MediaPipeInferencer:
    """MediaPipe 姿态 + 手部推理，输出和 zone_monitor 期望的结构一致。"""

    def __init__(self, cfg: Dict[str, Any], paths: Dict[str, str], zone: Any,
                 no_hands: bool = False):
        pose_backend = str(cfg["models"]["pose_backend"])
        if pose_backend == "mp":
            if not os.path.isfile(paths["pose_model"]):
                raise RuntimeError(
                    f"姿态模型不存在：{paths['pose_model']}（检查 perception.yaml "
                    f"models.pose，或重跑 bootstrap 复制模型）")
            if not no_hands and not os.path.isfile(paths["hand_model"]):
                raise RuntimeError(
                    f"手部模型不存在：{paths['hand_model']}（检查 perception.yaml "
                    f"models.hand，或重跑 bootstrap 复制模型）")

        self.zone = zone
        self.no_hands = bool(no_hands)
        self.parts = str(cfg["pose"]["body_parts"])
        self.ptrack = zm.PersonTracker()
        self.quality_gate = zm.QualityGate()
        self.grader = zm.Grader()

        try:
            self.pose = zm.make_pose(paths["pose_model"])
            self.hands = None if no_hands else zm.make_hands(paths["hand_model"])
        except Exception as exc:
            raise RuntimeError(f"加载 MediaPipe 模型失败：{exc}") from exc

    def quality(self, color, depth):
        """pipeline 的 quality_fn：注意参数顺序是 (color, depth)。"""
        return zm.frame_quality(depth, color)

    def judge(self, frame, t: float, quality):
        """pipeline 的 judge_fn：跑一次姿态/手部，再交给 judge_multi。"""
        rgb = cv2.cvtColor(frame.color, cv2.COLOR_BGR2RGB)
        image = zm.mp.Image(image_format=zm.mp.ImageFormat.SRGB,
                            data=np.ascontiguousarray(rgb))
        ts_ms = int(float(t) * 1000)
        pose_res = self.pose.detect_for_video(image, ts_ms)
        hand_res = (self.hands.detect_for_video(image, ts_ms)
                    if self.hands is not None else None)
        poses = pose_res.pose_landmarks if pose_res.pose_landmarks else []
        hands = (hand_res.hand_landmarks
                 if hand_res is not None and hand_res.hand_landmarks else [])
        return zm.judge_multi(poses, hands, frame.depth, frame.intr, self.zone,
                              self.ptrack, float(t), quality, None, self.parts)

    def close(self):
        for obj in (getattr(self, "pose", None), getattr(self, "hands", None)):
            try:
                if obj is not None:
                    obj.close()
            except Exception:
                pass


@dataclass
class Runtime:
    pipeline: ObservationPipeline
    inferencer: MediaPipeInferencer
    paths: Dict[str, str]
    calibration: CalibrationResult
    zone: ZoneResult

    def close(self):
        self.inferencer.close()


def build_runtime(cfg: Dict[str, Any], config_path: str,
                  allow_uncalibrated: bool = False, no_hands: bool = False,
                  log: Callable[[str], None] = print) -> Runtime:
    """启动期一次性组装。任何一步失败都抛异常，由入口脚本拒绝启动。"""
    paths, calib, zone_result = prepare_zone(
        cfg, config_path, allow_uncalibrated=allow_uncalibrated, log=log)
    inferencer = MediaPipeInferencer(cfg, paths, zone_result.zone, no_hands=no_hands)
    pipeline = ObservationPipeline(
        source_id=str(cfg["node"]["source_id"]),
        zone_id=str(cfg["node"]["zone_id"]),
        quality_fn=inferencer.quality,
        judge_fn=inferencer.judge,
        quality_gate=inferencer.quality_gate,
        grader=inferencer.grader,
        zone_mode=zone_result.mode,
    )
    return Runtime(pipeline=pipeline, inferencer=inferencer, paths=paths,
                   calibration=calib, zone=zone_result)
