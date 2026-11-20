"""perception.yaml 的加载与严格校验。

配置属于安全策略的一部分：字段缺失、类型不对、取值越界、或者出现拼错的
未知字段，都直接拒绝启动，不做静默兜底。这样现场改错一个数不会变成
「看起来启动了，实际上按默认值在跑」。
"""

import os
from typing import Any, Dict

import yaml


class ConfigError(ValueError):
    """配置不合法。入口脚本捕获后打印并以非零码退出。"""


_BACKENDS = ("orbbec", "realsense", "auto")
_POSE_BACKENDS = ("mp", "gpu")
_ZONE_MODES = ("polygon", "reach")
_BODY_PARTS = ("upper", "full")

# 每一节必须恰好包含这些字段；多一个（多半是拼错）少一个都报错。
_SCHEMA = {
    "node": {"source_id", "zone_id", "topic", "publish_rate_hz",
             "unknown_rate_hz", "camera_retry_s", "camera_max_consecutive_errors"},
    "camera": {"backend", "width", "height", "fps", "latest_only"},
    "models": {"pose", "hand", "pose_backend", "max_persons"},
    "calibration": {"zone_mode", "zone_file", "zone_config", "extrinsic_file",
                    "camera_id", "require_valid", "max_ground_angle_deg",
                    "max_ground_offset_m"},
    "reach": {"margin_m", "samples"},
    "zone": {"height_m", "near_m"},
    "quality": {"min_depth_valid", "max_near_cover", "min_texture", "bad_s", "good_s"},
    "pose": {"body_parts", "min_good_lms_upper", "pose_conf"},
    "grader": {"relax_n"},
}


def _mapping(value: Any, path: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigError(f"{path} 必须是映射")
    return value


def _check_keys(value: Dict[str, Any], path: str, required) -> None:
    keys = set(value)
    missing = sorted(required - keys)
    unknown = sorted(keys - required)
    if missing:
        raise ConfigError(f"{path} 缺少字段：{', '.join(missing)}")
    if unknown:
        raise ConfigError(f"{path} 有未知字段：{', '.join(unknown)}")


def _num(value: Any, path: str, lo=None, hi=None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{path} 必须是数字")
    value = float(value)
    if value != value or value in (float("inf"), float("-inf")):
        raise ConfigError(f"{path} 必须是有限数字")
    if lo is not None and value < lo:
        raise ConfigError(f"{path} 不能小于 {lo}")
    if hi is not None and value > hi:
        raise ConfigError(f"{path} 不能大于 {hi}")
    return value


def _int(value: Any, path: str, lo=None, hi=None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f"{path} 必须是整数")
    if lo is not None and value < lo:
        raise ConfigError(f"{path} 不能小于 {lo}")
    if hi is not None and value > hi:
        raise ConfigError(f"{path} 不能大于 {hi}")
    return value


def _bool(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigError(f"{path} 必须是 true/false")
    return value


def _str(value: Any, path: str, allow_empty=False) -> str:
    if not isinstance(value, str) or (not allow_empty and not value.strip()):
        raise ConfigError(f"{path} 必须是非空字符串")
    return value


def _enum(value: Any, path: str, choices) -> str:
    value = _str(value, path)
    if value not in choices:
        raise ConfigError(f"{path} 必须是 {'/'.join(choices)} 之一，实际 {value!r}")
    return value


def validate_config(raw: Any) -> Dict[str, Any]:
    """校验已经读进来的字典，返回原字典（不补默认值）。"""
    cfg = _mapping(raw, "配置顶层")
    _check_keys(cfg, "配置顶层", set(_SCHEMA))

    for section, fields in _SCHEMA.items():
        _mapping(cfg[section], section)
        _check_keys(cfg[section], section, fields)

    node = cfg["node"]
    _str(node["source_id"], "node.source_id")
    _str(node["zone_id"], "node.zone_id")
    topic = _str(node["topic"], "node.topic")
    if not topic.startswith("/"):
        raise ConfigError("node.topic 必须以 / 开头")
    _num(node["publish_rate_hz"], "node.publish_rate_hz", 0.1, 120.0)
    _num(node["unknown_rate_hz"], "node.unknown_rate_hz", 0.1, 60.0)
    _num(node["camera_retry_s"], "node.camera_retry_s", 0.1, 60.0)
    _int(node["camera_max_consecutive_errors"],
         "node.camera_max_consecutive_errors", 1, 1000)

    cam = cfg["camera"]
    _enum(cam["backend"], "camera.backend", _BACKENDS)
    _int(cam["width"], "camera.width", 160, 4096)
    _int(cam["height"], "camera.height", 120, 4096)
    _int(cam["fps"], "camera.fps", 1, 120)
    _bool(cam["latest_only"], "camera.latest_only")

    models = cfg["models"]
    _str(models["pose"], "models.pose")
    _str(models["hand"], "models.hand")
    _enum(models["pose_backend"], "models.pose_backend", _POSE_BACKENDS)
    _int(models["max_persons"], "models.max_persons", 1, 10)

    cal = cfg["calibration"]
    _enum(cal["zone_mode"], "calibration.zone_mode", _ZONE_MODES)
    _str(cal["zone_file"], "calibration.zone_file")
    _str(cal["zone_config"], "calibration.zone_config")
    _str(cal["extrinsic_file"], "calibration.extrinsic_file")
    _str(cal["camera_id"], "calibration.camera_id")
    _bool(cal["require_valid"], "calibration.require_valid")
    _num(cal["max_ground_angle_deg"], "calibration.max_ground_angle_deg", 0.0, 90.0)
    _num(cal["max_ground_offset_m"], "calibration.max_ground_offset_m", 0.0, 1.0)

    reach = cfg["reach"]
    _num(reach["margin_m"], "reach.margin_m", 0.0, 2.0)
    _int(reach["samples"], "reach.samples", 0, 100000)

    zone = cfg["zone"]
    _num(zone["height_m"], "zone.height_m", 0.1, 10.0)
    _num(zone["near_m"], "zone.near_m", 0.0, 10.0)

    quality = cfg["quality"]
    _num(quality["min_depth_valid"], "quality.min_depth_valid", 0.0, 1.0)
    _num(quality["max_near_cover"], "quality.max_near_cover", 0.0, 1.0)
    _num(quality["min_texture"], "quality.min_texture", 0.0, 10000.0)
    _num(quality["bad_s"], "quality.bad_s", 0.0, 60.0)
    _num(quality["good_s"], "quality.good_s", 0.0, 60.0)

    pose = cfg["pose"]
    _enum(pose["body_parts"], "pose.body_parts", _BODY_PARTS)
    _int(pose["min_good_lms_upper"], "pose.min_good_lms_upper", 1, 25)
    _num(pose["pose_conf"], "pose.pose_conf", 0.0, 1.0)

    _int(cfg["grader"]["relax_n"], "grader.relax_n", 1, 1000)
    return cfg


def load_config(path: str) -> Dict[str, Any]:
    """读 YAML 并校验。任何问题都抛 ConfigError。"""
    if not path or not os.path.isfile(path):
        raise ConfigError(f"配置文件不存在：{path!r}")
    try:
        with open(path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
    except yaml.YAMLError as exc:
        raise ConfigError(f"配置文件不是合法 YAML：{exc}") from exc
    return validate_config(raw)


def config_base_dir(config_path: str) -> str:
    """配置里相对路径的基准目录。

    perception.yaml 按约定放在 VR_safe/config/ 下，里面的路径都相对 VR_safe/
    写（例如 models/pose_landmarker_full.task）。所以基准是配置文件目录的
    上一级。配置文件放在别处时也按这个约定，不猜。
    """
    return os.path.dirname(os.path.dirname(os.path.abspath(config_path)))


def resolve_path(config_path: str, value: str) -> str:
    """把配置里的路径按 VR_safe/ 基准解析成绝对路径。"""
    value = os.path.expanduser(value)
    if os.path.isabs(value):
        return os.path.normpath(value)
    return os.path.normpath(os.path.join(config_base_dir(config_path), value))


def resolved_paths(cfg: Dict[str, Any], config_path: str) -> Dict[str, str]:
    """一次性解析出运行期要用的所有路径。"""
    return {
        "pose_model": resolve_path(config_path, cfg["models"]["pose"]),
        "hand_model": resolve_path(config_path, cfg["models"]["hand"]),
        "zone_file": resolve_path(config_path, cfg["calibration"]["zone_file"]),
        "zone_config": resolve_path(config_path, cfg["calibration"]["zone_config"]),
        "extrinsic_file": resolve_path(
            config_path, cfg["calibration"]["extrinsic_file"]),
    }
