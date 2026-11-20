"""配置加载与校验。配置是安全策略的一部分，缺字段/类型不对直接拒绝启动。"""

import hashlib
import json
import os
from typing import Any, Dict

DEFAULTS: Dict[str, Any] = {
    "safety": {
        "observation_timeout_s": 0.8,
        "heartbeat_timeout_s": 0.3,
        "occupied_debounce_frames": 2,
        "require_teleop_heartbeat": True,
        "required_sources": ["camera_top"],
        "teleop_source": "teleop",
        "publish_rate_hz": 10.0,
        "audit_path": "log/safety.jsonl",
    },
    "topics": {
        "observation": "/vr_safe/observation",
        "heartbeat": "/vr_safe/teleop/heartbeat",
        "permit": "/vr_safe/permit",
        "confirm_resume": "/vr_safe/confirm_resume",
        "clear_estop": "/vr_safe/clear_estop",
    },
}


class ConfigError(ValueError):
    pass


def _merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    out = json.loads(json.dumps(base))
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path: str) -> Dict[str, Any]:
    if not path or not os.path.isfile(path):
        raise ConfigError(f"配置文件不存在：{path!r}")
    try:
        import yaml
    except ImportError as exc:
        raise ConfigError("缺少 PyYAML；请在 bootstrap 里安装 pyyaml") from exc
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        raise ConfigError("配置文件顶层必须是映射")
    cfg = _merge(DEFAULTS, raw)

    safety = cfg["safety"]
    for key in ("observation_timeout_s", "heartbeat_timeout_s"):
        val = safety.get(key)
        if not isinstance(val, (int, float)) or val <= 0:
            raise ConfigError(f"safety.{key} 必须是正数")
    debounce = safety.get("occupied_debounce_frames")
    if not isinstance(debounce, int) or isinstance(debounce, bool) or debounce < 1:
        raise ConfigError("safety.occupied_debounce_frames 必须是 >=1 的整数")
    if not isinstance(safety.get("required_sources"), list) or \
            not all(isinstance(x, str) and x for x in safety["required_sources"]):
        raise ConfigError("safety.required_sources 必须是非空字符串列表")
    if not isinstance(safety.get("publish_rate_hz"), (int, float)) or \
            safety["publish_rate_hz"] <= 0:
        raise ConfigError("safety.publish_rate_hz 必须是正数")
    for key in ("observation", "heartbeat", "permit", "confirm_resume", "clear_estop"):
        if not isinstance(cfg["topics"].get(key), str) or not cfg["topics"][key]:
            raise ConfigError(f"topics.{key} 不能为空")
    return cfg


def config_hash(cfg: Dict[str, Any]) -> str:
    blob = json.dumps(cfg, sort_keys=True, ensure_ascii=False).encode()
    return hashlib.sha256(blob).hexdigest()[:16]
