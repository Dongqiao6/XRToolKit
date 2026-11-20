"""底盘配置：默认值、YAML 加载、ROS 参数覆盖与启动前校验。

配置是安全策略的一部分：字段缺失、类型不对、取值越界或出现未知字段都直接
拒绝启动，避免现场把笔误当成默认值跑起来。

``VR_safe/config/chassis.yaml`` 用本项目的纯 YAML 格式（不是 ROS 参数文件）：

    chassis:
      dry_run: false
      connect_ip: "192.168.71.50"
      ...
    topics:
      permit: /vr_safe/permit
      ...

节点也接受扁平 ROS 参数覆盖（例如 ``-p dry_run:=true``、``-p slow_scale:=0.2``），
显式参数优先于 YAML。
"""

import copy
import math
import os

DEFAULTS = {
    "chassis": {
        # 连接与坐标系
        "dry_run": False,
        "connect_ip": "192.168.71.50",
        "connect_username": "",
        "connect_passwd": "",
        "frame_id": "base_link",
        "parent_frame_id": "map",
        "odom_frame_id": "odom",
        "odom_use_integration": True,
        # 手柄与速度
        "max_linear_speed": 0.5,
        "max_angular_speed": 0.8,
        "joy_deadzone": 0.2,
        "slow_scale": 0.3,
        # 安全时序
        "zero_keepalive_ms": 100,
        "twist_watchdog_ms": 200,
        "permit_watchdog_ms": 300,
        "max_zero_failures": 5,
        "gate_tick_period_ms": 20,
        "speed_resend_ms": 50,
        # 按键：X 已是夹爪锁定，不能用作底盘急停；-1 = 禁用。
        "estop_button": -1,
        "estop_release_button": 1,  # Y
        # 状态发布与 SRP 状态同步
        "state_publish_rate_hz": 10.0,
        "sync_state_period_s": 0.3,
    },
    "topics": {
        "permit": "/vr_safe/permit",
        "button": "/vr_safe/teleop/button",
        "cmd_vel": "/vr_safe/chassis/cmd_vel",
        "state": "/vr_safe/chassis/state",
        "system_state": "/vr_safe/chassis/system_state",
        "battery_state": "/vr_safe/chassis/battery_state",
        "odom": "/vr_safe/chassis/odom",
        "remote_control_enabled": "/vr_safe/chassis/remote_control_enabled",
        "remote_control_oba_enabled": "/vr_safe/chassis/remote_control_oba_enabled",
        "emergency_stop": "/vr_safe/chassis/emergency_stop",
        "release_emergency_stop": "/vr_safe/chassis/release_emergency_stop",
    },
}


class ConfigError(ValueError):
    """配置非法；节点应拒绝启动而不是猜一个默认值。"""


def load_config(path: str = "") -> dict:
    """加载并校验配置；``path`` 为空时只用默认值。"""
    cfg = copy.deepcopy(DEFAULTS)
    if path:
        if not os.path.isfile(path):
            raise ConfigError(f"配置文件不存在：{path!r}")
        try:
            import yaml
        except ImportError as exc:  # pragma: no cover - 环境缺包时给出明确提示
            raise ConfigError("缺少 PyYAML；请在 VR_safe venv 里安装 pyyaml") from exc
        with open(path, encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}
        if not isinstance(raw, dict):
            raise ConfigError("配置文件顶层必须是映射")
        _reject_unknown(raw, DEFAULTS, "顶层")
        cfg = _merge(cfg, raw)
    _validate(cfg)
    return cfg


def apply_parameter_overrides(cfg: dict, overrides: dict) -> dict:
    """把显式 ROS 参数（扁平键）覆盖到配置上并重新校验。"""
    result = copy.deepcopy(cfg)
    for section in ("chassis", "topics"):
        for key in result[section]:
            if key in overrides:
                result[section][key] = overrides[key]
    _validate(result)
    return result


def _merge(base: dict, override: dict) -> dict:
    result = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if not isinstance(value, dict):
            raise ConfigError(f"配置项 {key!r} 必须是映射")
        _reject_unknown(value, base[key], key)
        result[key].update(value)
    return result


def _reject_unknown(values: dict, allowed: dict, where: str):
    unknown = [key for key in values if key not in allowed]
    if unknown:
        raise ConfigError(
            f"{where} 含未知字段 {unknown}；允许的字段为 {sorted(allowed)}"
        )


def _validate(cfg: dict):
    chassis = cfg["chassis"]
    topics = cfg["topics"]

    _bool(chassis, "dry_run")
    _bool(chassis, "odom_use_integration")

    _string(chassis, "connect_ip", allow_empty=True)
    _string(chassis, "connect_username", allow_empty=True)
    _string(chassis, "connect_passwd", allow_empty=True)
    _string(chassis, "frame_id")
    _string(chassis, "parent_frame_id")
    _string(chassis, "odom_frame_id")
    if not chassis["dry_run"] and not chassis["connect_ip"].strip():
        raise ConfigError("chassis.connect_ip 在非 dry_run 模式下不能为空")

    _positive(chassis, "max_linear_speed", allow_zero=False)
    _positive(chassis, "max_angular_speed", allow_zero=False)
    _positive(chassis, "state_publish_rate_hz", allow_zero=False)
    _positive(chassis, "sync_state_period_s", allow_zero=False)
    _positive(chassis, "gate_tick_period_ms", allow_zero=False)
    _positive(chassis, "zero_keepalive_ms", allow_zero=False)
    _positive(chassis, "twist_watchdog_ms", allow_zero=False)
    _positive(chassis, "permit_watchdog_ms", allow_zero=False)
    _positive(chassis, "speed_resend_ms", allow_zero=False)

    deadzone = chassis["joy_deadzone"]
    if not _is_number(deadzone) or not 0.0 <= float(deadzone) < 1.0:
        raise ConfigError("chassis.joy_deadzone 必须在 [0, 1) 内")
    slow = chassis["slow_scale"]
    if not _is_number(slow) or not 0.0 < float(slow) <= 1.0:
        raise ConfigError("chassis.slow_scale 必须在 (0, 1] 内")

    failures = chassis["max_zero_failures"]
    if isinstance(failures, bool) or not isinstance(failures, int) or failures < 1:
        raise ConfigError("chassis.max_zero_failures 必须是 >= 1 的整数")

    for key in ("estop_button", "estop_release_button"):
        value = chassis[key]
        if isinstance(value, bool) or not isinstance(value, int) or value < -1:
            raise ConfigError(f"chassis.{key} 必须是 >= -1 的整数（-1 = 禁用）")

    if chassis["gate_tick_period_ms"] > chassis["zero_keepalive_ms"]:
        raise ConfigError("chassis.gate_tick_period_ms 不能大于 zero_keepalive_ms")
    if chassis["gate_tick_period_ms"] > chassis["twist_watchdog_ms"]:
        raise ConfigError("chassis.gate_tick_period_ms 不能大于 twist_watchdog_ms")
    if chassis["gate_tick_period_ms"] > chassis["permit_watchdog_ms"]:
        raise ConfigError("chassis.gate_tick_period_ms 不能大于 permit_watchdog_ms")
    if chassis["speed_resend_ms"] >= chassis["twist_watchdog_ms"]:
        raise ConfigError(
            "chassis.speed_resend_ms 必须小于 twist_watchdog_ms，否则重复指令"
            "会在看门狗触发前来不及重发")

    if not isinstance(topics, dict):
        raise ConfigError("topics 必须是映射")
    for key in DEFAULTS["topics"]:
        _string(topics, key)


def _is_number(value) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float)) and \
        math.isfinite(float(value))


def _positive(section: dict, key: str, *, allow_zero: bool):
    value = section.get(key)
    if not _is_number(value):
        raise ConfigError(f"chassis.{key} 必须是有限数")
    if allow_zero:
        if value < 0:
            raise ConfigError(f"chassis.{key} 不能为负")
    elif value <= 0:
        raise ConfigError(f"chassis.{key} 必须大于 0")


def _bool(section: dict, key: str):
    if not isinstance(section.get(key), bool):
        raise ConfigError(f"chassis.{key} 必须是 true/false")


def _string(section: dict, key: str, *, allow_empty: bool = False):
    value = section.get(key)
    if not isinstance(value, str):
        raise ConfigError(f"{key} 必须是字符串")
    if not allow_empty and not value.strip():
        raise ConfigError(f"{key} 不能为空")
