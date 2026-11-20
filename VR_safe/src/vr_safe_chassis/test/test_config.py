"""chassis.yaml / 参数覆盖的加载与校验测试。"""

from pathlib import Path

import pytest

from vr_safe_chassis.config import (ConfigError, apply_parameter_overrides,
                                    load_config)


def _shipped_yaml() -> Path:
    # test → vr_safe_chassis → src → VR_safe
    return Path(__file__).resolve().parents[3] / "config" / "chassis.yaml"


def test_defaults_are_valid():
    cfg = load_config("")
    assert cfg["chassis"]["dry_run"] is False
    assert cfg["chassis"]["zero_keepalive_ms"] == 100
    assert cfg["chassis"]["twist_watchdog_ms"] == 200
    assert cfg["chassis"]["slow_scale"] == 0.3
    assert cfg["chassis"]["estop_button"] == -1
    assert cfg["topics"]["state"] == "/vr_safe/chassis/state"


def test_shipped_yaml_matches_ticket_values():
    path = _shipped_yaml()
    if not path.is_file():  # 安装后的 package 里没有仓库 config
        pytest.skip("仓库内 chassis.yaml 不存在")
    cfg = load_config(str(path))
    chassis = cfg["chassis"]
    assert chassis["dry_run"] is False
    assert chassis["connect_ip"] == "192.168.71.50"
    assert chassis["max_linear_speed"] == 0.5
    assert chassis["max_angular_speed"] == 0.8
    assert chassis["joy_deadzone"] == 0.2
    assert chassis["slow_scale"] == 0.3
    assert chassis["zero_keepalive_ms"] == 100
    assert chassis["twist_watchdog_ms"] == 200
    assert chassis["max_zero_failures"] == 5
    assert chassis["estop_button"] == -1
    assert cfg["topics"]["permit"] == "/vr_safe/permit"
    assert cfg["topics"]["button"] == "/vr_safe/teleop/button"


def test_parameter_overrides_win(tmp_path):
    path = tmp_path / "chassis.yaml"
    path.write_text(
        "chassis:\n  dry_run: false\n  slow_scale: 0.3\n",
        encoding="utf-8",
    )
    cfg = load_config(str(path))
    cfg = apply_parameter_overrides(
        cfg, {"dry_run": True, "slow_scale": 0.2, "connect_ip": "10.0.0.5"})
    assert cfg["chassis"]["dry_run"] is True
    assert cfg["chassis"]["slow_scale"] == 0.2
    assert cfg["chassis"]["connect_ip"] == "10.0.0.5"


def test_unknown_field_is_rejected(tmp_path):
    path = tmp_path / "chassis.yaml"
    path.write_text("chassis:\n  dry_run: true\n  slow_scal: 0.3\n",
                    encoding="utf-8")
    with pytest.raises(ConfigError, match="未知字段"):
        load_config(str(path))


def test_out_of_range_values_are_rejected(tmp_path):
    path = tmp_path / "chassis.yaml"
    path.write_text("chassis:\n  slow_scale: 1.5\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="slow_scale"):
        load_config(str(path))

    path.write_text(
        "chassis:\n  dry_run: true\n  zero_keepalive_ms: 50\n"
        "  gate_tick_period_ms: 100\n",
        encoding="utf-8",
    )
    with pytest.raises(ConfigError, match="gate_tick_period_ms"):
        load_config(str(path))


def test_missing_ip_requires_dry_run(tmp_path):
    path = tmp_path / "chassis.yaml"
    path.write_text(
        'chassis:\n  dry_run: false\n  connect_ip: ""\n', encoding="utf-8")
    with pytest.raises(ConfigError, match="connect_ip"):
        load_config(str(path))

    path.write_text(
        'chassis:\n  dry_run: true\n  connect_ip: ""\n', encoding="utf-8")
    cfg = load_config(str(path))
    assert cfg["chassis"]["dry_run"] is True
