"""配置校验：现场救急参数必须能被加载，非法去抖帧数必须拒绝。"""

from pathlib import Path

import pytest

from vr_safe_supervisor.config import ConfigError, load_config

ROOT = Path(__file__).resolve().parents[3]


def test_site_config_has_extended_timeout_and_debounce():
    cfg = load_config(str(ROOT / "config" / "vr_safe.yaml"))
    assert cfg["safety"]["observation_timeout_s"] == 0.8
    assert cfg["safety"]["occupied_debounce_frames"] == 2


def test_invalid_debounce_is_rejected(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("safety:\n  occupied_debounce_frames: 0\n")
    with pytest.raises(ConfigError):
        load_config(str(p))
