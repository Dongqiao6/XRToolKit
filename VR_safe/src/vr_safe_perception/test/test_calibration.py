"""外参有效性检查测试：生产拒绝 vs 开发显式降级。"""

import json
import math
import os

import numpy as np
import pytest

from vr_safe_perception.calibration import (CalibrationError, load_extrinsic,
                                            plane_diff, validate_calibration)
from vr_safe_perception.paths import vr_safe_root


def write_zone_config(path, normal=(0.0, 0.0, -1.0), origin=(0.0, 0.0, 2.0)):
    path.write_text(json.dumps({
        "plane": {"origin": list(origin), "normal": list(normal)},
        "shape": {"size_m": [1.2, 1.2]},
    }), encoding="utf-8")


def write_extrinsic(path, ok=True, normal=(0.0, 0.0, -1.0), origin=(0.0, 0.0, 2.0),
                    T=None):
    path.write_text(json.dumps({
        "ok": ok,
        "not_ok_reasons": ["测试"] if not ok else [],
        "T_cam_base": (np.eye(4) if T is None else T).tolist(),
        "plane": {"origin": list(origin), "normal": list(normal)},
    }), encoding="utf-8")


def test_plane_diff_detects_angle_and_offset():
    plane = {"origin": [0, 0, 2], "normal": [0, 0, -1]}
    same = dict(plane)
    assert plane_diff(plane, same) == (0.0, 0.0)

    deg = 5.0
    tilted = {"origin": [0, 0, 2],
              "normal": [0, math.sin(math.radians(deg)), -math.cos(math.radians(deg))]}
    angle, dist = plane_diff(plane, tilted)
    assert angle == pytest.approx(deg, abs=1e-6)
    # 原点不变、法向转 5°，代表的是另一个平面，距离差本来就不为 0。
    assert dist > 0.0

    shifted = {"origin": [0, 0, 2.03], "normal": [0, 0, -1]}
    angle, dist = plane_diff(plane, shifted)
    assert angle == pytest.approx(0.0, abs=1e-9)
    assert dist == pytest.approx(0.03, abs=1e-9)


def test_valid_extrinsic_and_matching_ground(tmp_path):
    zone_cfg = tmp_path / "zone_config.json"
    ext = tmp_path / "extrinsic.json"
    write_zone_config(zone_cfg)
    write_extrinsic(ext)
    result = validate_calibration(
        str(ext), str(zone_cfg), "top", require_valid=True,
        max_ground_angle_deg=0.5, max_ground_offset_m=0.02)
    assert result.used_calibrated is True
    assert result.T_cam_base is not None
    np.testing.assert_allclose(result.T_cam_base, np.eye(4))
    assert result.ground_angle_deg == pytest.approx(0.0, abs=1e-9)


def test_ok_false_rejected_in_production(tmp_path):
    zone_cfg = tmp_path / "zone_config.json"
    ext = tmp_path / "extrinsic.json"
    write_zone_config(zone_cfg)
    write_extrinsic(ext, ok=False)
    with pytest.raises(CalibrationError, match="ok != true"):
        validate_calibration(str(ext), str(zone_cfg), "top", require_valid=True,
                             max_ground_angle_deg=0.5, max_ground_offset_m=0.02)


def test_ground_mismatch_rejected_in_production(tmp_path):
    zone_cfg = tmp_path / "zone_config.json"
    ext = tmp_path / "extrinsic.json"
    write_zone_config(zone_cfg)
    write_extrinsic(ext, normal=(0.0, 0.1, -0.995))
    with pytest.raises(CalibrationError, match="地面和当前地面不一致"):
        validate_calibration(str(ext), str(zone_cfg), "top", require_valid=True,
                             max_ground_angle_deg=0.5, max_ground_offset_m=0.02)


def test_missing_extrinsic_rejected_in_production(tmp_path):
    zone_cfg = tmp_path / "zone_config.json"
    write_zone_config(zone_cfg)
    with pytest.raises(CalibrationError, match="读不了"):
        validate_calibration(str(tmp_path / "nope.json"), str(zone_cfg), "top",
                             require_valid=True, max_ground_angle_deg=0.5,
                             max_ground_offset_m=0.02)


def test_allow_uncalibrated_falls_back_to_design(tmp_path):
    zone_cfg = tmp_path / "zone_config.json"
    ext = tmp_path / "extrinsic.json"
    write_zone_config(zone_cfg)
    write_extrinsic(ext, ok=False)
    result = validate_calibration(
        str(ext), str(zone_cfg), "top", require_valid=True,
        max_ground_angle_deg=0.5, max_ground_offset_m=0.02,
        allow_uncalibrated=True)
    assert result.used_calibrated is False
    assert result.warnings
    # 包内带了 jaka_s5_sim，设计值 + 地面摆正应该能算出来。
    assert result.T_cam_base is not None
    assert result.T_cam_base.shape == (4, 4)
    assert np.all(np.isfinite(result.T_cam_base))


def test_load_extrinsic_checks_transform(tmp_path):
    bad = tmp_path / "bad.json"
    write_extrinsic(bad, T=np.zeros((4, 4)))
    with pytest.raises(CalibrationError):
        load_extrinsic(str(bad))


def test_vr_safe_root_has_config():
    root = vr_safe_root()
    assert root is not None
    assert os.path.isfile(str(root / "config" / "perception.yaml"))
