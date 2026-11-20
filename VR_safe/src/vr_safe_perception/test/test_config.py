"""perception.yaml 的严格校验测试：缺字段、多字段、坏值都必须拒绝。"""

import copy
import os

import pytest
import yaml

from vr_safe_perception.config import (ConfigError, load_config, resolve_path,
                                       validate_config)
from vr_safe_perception.paths import vr_safe_root


def real_config_path():
    root = vr_safe_root()
    assert root is not None, "测试必须能找到 VR_safe 根目录"
    return os.path.join(str(root), "config", "perception.yaml")


def real_raw():
    with open(real_config_path(), encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_real_config_is_valid():
    cfg = load_config(real_config_path())
    assert cfg["node"]["source_id"] == "camera_top"
    assert cfg["calibration"]["zone_mode"] == "polygon"
    assert cfg["calibration"]["require_valid"] is True


@pytest.mark.parametrize("section,key", [
    ("node", "topic"),
    ("camera", "backend"),
    ("models", "pose"),
    ("calibration", "extrinsic_file"),
    ("reach", "samples"),
    ("zone", "height_m"),
    ("quality", "bad_s"),
    ("pose", "body_parts"),
    ("grader", "relax_n"),
])
def test_missing_field_rejected(section, key):
    raw = real_raw()
    del raw[section][key]
    with pytest.raises(ConfigError, match="缺少字段"):
        validate_config(raw)


def test_unknown_field_rejected():
    raw = real_raw()
    raw["camera"]["fov"] = 60
    with pytest.raises(ConfigError, match="未知字段"):
        validate_config(raw)


def test_unknown_section_rejected():
    raw = real_raw()
    raw["extra"] = {}
    with pytest.raises(ConfigError, match="未知字段"):
        validate_config(raw)


@pytest.mark.parametrize("section,key,value", [
    ("node", "publish_rate_hz", 0),
    ("node", "unknown_rate_hz", -1),
    ("node", "camera_max_consecutive_errors", 0),
    ("camera", "backend", "kinect"),
    ("camera", "width", 32),
    ("camera", "latest_only", "yes"),
    ("models", "pose_backend", "openvino"),
    ("calibration", "zone_mode", "grid"),
    ("calibration", "require_valid", 1),
    ("reach", "margin_m", 5),
    ("zone", "height_m", 0),
    ("quality", "min_depth_valid", 2),
    ("pose", "body_parts", "head"),
    ("pose", "pose_conf", 1.5),
    ("grader", "relax_n", 0),
])
def test_bad_values_rejected(section, key, value):
    raw = real_raw()
    raw[section][key] = value
    with pytest.raises(ConfigError):
        validate_config(raw)


def test_bool_is_not_accepted_as_int():
    raw = real_raw()
    raw["camera"]["width"] = True
    with pytest.raises(ConfigError):
        validate_config(raw)


def test_topic_must_be_absolute():
    raw = real_raw()
    raw["node"]["topic"] = "vr_safe/observation"
    with pytest.raises(ConfigError, match="以 / 开头"):
        validate_config(raw)


def test_relative_paths_resolve_against_vr_safe_root():
    cfg_path = "/tmp/vr_safe_test/config/perception.yaml"
    assert resolve_path(cfg_path, "models/pose.task") == \
        "/tmp/vr_safe_test/models/pose.task"
    assert resolve_path(cfg_path, "/abs/model.task") == "/abs/model.task"


def test_validate_does_not_mutate_input():
    raw = real_raw()
    before = copy.deepcopy(raw)
    validate_config(raw)
    assert raw == before


def test_resolved_paths_point_into_vr_safe():
    from vr_safe_perception.config import resolved_paths
    cfg = load_config(real_config_path())
    paths = resolved_paths(cfg, real_config_path())
    root = str(vr_safe_root())
    assert paths["pose_model"] == os.path.join(root, "models",
                                               "pose_landmarker_full.task")
    assert paths["hand_model"] == os.path.join(root, "models",
                                               "hand_landmarker.task")
    assert paths["zone_file"] == os.path.join(root, "config", "calibration",
                                              "zone_ground.json")
