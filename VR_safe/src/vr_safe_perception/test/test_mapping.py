"""五档 → 四值映射、evidence/置信度、detail_json 的纯逻辑测试。"""

import json

import numpy as np
import pytest

from vr_safe_perception import mapping
from vr_safe_perception.mapping import (BESIDE, CLEAR, HAND_IN, IDLE, NEAR,
                                        OCCUPIED, OCCUPIED_SAFE, UNCLEAR,
                                        UNKNOWN, build_observation,
                                        evidence_for, jsonable,
                                        map_five_to_four)


@pytest.mark.parametrize("five,four", [
    (IDLE, CLEAR),
    (BESIDE, NEAR),
    (UNCLEAR, UNKNOWN),
    (HAND_IN, OCCUPIED_SAFE),
    (OCCUPIED, OCCUPIED_SAFE),
])
def test_five_to_four_mapping(five, four):
    assert map_five_to_four(five) == four


def test_unknown_five_state_is_rejected_not_guessed_as_clear():
    with pytest.raises(ValueError):
        map_five_to_four("有人吗")


def test_evidence_codes():
    assert evidence_for(IDLE) == mapping.EV_NONE
    assert evidence_for(BESIDE) == mapping.EV_NONE
    assert evidence_for(UNCLEAR) == mapping.EV_LOST
    assert evidence_for(HAND_IN, "伸进去了（肢体 1 点、手 0 点）") == mapping.EV_LIMB
    assert evidence_for(HAND_IN, "头探进区域了") == mapping.EV_HEAD
    assert evidence_for(OCCUPIED, "人在区域里（躯干 4 个点）") == mapping.EV_TORSO
    assert evidence_for(OCCUPIED, "人在区域里", {"torso_pts": 0}) == mapping.EV_LIMB


def test_observation_fields_and_detail_json():
    info = {"n_persons": 1, "torso_pts": 4, "inferred": {1, 2},
            "pts3d": {11: np.array([0.1, 0.2, 1.0])}, "bad": float("nan")}
    obs = build_observation("camera_top", "default", OCCUPIED,
                            "人在区域里（躯干 4 个点）", info)
    assert obs.source_id == "camera_top"
    assert obs.zone_id == "default"
    assert obs.state == OCCUPIED_SAFE
    assert obs.evidence == mapping.EV_TORSO
    assert 0.0 <= obs.confidence <= 1.0
    detail = json.loads(obs.detail_json)
    assert detail["five_state"] == OCCUPIED
    assert detail["safe_state"] == OCCUPIED_SAFE
    assert detail["torso_pts"] == 4
    assert detail["inferred"] == [1, 2]
    assert detail["bad"] is None          # NaN 不能把整条观察弄坏
    assert detail["pts3d"]["11"] == [0.1, 0.2, 1.0]


def test_unknown_confidence_is_low():
    obs = build_observation("camera_top", "default", UNCLEAR, "深度全空", {})
    assert obs.confidence < 0.5


def test_jsonable_handles_numpy_and_nested():
    value = {"a": np.float32(0.5), "b": np.array([1, 2]),
             "c": {3, 1, 2}, "d": (1, float("inf"))}
    out = jsonable(value)
    assert out == {"a": 0.5, "b": [1, 2], "c": [1, 2, 3], "d": [1, None]}
