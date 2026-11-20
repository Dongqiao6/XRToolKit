"""dry-run 合成输入与 teleop.yaml 的单测。"""

import json
from pathlib import Path

import pytest
import yaml

from vr_safe_teleop.quest_source import (
    DryRunQuestSource,
    create_source,
)
from vr_safe_teleop.teleop_core import TeleopCore

CONFIG_PATH = (
    Path(__file__).resolve().parents[3] / "config" / "teleop.yaml"
)
POSE = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]


class FakeClock:
    def __init__(self, now=0.0):
        self.now = now

    def __call__(self):
        return self.now


def test_neutral_dry_run_source_is_untracked():
    source = DryRunQuestSource(clock=FakeClock())
    raw = source.read()
    assert raw == {"left": {}, "right": {}}
    # 中性输入仍能生成 Joy/心跳；目标不发布由 core 的未跟踪判断保证。
    core = TeleopCore()
    frame = core.update(raw["left"], raw["right"])
    assert frame.targets == {"left": None, "right": None}
    assert frame.joy_changed is True
    assert frame.estop_pressed is False


def test_dry_run_frames_merge_forward_and_drive_estop():
    clock = FakeClock()
    source = DryRunQuestSource(
        frames=[
            {"t": 0.0, "left": {"pose": POSE, "grip": 0.5}},
            {"t": 1.0, "left": {"axis_click": True}},
            {"t": 1.5, "left": {"axis_click": False, "Y": True}},
        ],
        loop=False,
        clock=clock,
    )
    core = TeleopCore()

    clock.now = 0.5
    raw = source.read()
    assert raw["left"]["pose"] == POSE
    assert raw["left"]["grip"] == 0.5
    assert raw["left"].get("axis_click", False) is False
    assert core.update(raw["left"], raw["right"]).estop_pressed is False

    clock.now = 1.2
    raw = source.read()
    # 未写的键沿用上一帧，按钮由后面的帧显式清除。
    assert raw["left"]["pose"] == POSE
    assert raw["left"]["axis_click"] is True
    frame = core.update(raw["left"], raw["right"])
    assert frame.estop_pressed is True
    assert "LJ" in frame.estop_reason

    clock.now = 1.6
    raw = source.read()
    assert raw["left"]["axis_click"] is False
    assert raw["left"]["Y"] is True
    frame = core.update(raw["left"], raw["right"])
    assert frame.clear_estop_pressed is True


def test_dry_run_loop_wraps_to_neutral_before_first_frame():
    clock = FakeClock()
    # 最后一帧 t=1.0，循环周期 = 1.0 + LOOP_HOLD_S(1.0) = 2.0 s。
    source = DryRunQuestSource(
        frames=[{"t": 1.0, "left": {"pose": POSE}}],
        loop=True,
        clock=clock,
    )
    clock.now = 0.5
    assert source.read()["left"] == {}
    clock.now = 2.2
    assert source.read()["left"] == {}
    clock.now = 3.2
    assert source.read()["left"]["pose"] == POSE


def test_dry_run_source_from_json_file(tmp_path):
    path = tmp_path / "frames.json"
    path.write_text(json.dumps({
        "frames": [
            {"t": 0.0, "left": {"pose": POSE}, "right": {}},
            {"t": 0.5, "left": {"axis_click": True}},
        ]
    }), encoding="utf-8")
    clock = FakeClock(1.0)
    source = DryRunQuestSource.from_file(str(path), loop=False, clock=clock)
    clock.now = 1.6
    raw = source.read()
    assert raw["left"]["pose"] == POSE
    assert raw["left"]["axis_click"] is True


def test_dry_run_source_rejects_missing_file():
    with pytest.raises(ValueError, match="不存在"):
        DryRunQuestSource.from_file("/no/such/teleop_frames.json")


def test_dry_run_source_rejects_bad_frames(tmp_path):
    with pytest.raises(ValueError, match="非负"):
        DryRunQuestSource(frames=[{"t": -1.0}], clock=FakeClock())
    with pytest.raises(ValueError, match="非降序"):
        DryRunQuestSource(
            frames=[{"t": 1.0}, {"t": 0.5}], clock=FakeClock()
        )
    path = tmp_path / "bad.json"
    path.write_text("{not json", encoding="utf-8")
    with pytest.raises(ValueError, match="JSON 解析失败"):
        DryRunQuestSource.from_file(str(path))


def test_create_source_dry_run_never_needs_sdk():
    source = create_source(dry_run=True, dry_run_input="", dry_run_loop=True)
    assert isinstance(source, DryRunQuestSource)
    assert source.read() == {"left": {}, "right": {}}


def test_teleop_config_contract():
    if not CONFIG_PATH.is_file():
        pytest.skip("不在源码树中运行，跳过 VR_safe/config/teleop.yaml 检查")
    raw = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    params = raw["vr_safe_teleop_bridge"]["ros__parameters"]

    # 话题必须全部在 /vr_safe/teleop/* 和 /vr_safe/observation 下。
    assert params["left_target_topic"] == "/vr_safe/teleop/left_target"
    assert params["right_target_topic"] == "/vr_safe/teleop/right_target"
    assert params["button_topic"] == "/vr_safe/teleop/button"
    assert params["heartbeat_topic"] == "/vr_safe/teleop/heartbeat"
    assert params["observation_topic"] == "/vr_safe/observation"
    assert params["clear_estop_service"] == "/vr_safe/clear_estop"
    assert "/rc_ctrl" not in json.dumps(params)

    assert params["heartbeat_rate_hz"] == pytest.approx(20.0)
    assert params["heartbeat_source_id"] == "teleop"
    assert params["publish_rate"] == pytest.approx(125.0)
    assert params["smooth_alpha"] == pytest.approx(0.4)
    assert params["grip_threshold"] == pytest.approx(0.5)
    assert params["pose_scale"] == pytest.approx(1.0)

    assert params["estop_button"] == "LJ"
    assert params["clear_estop_button"] == "Y"
    assert params["dual_arm_button"] == "B"
    assert params["gripper_left_button"] == "X"
    assert params["gripper_right_button"] == "A"

    assert params["dry_run"] is False
    assert params["dry_run_input"] == ""
    assert len(params["headset_to_robot"]) == 9
    # Quest MAC 只用于诊断，但配置里必须能追溯是哪台设备。
    assert params["quest_mac"] == "50:99:03:0a:38:78"
