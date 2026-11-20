"""录像回放 → SafetyObservation 的构造测试（不需要真相机/MediaPipe）。

用 clip_io 写一小段真实格式的彩色+深度片段，再用假的 quality/judge/grader
跑 ObservationPipeline。覆盖：
  - 回放帧源按原始顺序/时间戳产出
  - 五档 → 四值在回放路径上生效
  - detail_json 保留中间量
  - 人突然消失的 UNKNOWN 兜底
  - 循环回放的时间戳单调递增（MediaPipe 要求）
"""

import json
import itertools
from types import SimpleNamespace

import numpy as np
import pytest

from vr_safe_perception.camera_backend import Intrinsics
from vr_safe_perception.clip_io import ClipWriter
from vr_safe_perception.mapping import (BESIDE, CLEAR, HAND_IN, IDLE, NEAR,
                                        OCCUPIED, OCCUPIED_SAFE, UNCLEAR,
                                        UNKNOWN)
from vr_safe_perception.pipeline import ObservationPipeline
from vr_safe_perception.sources import ClipSource


class FakeQualityGate:
    def __init__(self, blind=False):
        self.blind = blind

    def feed(self, ok, t):
        return self.blind, 0.0


class FakeGrader:
    """测试只关心映射，不做迟滞：原样返回。"""

    def __init__(self):
        self.state = IDLE

    def feed(self, raw, t):
        self.state = raw
        return raw


def make_clip(path, n=5):
    intr = Intrinsics(fx=6.0, fy=6.0, ppx=3.5, ppy=3.5, width=8, height=8)
    writer = ClipWriter(str(path), intr)
    for i in range(n):
        color = np.full((8, 8, 3), 40 + i * 20, dtype=np.uint8)
        depth = np.full((8, 8), 1.0 + i * 0.1, dtype=np.float32)
        writer.write(color, depth, i * 0.1)
    writer.close()
    return intr


def run_pipeline(states, tmp_path, infos=None):
    make_clip(tmp_path / "clip")
    source = ClipSource(str(tmp_path / "clip"), realtime=False)
    counter = itertools.count()

    def judge(frame, t, quality):
        i = next(counter)
        state = states[i]
        info = dict(infos[i]) if infos else {}
        info.setdefault("n_persons", 1 if state in (BESIDE, HAND_IN, OCCUPIED) else 0)
        info["quality_arg"] = list(quality)
        return state, f"理由：{state}", info

    pipe = ObservationPipeline(
        source_id="camera_top", zone_id="default",
        quality_fn=lambda color, depth: (True, ""),
        judge_fn=judge,
        quality_gate=FakeQualityGate(),
        grader=FakeGrader(),
    )
    observations = []
    for frame in source.frames():
        observations.append(pipe.step(frame, frame.t))
        if len(observations) == len(states):
            break
    return observations


def test_replay_frames_have_original_order_and_timestamps(tmp_path):
    make_clip(tmp_path / "clip", n=3)
    source = ClipSource(str(tmp_path / "clip"), realtime=False)
    frames = list(source.frames())
    assert [round(f.t, 3) for f in frames] == [0.0, 0.1, 0.2]
    assert frames[0].color.shape == (8, 8, 3)
    assert frames[0].depth.shape == (8, 8)
    assert frames[0].depth.dtype == np.float32


def test_replay_path_maps_all_five_states(tmp_path):
    states = [IDLE, BESIDE, UNCLEAR, HAND_IN, OCCUPIED]
    obs = run_pipeline(states, tmp_path)
    assert [o.state for o in obs] == [CLEAR, NEAR, UNKNOWN,
                                      OCCUPIED_SAFE, OCCUPIED_SAFE]
    assert [o.five_state for o in obs] == states
    details = [json.loads(o.detail_json) for o in obs]
    assert details[0]["t"] == 0.0
    assert details[2]["quality_arg"][0] is False
    assert details[4]["safe_state"] == OCCUPIED_SAFE
    assert details[4]["reason"] == "理由：占用"


def test_vanish_hold_turns_sudden_idle_into_unknown(tmp_path):
    make_clip(tmp_path / "clip", n=2)
    source = ClipSource(str(tmp_path / "clip"), realtime=False)
    states = [OCCUPIED, IDLE]
    infos = [{"n_persons": 1}, {"n_persons": 0}]
    calls = itertools.count()

    def judge(frame, t, quality):
        i = next(calls)
        return states[i], "单帧", infos[i]

    pipe = ObservationPipeline(
        source_id="camera_top", zone_id="default",
        quality_fn=lambda color, depth: (True, ""),
        judge_fn=judge,
        quality_gate=FakeQualityGate(),
        grader=FakeGrader(),
        vanish_hold_s=1.5,
    )
    obs = [pipe.step(frame, frame.t) for frame in source.frames()]
    assert obs[0].state == OCCUPIED_SAFE
    # 0.1 s 前刚看到人，这一帧突然空了：不能直接放行成 CLEAR。
    assert obs[1].state == UNKNOWN
    assert "突然检不到" in obs[1].reason


def test_loop_replay_timestamps_are_monotonic(tmp_path):
    make_clip(tmp_path / "clip", n=2)
    source = ClipSource(str(tmp_path / "clip"), loop=True, realtime=False,
                        max_frames=5)
    ts = [f.t for f in source.frames()]
    assert len(ts) == 5
    assert all(b > a for a, b in zip(ts, ts[1:]))


def test_unknown_five_state_rejected_in_pipeline(tmp_path):
    make_clip(tmp_path / "clip", n=1)
    source = ClipSource(str(tmp_path / "clip"), realtime=False)

    pipe = ObservationPipeline(
        source_id="camera_top", zone_id="default",
        quality_fn=lambda color, depth: (True, ""),
        judge_fn=lambda frame, t, quality: ("乱码", "", {}),
        quality_gate=FakeQualityGate(),
        grader=FakeGrader(),
    )
    with pytest.raises(ValueError):
        pipe.step(next(iter(source.frames())), 0.0)


def test_startup_blind_frame_never_reports_clear(tmp_path):
    make_clip(tmp_path / "clip", n=1)
    frame = next(iter(ClipSource(str(tmp_path / "clip"), realtime=False).frames()))
    pipe = ObservationPipeline(
        source_id="camera_top", zone_id="default",
        quality_fn=lambda color, depth: (False, "深度图全空"),
        judge_fn=lambda frame, t, quality: (IDLE, "画面里没人", {"n_persons": 0}),
        quality_gate=FakeQualityGate(blind=False),
        grader=FakeGrader(),
    )
    obs = pipe.step(frame, 0.0)
    # QualityGate 的 BAD_S 迟滞还没到，但启动阶段没有可信历史，必须按 UNKNOWN。
    assert obs.state == UNKNOWN
    assert "画面质量未通过" in obs.reason


def test_reset_after_reconnect_requires_fresh_good_frame(tmp_path):
    make_clip(tmp_path / "clip", n=1)
    frame = next(iter(ClipSource(str(tmp_path / "clip"), realtime=False).frames()))
    pipe = ObservationPipeline(
        source_id="camera_top", zone_id="default",
        quality_fn=lambda color, depth: (True, ""),
        judge_fn=lambda frame, t, quality: (IDLE, "画面里没人", {"n_persons": 0}),
        quality_gate=FakeQualityGate(),
        grader=FakeGrader(),
    )
    assert pipe.step(frame, 0.0).state == CLEAR
    pipe.reset()
    # 换一个质量不可信的帧：重连后第一帧不能直接回到 CLEAR。
    pipe.quality_fn = lambda color, depth: (False, "深度图全空")
    assert pipe.step(frame, 2.0).state == UNKNOWN


def test_all_empty_depth_is_unknown_immediately(tmp_path):
    make_clip(tmp_path / "clip", n=2)
    frames = list(ClipSource(str(tmp_path / "clip"), realtime=False).frames())
    pipe = ObservationPipeline(
        source_id="camera_top", zone_id="default",
        quality_fn=lambda color, depth: (True, ""),   # 故意让 QualityGate 不迟滞
        judge_fn=lambda frame, t, quality: (IDLE, "画面里没人", {"n_persons": 0}),
        quality_gate=FakeQualityGate(blind=False),
        grader=FakeGrader(),
    )
    assert pipe.step(frames[0], 0.0).state == CLEAR
    empty = SimpleNamespace(color=frames[0].color,
                            depth=np.zeros_like(frames[0].depth), t=0.1)
    # 深度全空是硬失效，必须立刻 UNKNOWN，不能沿用上一帧的 CLEAR。
    obs = pipe.step(empty, 0.1)
    assert obs.state == UNKNOWN
    assert "深度图全空" in obs.reason
