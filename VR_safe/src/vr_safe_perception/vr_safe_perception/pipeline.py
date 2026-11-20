"""单帧判定流水线：画面质量 → judge_multi → 迟滞 → 四值观察。

这个模块不 import mediapipe / rclpy / 相机 SDK。真实推理和相机由
runtime.py、sources.py 注入，测试可以给假的帧源和假的判定函数，
从而在没相机、没模型的机器上验证「回放 → 观察」的构造。
"""

from typing import Any, Callable, Dict, Optional, Tuple

from .mapping import (BESIDE, IDLE, UNCLEAR, Observation, build_observation)

# 人从画面里突然消失时，先按「看不清」保持这么久，不要立刻放行。
# 和 camera_zone/zone_monitor.py 的 VANISH_HOLD_S 保持一致。
VANISH_HOLD_S = 1.5


class ObservationPipeline:
    """把一帧的原始判定收敛成一条 SafetyObservation。

    注入点：
        quality_fn(color, depth) -> (ok, why)
        judge_fn(frame, t, quality) -> (五档状态, 理由, info)
        quality_gate.feed(ok, t) -> (blind, blind_s)
        grader.feed(raw, t) -> 五档稳定状态

    judge_fn 拿到的 quality 是 (blind, why, blind_s)，和 zone_monitor.judge_multi
    的 quality 参数同形。
    """

    def __init__(self, source_id: str, zone_id: str,
                 quality_fn: Callable[[Any, Any], Tuple[bool, str]],
                 judge_fn: Callable[[Any, float, Tuple[bool, str, float]],
                                    Tuple[str, str, Dict[str, Any]]],
                 quality_gate: Any, grader: Any,
                 zone_mode: str = "polygon",
                 vanish_hold_s: float = VANISH_HOLD_S):
        self.source_id = str(source_id)
        self.zone_id = str(zone_id)
        self.quality_fn = quality_fn
        self.judge_fn = judge_fn
        self.quality_gate = quality_gate
        self.grader = grader
        self.zone_mode = str(zone_mode)
        self.vanish_hold_s = float(vanish_hold_s)
        self._last_seen = -1e9
        # 启动/相机重连后还没有一帧通过质量门时，不能把「什么都测不到」
        # 当成 CLEAR。QualityGate 的时间迟滞是为短暂遮挡设计的，不能拿
        # 它给启动阶段兜底。
        self._seen_good = False

    def step(self, frame: Any, t: Optional[float] = None) -> Observation:
        """处理一帧。frame 至少要有 color/depth/t 三个属性。"""
        if t is None:
            t = float(getattr(frame, "t", 0.0))

        ok, why = self.quality_fn(frame.color, frame.depth)
        blind, blind_s = self.quality_gate.feed(ok, t)
        quality = (bool(blind), why or "", float(blind_s))

        # 深度一个有效像素都没有是硬失效（相机掉线/深度流死掉），不是
        # 「闪一下」。它必须立刻 UNKNOWN，不能等 QualityGate 的 BAD_S 迟滞。
        depth = getattr(frame, "depth", None)
        depth_all_empty = (depth is not None and getattr(depth, "size", 0) > 0
                           and not bool((depth > 0).any()))

        raw, reason, info = self.judge_fn(frame, t, quality)
        info = dict(info or {})
        if ok:
            self._seen_good = True
        if depth_all_empty and raw in (IDLE, BESIDE):
            raw = UNCLEAR
            reason = "深度图全空（相机或深度流失效）"
        elif not self._seen_good and raw in (IDLE, BESIDE):
            raw = UNCLEAR
            reason = f"画面质量未通过（还没有可信帧）：{why or '原因未知'}"
        if info.get("n_persons"):
            self._last_seen = t
        elif raw == IDLE and (t - self._last_seen) < self.vanish_hold_s:
            # 和 zone_monitor.main 的兜底一致：人刚还在，突然检不到，
            # 多半是被挡住或转身，不能当成「区域空了」。
            raw = UNCLEAR
            reason = f"人突然检不到了（{t - self._last_seen:.1f} s）"

        state_five = self.grader.feed(raw, t)

        detail = dict(info)
        detail.update({
            "raw": raw,
            "five_state": state_five,
            "why": reason,
            "zone_mode": self.zone_mode,
            "quality_ok": bool(ok),
            "quality_why": why or "",
            "depth_all_empty": bool(depth_all_empty),
            "blind_s": round(float(blind_s), 3),
            "t": round(float(t), 3),
        })
        obs = build_observation(self.source_id, self.zone_id, state_five,
                                reason=reason, info=detail)
        return obs

    def reset(self):
        """相机重连后调用：清掉「上一段视频里积累的可信历史」。"""
        self._last_seen = -1e9
        self._seen_good = False
        gate = self.quality_gate
        for attr in ("bad_since", "good_since"):
            if hasattr(gate, attr):
                setattr(gate, attr, None)
        if hasattr(gate, "blind"):
            gate.blind = False
