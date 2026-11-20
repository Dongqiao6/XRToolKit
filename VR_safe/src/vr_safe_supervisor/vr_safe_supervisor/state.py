"""安全状态机：观察、许可、锁存、确认的纯逻辑实现。

策略（2026-11-20 对齐）：
  - 只有 **当前 OCCUPIED / 手伸入 / 头伸入** 才直接停止。
  - UNKNOWN 单独出现不停止；但如果上一次稳定观察是 OCCUPIED（人进过区，
    随后模型跟丢），要一直 HOLD 到重新看到 CLEAR/NEAR 并由人工确认。
  - 观察超时、观察源消失、遥操作心跳丢失仍然 HOLD（感知/链路故障不能
    和“画面里暂时看不清”混为一谈）。
  - NEAR 给 SLOW；CLEAR 给 GRANTED。
  - 确认后仍不自动跟随；双臂/底盘由各自执行端重新建立。
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

GRANTED = "GRANTED"
SLOW = "SLOW"
REVOKED = "REVOKED"

CLEAR = "CLEAR"
NEAR = "NEAR"
OCCUPIED = "OCCUPIED"
UNKNOWN = "UNKNOWN"
ESTOP = "ESTOP"

STATES = (CLEAR, NEAR, OCCUPIED, UNKNOWN, ESTOP)
_RANK = {CLEAR: 0, NEAR: 1, OCCUPIED: 2}


@dataclass(frozen=True)
class Permit:
    decision: str
    reason: str
    source: str
    latched: bool
    seq: int


@dataclass(frozen=True)
class Status:
    worst: Optional[str]
    source: str
    why: str
    failure: bool = False          # 观察超时/源消失，或启动中还没观察
    lost_after_occupied: bool = False


class SafetyStateMachine:
    """唯一许可的纯状态机。"""

    def __init__(
        self,
        observation_timeout_s: float = 0.3,
        heartbeat_timeout_s: float = 0.3,
        require_teleop_heartbeat: bool = True,
        required_sources: Tuple[str, ...] = ("camera_top",),
        teleop_source: str = "teleop",
        occupied_debounce_frames: int = 1,
    ):
        self.observation_timeout_s = float(observation_timeout_s)
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        self.require_teleop_heartbeat = bool(require_teleop_heartbeat)
        self.required_sources = tuple(required_sources)
        self.teleop_source = teleop_source
        self.occupied_debounce_frames = int(occupied_debounce_frames)
        if self.occupied_debounce_frames < 1:
            raise ValueError("occupied_debounce_frames 必须 >= 1")

        self._obs: Dict[str, Tuple[str, float, float]] = {}
        self._stable: Dict[str, str] = {}
        self._occupied_streak: Dict[str, int] = {}
        self._heartbeat_last: Optional[float] = None
        self._heartbeat_seen = False
        self._estop_latched = False
        self._hold_latched = False
        self._started = False

        self._seq = 0
        self._last_key = None
        self._permit = Permit(REVOKED, "启动中，等待有效观察", "startup", False, 0)

    @property
    def permit(self) -> Permit:
        return self._permit

    # ── 输入 ──
    def observe(self, source_id: str, state: str, now: float,
                confidence: float = 1.0) -> Permit:
        if state not in STATES:
            raise ValueError(f"unknown observation state: {state!r}")
        if state == ESTOP:
            return self.press_estop(now, source_id=source_id)
        self._obs[source_id] = (state, float(now), float(confidence))
        self._started = True
        if state == OCCUPIED:
            streak = self._occupied_streak.get(source_id, 0) + 1
            self._occupied_streak[source_id] = streak
            # 只有连续 N 帧占用才把“上次稳定”改成 OCCUPIED 并锁存。
            # 单帧跳点只让许可短暂 SLOW，不留下“跟丢后 HOLD”的记忆。
            if streak >= self.occupied_debounce_frames:
                self._stable[source_id] = OCCUPIED
                self._hold_latched = True
        else:
            self._occupied_streak[source_id] = 0
            if state in (CLEAR, NEAR):
                self._stable[source_id] = state
        return self.tick(now)

    def heartbeat(self, source_id: str, now: float) -> Permit:
        if source_id == self.teleop_source:
            self._heartbeat_last = float(now)
            self._heartbeat_seen = True
        return self.tick(now)

    def press_estop(self, now: float, source_id: str = "system_estop") -> Permit:
        self._estop_latched = True
        self._hold_latched = True
        self._started = True
        return self.tick(now)

    def clear_estop(self, now: float) -> Permit:
        self._estop_latched = False
        return self.tick(now)

    def confirm(self, confirmed_by: str, now: float,
                note: str = "") -> Tuple[bool, str, Permit]:
        self.tick(now)

        if self._estop_latched:
            return False, "系统软件急停还没解除", self._permit
        if self.require_teleop_heartbeat and not self._heartbeat_fresh(now):
            return False, "遥操作链路失联，不能恢复", self._permit

        status = self._status(now)
        if status.failure:
            return False, f"{status.why}，不能恢复", self._permit
        if status.lost_after_occupied:
            return False, "人进过区域后跟丢了，等重新看到 CLEAR/NEAR 再恢复", self._permit
        if status.worst == OCCUPIED:
            return False, "区域里还有人，不能恢复", self._permit
        if status.worst not in (CLEAR, NEAR):
            return False, "当前看不清，先等画面明确", self._permit

        self._hold_latched = False
        who = confirmed_by or "（没填）"
        if note:
            who += f"：{note}"
        if status.worst == NEAR:
            self._set(SLOW, f"{who} 人工确认；人在旁边，降速", "confirm", False)
            return True, "已恢复，人在旁边，只给慢速", self._permit
        self._set(GRANTED, f"{who} 人工确认恢复", "confirm", False)
        return True, "已恢复运动许可", self._permit

    def tick(self, now: float) -> Permit:
        return self._evaluate(float(now))

    # ── 内部 ──
    def _heartbeat_fresh(self, now: float) -> bool:
        if not self.require_teleop_heartbeat:
            return True
        if self._heartbeat_last is None:
            return False
        return (now - self._heartbeat_last) <= self.heartbeat_timeout_s

    def _occupied_confirmed(self) -> bool:
        return any(v >= self.occupied_debounce_frames
                   for v in self._occupied_streak.values())

    def _last_stable(self) -> Optional[str]:
        vals = set(self._stable.values())
        if NEAR in vals:
            return NEAR
        if CLEAR in vals:
            return CLEAR
        return None

    def _status(self, now: float) -> Status:
        fresh, stale = [], []
        for source_id, (state, t, conf) in self._obs.items():
            if (now - t) > self.observation_timeout_s:
                stale.append((source_id, state, t))
            else:
                fresh.append((source_id, state))

        for source_id in self.required_sources:
            if source_id not in self._obs:
                if self._started:
                    return Status(None, source_id,
                                  f"观察源 {source_id} 消失了", failure=True)
                return Status(None, source_id, "还没有有效观察")

        if stale:
            source_id, _state, t = stale[0]
            return Status(None, source_id,
                          f"观察超时（{source_id} 已 {now - t:.2f} s 没有新观察）",
                          failure=True)

        known = []
        lost_after = False
        lost_source = ""
        has_unknown = False
        for source_id, state in fresh:
            if state == OCCUPIED:
                known.append((OCCUPIED, source_id))
            elif state in (CLEAR, NEAR):
                known.append((state, source_id))
            elif state == UNKNOWN:
                has_unknown = True
                if self._stable.get(source_id) == OCCUPIED:
                    lost_after = True
                    lost_source = source_id

        if lost_after:
            return Status(UNKNOWN, lost_source,
                          "人进过区域后跟丢了（看不清）",
                          lost_after_occupied=True)

        if known:
            state, source_id = max(known, key=lambda x: _RANK[x[0]])
            if state == OCCUPIED:
                return Status(OCCUPIED, source_id, "区域被占")
            if state == NEAR:
                return Status(NEAR, source_id, "人在旁边")
            return Status(CLEAR, source_id, "区域空闲")

        if has_unknown:
            return Status(UNKNOWN, "camera_top", "看不清，未见过人")

        return Status(None, "startup", "还没有有效观察")

    def _evaluate(self, now: float) -> Permit:
        if self._estop_latched:
            return self._set(REVOKED, "系统软件急停", "system_estop", True)

        if self.require_teleop_heartbeat:
            if self._heartbeat_last is None:
                return self._set(REVOKED, "等待遥操作心跳", "teleop",
                                 self._hold_latched)
            if (now - self._heartbeat_last) > self.heartbeat_timeout_s:
                self._hold_latched = True
                return self._set(REVOKED, "遥操作链路失联", "teleop", True)

        status = self._status(now)

        if status.failure:
            if self._started:
                self._hold_latched = True
            return self._set(REVOKED, status.why, status.source,
                             self._hold_latched)

        if status.lost_after_occupied:
            self._hold_latched = True
            return self._set(REVOKED, status.why, status.source, True)

        if status.worst == OCCUPIED:
            if self._occupied_confirmed():
                self._hold_latched = True
                return self._set(REVOKED, status.why, status.source, True)
            # 去抖窗口内先降速；连续命中后再 HOLD。
            if self._hold_latched:
                return self._set(REVOKED, "等待人工确认", "confirm", True)
            return self._set(SLOW, "区域占用确认中（去抖）",
                             status.source, False)

        if self._hold_latched:
            return self._set(REVOKED, "等待人工确认", "confirm", True)

        if status.worst == NEAR:
            return self._set(SLOW, "人在旁边，降速", status.source, False)
        if status.worst == CLEAR:
            return self._set(GRANTED, "区域空闲", status.source, False)
        if status.worst == UNKNOWN:
            # 单独 UNKNOWN 不停止；上一次稳定是 NEAR 时仍慢速。
            if self._last_stable() == NEAR:
                return self._set(SLOW, "看不清，旁边有人过，保持慢速",
                                 status.source, False)
            return self._set(GRANTED, "看不清，未见过人（按策略不停止）",
                             status.source, False)

        # 启动中：还没看到任何观察。
        return self._set(REVOKED, status.why or "还没有有效观察",
                         status.source, self._hold_latched)

    def _set(self, decision: str, reason: str, source: str, latched: bool) -> Permit:
        key = (decision, reason, source, bool(latched))
        if key != self._last_key:
            self._seq += 1
            self._last_key = key
        self._permit = Permit(decision, reason, source, bool(latched), self._seq)
        return self._permit
