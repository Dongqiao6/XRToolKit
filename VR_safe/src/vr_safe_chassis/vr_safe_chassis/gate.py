"""底盘许可门：纯逻辑，不依赖 ROS / SRP SDK。

这一层是工单 06 的安全核心，刻意做成“输入状态 + 显式时间 → SRP 调用”的纯
对象，便于用假时钟和假 SRP 做单元测试；ROS 订阅、服务和发布都在
``remote_control.py`` / ``state_publisher.py`` 里。

输入：
  - supervisor 的许可决定 GRANTED / SLOW / REVOKED；
  - 操作员速度（Joy 左摇杆或外部 Twist 转换后的 vx, vy, wz）；
  - 遥控使能（deadman）与重新使能事件；
  - 时间戳（由调用方传入 ``time.monotonic()``，测试传假时间）。

输出：
  - ``SrpClient.set_remote_control_speed(vx_mm, vy_mm, w_mrad)``；
  - 连续 ``max_zero_failures`` 次零速失败后调用 ``SrpClient.emergency_stop()``；
  - 供 ``ChassisState`` 发布的快照。

安全语义：
  - REVOKED：阻断操作员速度，立即并每 ``zero_keepalive_s`` 发一次零速；不退出
    MANUAL 模式，也不调用 ``move_pause``（它只对 action 有效）。
  - SLOW：速度按 ``slow_scale`` 缩放；GRANTED：原速。
  - 从 REVOKED 恢复：必须已经重新使能遥控（deadman）且摇杆回中，之前保持零速；
    重新使能不会自动补发停止期间累积的位移。
  - 本地看门狗：``twist_watchdog_s`` 内没有新 Twist 就自动零速。
  - 零速失败按“连续”计数，任意一次成功即清零；升级 emergency_stop 只做一次。
"""

from dataclasses import dataclass
import math
import threading

PERMIT_GRANTED = "GRANTED"
PERMIT_SLOW = "SLOW"
PERMIT_REVOKED = "REVOKED"
PERMITS = (PERMIT_GRANTED, PERMIT_SLOW, PERMIT_REVOKED)

STATE_MANUAL = "manual"
STATE_SLOW = "slow"
STATE_HOLD = "hold"
STATE_ESTOP = "estop"

# SRP 的速度单位是 mm/s 与 mrad/s；demo 的 utils/ros_bridge 用同一个比例。
SRP_SCALE = 1000.0
_NEUTRAL_EPS = 1e-6


@dataclass(frozen=True)
class GateSnapshot:
    """许可门对外可见的状态，用于 ChassisState 和日志。"""

    state: str
    permit: str
    command_blocked: bool
    hold_active: bool
    linear_mps: float
    linear_y_mps: float
    angular_rps: float
    armed: bool
    remote_control_enabled: bool
    zero_failures: int
    escalated: bool
    watchdog_tripped: bool
    stick_centered: bool
    reason: str
    last_error: str
    slow_scale: float


class ChassisGate:
    """AMR 遥控的本地安全闸门。"""

    def __init__(
        self,
        srp_client,
        *,
        slow_scale=0.3,
        zero_keepalive_s=0.1,
        twist_watchdog_s=0.2,
        permit_watchdog_s=0.3,
        max_zero_failures=5,
        max_linear_mps=0.5,
        max_angular_rps=0.8,
        speed_resend_s=0.05,
    ):
        if not 0.0 < float(slow_scale) <= 1.0:
            raise ValueError("slow_scale 必须在 (0, 1] 内")
        for name, value in (
            ("zero_keepalive_s", zero_keepalive_s),
            ("twist_watchdog_s", twist_watchdog_s),
            ("permit_watchdog_s", permit_watchdog_s),
            ("max_linear_mps", max_linear_mps),
            ("max_angular_rps", max_angular_rps),
        ):
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{name} 必须是正的有限数")
        if int(max_zero_failures) < 1:
            raise ValueError("max_zero_failures 必须 >= 1")
        if not math.isfinite(float(speed_resend_s)) or float(speed_resend_s) < 0.0:
            raise ValueError("speed_resend_s 必须是非负有限数")

        self._srp = srp_client
        self._slow_scale = float(slow_scale)
        self._zero_keepalive_s = float(zero_keepalive_s)
        self._twist_watchdog_s = float(twist_watchdog_s)
        self._permit_watchdog_s = float(permit_watchdog_s)
        self._max_zero_failures = int(max_zero_failures)
        self._max_linear_mps = float(max_linear_mps)
        self._max_angular_rps = float(max_angular_rps)
        self._speed_resend_s = float(speed_resend_s)
        self._lock = threading.RLock()

        self._permit = PERMIT_REVOKED
        self._permit_received = False
        self._last_permit_at = None
        self._remote_enabled = False
        self._needs_rearm = True
        self._require_center = False
        self._last_cmd = (0.0, 0.0, 0.0)
        self._last_sent = (0.0, 0.0, 0.0)
        self._last_twist_at = None
        self._last_motion_at = None
        self._last_zero_at = None
        self._zero_failures = 0
        self._last_error = ""
        self._escalated = False
        self._watchdog_tripped = False
        self._reason = "startup"

    # ── 许可与遥控使能 ──

    def on_permit(self, decision: str, now: float):
        """处理 supervisor 许可。

        重复的 REVOKED 只更新许可，不重置“重新使能/回中”进度：supervisor 会
        周期重发许可，操作员在 HOLD 期间按下的 Y 不能被下一帧冲掉。
        """
        if decision not in PERMITS:
            raise ValueError(f"未知许可：{decision!r}")
        with self._lock:
            previous = self._permit if self._permit_received else None
            self._permit_received = True
            self._last_permit_at = now
            self._permit = decision

            if decision == PERMIT_REVOKED:
                if previous != PERMIT_REVOKED:
                    self._needs_rearm = True
                    # 停止瞬间摇杆若还偏着，恢复前必须先看到回中。
                    self._require_center = not self._is_neutral(*self._last_cmd)
                    self._last_twist_at = None
                    self._watchdog_tripped = False
                    self._last_zero_at = None
                    if not self._escalated:
                        self._reason = "permit_revoked"
                    # “立即零速”：不等 100 ms 周期，进入 REVOKED 就发一次。
                    self._send_zero(now)
                elif not self._escalated:
                    self._reason = "permit_revoked"
                return

            if previous == PERMIT_REVOKED:
                # 恢复后的第一段仍保持零速，直到 deadman + 回中满足。
                self._last_zero_at = None
                self._reason = "recovery_pending"
            elif decision == PERMIT_SLOW:
                self._reason = "permit_slow"
            else:
                self._reason = "permit_granted"

    def set_remote_control_enabled(self, enabled: bool, now: float):
        """同步“AMR 是否已进入手动遥控”的确认状态。

        未使能时底盘不在 MANUAL，零速指令既没有对象也可能被 SRP 拒绝，因此
        不发零速、不计失败次数；使能后才开始零速 keepalive。
        """
        with self._lock:
            self._remote_enabled = bool(enabled)
            self._zero_failures = 0
            self._last_error = ""
            self._watchdog_tripped = False
            if enabled:
                self._last_zero_at = None
                self._reason = (
                    "permit_revoked"
                    if self._permit == PERMIT_REVOKED
                    else "remote_control_enabled"
                )
            else:
                self._needs_rearm = True
                if not self._is_neutral(*self._last_cmd):
                    self._require_center = True
                self._last_sent = (0.0, 0.0, 0.0)
                self._reason = "remote_control_disabled"

    def on_rearm(self, now: float) -> bool:
        """重新使能遥控（deadman）。仅清除“需要重新使能”，不代替摇杆回中。"""
        with self._lock:
            if not self._remote_enabled:
                self._reason = "remote_control_disabled"
                return False
            self._needs_rearm = False
            self._escalated = False
            self._zero_failures = 0
            self._last_error = ""
            self._watchdog_tripped = False
            self._reason = "rearmed"
            if self._permit == PERMIT_REVOKED:
                # 仍在 HOLD：恢复零速 keepalive 的“立即”语义。
                self._last_zero_at = None
            return True

    # ── 操作员输入 ──

    def on_cmd_vel(self, linear_x, linear_y, angular_z, now: float) -> bool:
        """收到一条操作员 Twist；返回是否真的下发给 AMR。"""
        vx = _finite_float(linear_x)
        vy = _finite_float(linear_y)
        wz = _finite_float(angular_z)
        with self._lock:
            self._last_cmd = (vx, vy, wz)
            self._last_twist_at = now
            neutral = self._is_neutral(vx, vy, wz)
            if neutral:
                # 摇杆已回中：恢复条件之一满足。
                self._require_center = False
            elif self._permit == PERMIT_REVOKED or self._needs_rearm:
                # 停止/恢复期间出现偏杆，恢复前必须再看到一次回中。
                self._require_center = True

            if self._escalated:
                self._reason = "zero_speed_escalated"
                return False
            if self._permit == PERMIT_REVOKED:
                self._reason = "permit_revoked"
                return False
            if not self._remote_enabled:
                self._reason = "remote_control_disabled"
                return False
            if self._needs_rearm:
                self._reason = "rearm_required"
                return False
            if self._require_center:
                self._reason = "stick_not_centered"
                return False

            if neutral:
                self._watchdog_tripped = False
                self._reason = "neutral"
                # 上一条是运动指令时必须立刻补零；连续中性帧只按 keepalive 发。
                if self._last_sent != (0.0, 0.0, 0.0) or self._zero_due(now):
                    return self._send_zero(now)
                return True

            vx, vy, wz = self._clamp_and_scale(vx, vy, wz)
            # Joy 可能以 100+ Hz 重复同一帧；相同速度最多按 speed_resend_s 重发，
            # 避免把每条 Joy 都变成一次同步 SRP 请求，同时让看门狗继续看到 Twist。
            if (
                self._last_sent == (vx, vy, wz)
                and self._last_motion_at is not None
                and (now - self._last_motion_at) < self._speed_resend_s - 1e-9
            ):
                self._watchdog_tripped = False
                self._reason = (
                    "slow_motion" if self._permit == PERMIT_SLOW else "manual_motion"
                )
                return True

            if not self._send_motion(vx, vy, wz, now):
                return False
            self._last_motion_at = now
            self._watchdog_tripped = False
            self._reason = (
                "slow_motion" if self._permit == PERMIT_SLOW else "manual_motion"
            )
            return True

    # ── 周期 ──

    def tick(self, now: float):
        """周期任务：许可看门狗、维持零速、恢复保持、Twist 看门狗。"""
        with self._lock:
            # supervisor 挂了：不能再拿最后一条 GRANTED 一直动。
            if (self._permit_received and self._last_permit_at is not None
                    and (now - self._last_permit_at) > self._permit_watchdog_s
                    and self._permit != PERMIT_REVOKED):
                self._permit = PERMIT_REVOKED
                self._needs_rearm = True
                self._require_center = not self._is_neutral(*self._last_cmd)
                self._last_twist_at = None
                self._watchdog_tripped = False
                self._last_zero_at = None
                self._reason = "permit_timeout"
                self._send_zero(now)

            if not self._can_forward():
                # REVOKED / 等 deadman / 等回中 / 遥控未使能：保持零速。
                if self._zero_due(now):
                    self._send_zero(now)
                return

            if self._watchdog_tripped:
                if self._zero_due(now):
                    self._send_zero(now)
                return

            if self._last_sent == (0.0, 0.0, 0.0):
                return  # 已停稳，不需要额外 keepalive

            if self._last_twist_at is None or \
                    (now - self._last_twist_at) >= self._twist_watchdog_s - 1e-9:
                self._watchdog_tripped = True
                self._reason = "twist_watchdog"
                self._send_zero(now)

    def on_emergency_stop(self, reason: str, now: float):
        """外部（按钮服务）已触发 AMR 急停：停止下发并等待人工重新使能。"""
        with self._lock:
            self._escalated = True
            self._needs_rearm = True
            if not self._is_neutral(*self._last_cmd):
                # 急停时摇杆若还偏着，解除后必须回中才能再动。
                self._require_center = True
            self._watchdog_tripped = False
            self._last_sent = (0.0, 0.0, 0.0)
            self._last_zero_at = now
            self._reason = f"amr_emergency_stop:{reason}"

    def on_shutdown(self, now: float):
        """退出前尽力补一帧零速；不计数、不升级，避免关机流程里触发急停。"""
        with self._lock:
            self._reason = "shutdown"
            self._needs_rearm = True
            if self._remote_enabled and not self._escalated:
                ok, _error = self._call_speed(0.0, 0.0, 0.0)
                if ok:
                    self._last_sent = (0.0, 0.0, 0.0)
                self._last_zero_at = now

    # ── 快照 ──

    def snapshot(self, now=None) -> GateSnapshot:
        with self._lock:
            if self._escalated:
                state = STATE_ESTOP
                blocked = True
                hold = True
            elif self._permit == PERMIT_REVOKED:
                state = STATE_HOLD
                blocked = True
                hold = True
            elif not self._remote_enabled:
                state = STATE_HOLD
                blocked = True
                hold = True
            elif self._needs_rearm or self._require_center:
                state = STATE_HOLD
                blocked = True
                hold = True
            elif self._watchdog_tripped:
                state = STATE_HOLD
                blocked = False
                hold = True
            elif self._permit == PERMIT_SLOW:
                state = STATE_SLOW
                blocked = False
                hold = False
            else:
                state = STATE_MANUAL
                blocked = False
                hold = False

            return GateSnapshot(
                state=state,
                permit=self._permit,
                command_blocked=blocked,
                hold_active=hold,
                linear_mps=self._last_sent[0],
                linear_y_mps=self._last_sent[1],
                angular_rps=self._last_sent[2],
                armed=self._remote_enabled and not self._needs_rearm,
                remote_control_enabled=self._remote_enabled,
                zero_failures=self._zero_failures,
                escalated=self._escalated,
                watchdog_tripped=self._watchdog_tripped,
                stick_centered=not self._require_center,
                reason=self._reason,
                last_error=self._last_error,
                slow_scale=self._slow_scale,
            )

    # ── 内部 ──

    def _can_forward(self) -> bool:
        return (
            self._permit != PERMIT_REVOKED
            and not self._escalated
            and self._remote_enabled
            and not self._needs_rearm
            and not self._require_center
        )

    def _zero_due(self, now: float) -> bool:
        if self._escalated:
            return False
        if self._last_zero_at is None:
            return True
        # 1e-9 容差：定时器/浮点边界不应让 100 ms keepalive 滑成 120 ms。
        return (now - self._last_zero_at) >= self._zero_keepalive_s - 1e-9

    def _send_motion(self, vx: float, vy: float, wz: float, now: float) -> bool:
        ok, error = self._call_speed(vx, vy, wz)
        if ok:
            self._last_sent = (vx, vy, wz)
            return True
        self._last_error = error
        # 下发失败时 AMR 会保留上一条速度，必须立刻补零；补零失败才计零速失败。
        self._send_zero(now)
        return False

    def _send_zero(self, now: float) -> bool:
        if self._escalated:
            return False
        ok, error = self._call_speed(0.0, 0.0, 0.0)
        self._last_zero_at = now
        if ok:
            self._zero_failures = 0
            self._last_sent = (0.0, 0.0, 0.0)
            return True

        self._last_error = error
        if not self._remote_enabled:
            # 还没使能遥控时 SRP 常回 70001（MANUAL_CONTROL_NOT_ON）。零速照发，
            # 但这是预期状态，不能把“操作员还没按 Y”误升级成 AMR 急停。
            return False

        self._zero_failures += 1
        if not self._escalated and \
                self._zero_failures >= self._max_zero_failures:
            self._escalated = True
            self._reason = "zero_speed_escalated"
            # 零速都发不出去时，只剩 AMR 自己的急停兜底；只升级一次。
            try:
                self._srp.emergency_stop()
            except Exception as exc:  # noqa: BLE001 - 硬件边界，记录但不抛出
                self._last_error = (
                    f"zero_speed_failed={error}; emergency_stop_failed={exc}"
                )
            return False
        return False

    def _call_speed(self, vx: float, vy: float, wz: float):
        try:
            self._srp.set_remote_control_speed(
                int(round(vx * SRP_SCALE)),
                int(round(vy * SRP_SCALE)),
                int(round(wz * SRP_SCALE)),
            )
            return True, ""
        except Exception as exc:  # noqa: BLE001 - 硬件边界，任何异常都按下发失败
            code = getattr(exc, "result_code", None)
            name = type(exc).__name__
            if code is None:
                return False, f"{name}: {exc}"
            return False, f"{name}(result_code={code})"

    def _clamp_and_scale(self, vx: float, vy: float, wz: float):
        vx = max(-self._max_linear_mps, min(self._max_linear_mps, vx))
        vy = max(-self._max_linear_mps, min(self._max_linear_mps, vy))
        wz = max(-self._max_angular_rps, min(self._max_angular_rps, wz))
        if self._permit == PERMIT_SLOW:
            vx *= self._slow_scale
            vy *= self._slow_scale
            wz *= self._slow_scale
        return vx, vy, wz

    @staticmethod
    def _is_neutral(vx: float, vy: float, wz: float) -> bool:
        return (
            abs(vx) <= _NEUTRAL_EPS
            and abs(vy) <= _NEUTRAL_EPS
            and abs(wz) <= _NEUTRAL_EPS
        )


def _finite_float(value) -> float:
    """把外部输入变成有限浮点数；NaN/Inf 按 0 处理，不能让它们进入 SRP。"""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return 0.0
    return result if math.isfinite(result) else 0.0
