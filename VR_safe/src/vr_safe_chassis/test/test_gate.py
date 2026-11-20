"""许可门公开行为测试。

Seam：``ChassisGate`` 的许可/输入/周期方法 → 假 SRP 的调用记录。
不测内部字段，不测 ROS，不需要真 AMR。
"""

import pytest

from vr_safe_chassis.gate import (PERMIT_GRANTED, PERMIT_REVOKED, PERMIT_SLOW,
                                  STATE_ESTOP, STATE_HOLD, ChassisGate)


class FakeRequestFailed(Exception):
    def __init__(self, result_code=70001):
        super().__init__(f"result_code={result_code}")
        self.result_code = result_code


class FakeSrpClient:
    def __init__(self):
        self.attempts = []
        self.speeds = []
        self.emergency_stops = 0
        self.remote_control_calls = []
        self.fail_speed = False
        self.result_code = 70001

    def set_remote_control_speed(self, linear_x_mm, linear_y_mm, angular_mrad):
        command = (int(linear_x_mm), int(linear_y_mm), int(angular_mrad))
        self.attempts.append(command)
        if self.fail_speed:
            raise FakeRequestFailed(self.result_code)
        self.speeds.append(command)

    def emergency_stop(self):
        self.emergency_stops += 1

    def set_remote_control(self, enabled):
        self.remote_control_calls.append(bool(enabled))


def make_gate(**overrides):
    # 这些用例不测 supervisor 失联；把许可看门狗调远，避免时间推进把它触发。
    overrides.setdefault("permit_watchdog_s", 999.0)
    fake = FakeSrpClient()
    gate = ChassisGate(fake, **overrides)
    gate.set_remote_control_enabled(True, 0.0)
    gate.on_rearm(0.0)
    return gate, fake


def zero_count(fake):
    return sum(1 for command in fake.speeds if command == (0, 0, 0))


def test_revoked_blocks_operator_and_keeps_zero_keepalive():
    gate, fake = make_gate()
    gate.on_permit(PERMIT_GRANTED, 0.0)
    assert gate.on_cmd_vel(0.4, 0.0, 0.6, 0.001) is True
    assert fake.speeds == [(400, 0, 600)]

    gate.on_permit(PERMIT_REVOKED, 0.002)
    # “立即零速”：进入 REVOKED 的同一调用里发出，不等 100 ms。
    assert fake.speeds[-1] == (0, 0, 0)
    assert gate.on_cmd_vel(0.4, 0.0, 0.6, 0.003) is False
    assert fake.speeds[-1] == (0, 0, 0)

    gate.tick(0.05)
    assert zero_count(fake) == 1
    gate.tick(0.102)
    assert zero_count(fake) == 2
    gate.tick(0.15)
    assert zero_count(fake) == 2
    gate.tick(0.202)
    assert zero_count(fake) == 3

    # REVOKED 期间绝不退出 MANUAL 模式（不调用 set_remote_control(false)）。
    assert fake.remote_control_calls == []

    snap = gate.snapshot()
    assert snap.state == STATE_HOLD
    assert snap.command_blocked is True
    assert snap.hold_active is True


def test_slow_scales_speed_and_clamps_to_limits():
    gate, fake = make_gate()
    gate.on_permit(PERMIT_SLOW, 0.0)
    assert gate.on_cmd_vel(0.4, 0.0, 0.6, 0.001) is True
    assert fake.speeds[-1] == (120, 0, 180)  # 0.3 × (0.4 m/s, 0.6 rad/s)

    assert gate.on_cmd_vel(2.0, -2.0, 2.0, 0.002) is True
    # 先夹到上限 (0.5, 0.5, 0.8)，再乘 0.3。
    assert fake.speeds[-1] == (150, -150, 240)


def test_five_consecutive_zero_failures_escalate_emergency_stop_once():
    gate, fake = make_gate()
    fake.fail_speed = True

    gate.on_permit(PERMIT_REVOKED, 0.0)  # 第 1 次失败
    for moment in (0.1, 0.2, 0.3, 0.4):
        gate.tick(moment)
    assert len(fake.attempts) == 5
    assert fake.emergency_stops == 1

    snap = gate.snapshot()
    assert snap.state == STATE_ESTOP
    assert snap.escalated is True

    # 升级后不再反复调用 SRP。
    gate.tick(0.5)
    assert len(fake.attempts) == 5
    assert fake.emergency_stops == 1


def test_zero_failure_counter_resets_after_success():
    gate, fake = make_gate()
    fake.fail_speed = True
    gate.on_permit(PERMIT_REVOKED, 0.0)  # 1 次失败

    fake.fail_speed = False
    gate.tick(0.1)  # 成功 → 连续计数清零
    assert gate.snapshot().zero_failures == 0

    fake.fail_speed = True
    for moment in (0.2, 0.3, 0.4, 0.5):
        gate.tick(moment)  # 再失败 4 次，仍不到 5
    assert gate.snapshot().zero_failures == 4
    assert fake.emergency_stops == 0


def test_twist_watchdog_zeroes_and_keeps_zero_until_new_twist():
    gate, fake = make_gate()
    gate.on_permit(PERMIT_GRANTED, 0.0)
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.001) is True
    assert fake.speeds[-1] == (200, 0, 0)

    gate.tick(0.2)  # 199 ms，不到 200 ms
    assert fake.speeds[-1] == (200, 0, 0)

    gate.tick(0.201)  # 200 ms 没有新 Twist → 自动零速
    assert fake.speeds[-1] == (0, 0, 0)
    snap = gate.snapshot()
    assert snap.state == STATE_HOLD
    assert snap.watchdog_tripped is True
    assert snap.hold_active is True

    gate.tick(0.301)  # 保持零速，而不是沉默
    assert zero_count(fake) == 2

    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.35) is True
    assert fake.speeds[-1] == (200, 0, 0)
    assert gate.snapshot().watchdog_tripped is False


def test_recovery_requires_deadman_and_stick_centering():
    gate, fake = make_gate()
    gate.on_permit(PERMIT_GRANTED, 0.0)
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.001) is True

    gate.on_permit(PERMIT_REVOKED, 0.01)
    assert fake.speeds[-1] == (0, 0, 0)
    gate.on_permit(PERMIT_GRANTED, 0.2)

    # 还没重新使能：即使有速度也阻断。
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.21) is False
    assert gate.snapshot().command_blocked is True

    # 重新使能了，但停止时摇杆还偏着：仍要求回中。
    assert gate.on_rearm(0.22) is True
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.23) is False

    # 摇杆回中，两个条件都满足后才允许新的速度。
    assert gate.on_cmd_vel(0.0, 0.0, 0.0, 0.24) is True
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.25) is True
    assert fake.speeds[-1] == (200, 0, 0)


def test_repeated_revoked_permit_does_not_reset_rearm_progress():
    gate, fake = make_gate()
    gate.on_permit(PERMIT_REVOKED, 0.0)
    assert gate.on_rearm(0.01) is True

    # supervisor 会周期重发 REVOKED；不能把操作员刚按的 Y 冲掉。
    gate.on_permit(PERMIT_REVOKED, 0.02)
    gate.on_permit(PERMIT_GRANTED, 0.03)
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.04) is True


def test_startup_needs_rearm_before_forwarding():
    fake = FakeSrpClient()
    gate = ChassisGate(fake)
    gate.set_remote_control_enabled(True, 0.0)
    gate.on_permit(PERMIT_GRANTED, 0.0)
    # 未重新使能时输入被阻断；这次偏杆也会被记入“恢复需回中”。
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.01) is False

    assert gate.on_rearm(0.02) is True
    # 停止期间出现过偏杆，重新使能后仍要先回中。
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.025) is False
    assert gate.on_cmd_vel(0.0, 0.0, 0.0, 0.03) is True
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.035) is True


def test_zero_keepalive_before_remote_enable_does_not_escalate():
    # 未进入 MANUAL 时 SRP 常回 70001。零速仍然照发（满足 REVOKED 立即/周期
    # 发零），但“还没按 Y”不能被误升级成 AMR 急停。
    fake = FakeSrpClient()
    fake.fail_speed = True
    gate = ChassisGate(fake)
    gate.on_permit(PERMIT_REVOKED, 0.0)  # 立即尝试，失败但未使能 → 不计数
    for moment in (0.1, 0.2, 0.3, 0.4, 0.5):
        gate.tick(moment)
    assert len(fake.attempts) == 6
    assert fake.emergency_stops == 0
    assert gate.snapshot().state == STATE_HOLD
    assert gate.snapshot().zero_failures == 0

    # 使能遥控后失败才开始计入连续 5 次。
    gate.set_remote_control_enabled(True, 0.6)
    for moment in (0.7, 0.8, 0.9, 1.0, 1.1):
        gate.tick(moment)
    assert fake.emergency_stops == 1
    assert gate.snapshot().state == STATE_ESTOP


def test_repeated_identical_commands_are_rate_limited_but_refresh_watchdog():
    gate, fake = make_gate()
    gate.on_permit(PERMIT_GRANTED, 0.0)
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.001) is True
    assert fake.speeds == [(200, 0, 0)]

    # 19 ms 内重复同一速度：不再打一次同步 SRP，但仍算“新 Twist”。
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.02) is True
    assert fake.speeds == [(200, 0, 0)]
    gate.tick(0.19)  # 距最近 Twist 170 ms，不到看门狗
    assert fake.speeds == [(200, 0, 0)]

    # 超过 50 ms 后重发一次。
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.052) is True
    assert fake.speeds == [(200, 0, 0), (200, 0, 0)]

    # 回中必须立刻补零，不能被 rate limit 拖住。
    assert gate.on_cmd_vel(0.0, 0.0, 0.0, 0.053) is True
    assert fake.speeds[-1] == (0, 0, 0)


def test_amr_estop_blocks_and_requires_centering_after_release():
    gate, fake = make_gate()
    gate.on_permit(PERMIT_GRANTED, 0.0)
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.001) is True

    gate.on_emergency_stop("service", 0.002)
    assert gate.snapshot().state == STATE_ESTOP
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.003) is False

    assert gate.on_rearm(0.004) is True
    # 急停时摇杆偏着：解除后必须先回中。
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.005) is False
    assert gate.on_cmd_vel(0.0, 0.0, 0.0, 0.006) is True
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.007) is True


def test_non_finite_command_cannot_reach_srp():
    gate, fake = make_gate()
    gate.on_permit(PERMIT_GRANTED, 0.0)
    assert gate.on_cmd_vel(float("nan"), float("inf"), -float("inf"), 0.001) is True
    assert fake.speeds[-1] == (0, 0, 0)


def test_gate_rejects_invalid_tuning():
    with pytest.raises(ValueError):
        ChassisGate(FakeSrpClient(), slow_scale=0.0)
    with pytest.raises(ValueError):
        ChassisGate(FakeSrpClient(), zero_keepalive_s=0.0)
    with pytest.raises(ValueError):
        ChassisGate(FakeSrpClient(), max_zero_failures=0)


def test_permit_watchdog_holds_when_supervisor_disappears():
    gate, fake = make_gate(permit_watchdog_s=0.3)
    gate.on_permit(PERMIT_GRANTED, 0.0)
    assert gate.on_cmd_vel(0.2, 0.0, 0.0, 0.01) is True
    assert fake.speeds[-1] == (200, 0, 0)

    gate.tick(0.31)
    snap = gate.snapshot()
    assert snap.state == STATE_HOLD
    assert snap.reason == "permit_timeout"
    assert snap.command_blocked is True
    assert fake.speeds[-1] == (0, 0, 0)
