"""安全状态机的公开行为测试。

Seam: SafetyStateMachine 的 observe/heartbeat/press_estop/clear_estop/confirm/tick。
不测内部字段，不测 ROS。
"""

import pytest

from vr_safe_supervisor.state import SafetyStateMachine


def make(**kwargs):
    kwargs.setdefault("required_sources", ("camera_top",))
    return SafetyStateMachine(**kwargs)


def test_startup_is_revoked_until_observation_and_heartbeat():
    m = make()
    assert m.tick(0.0).decision == "REVOKED"
    m.observe("camera_top", "CLEAR", 0.0)
    assert m.tick(0.0).decision == "REVOKED"  # heartbeat still missing
    m.heartbeat("teleop", 0.0)
    assert m.tick(0.0).decision == "GRANTED"


def test_clear_grants_without_heartbeat_when_not_required():
    m = make(require_teleop_heartbeat=False)
    p = m.observe("camera_top", "CLEAR", 0.0)
    assert (p.decision, p.latched) == ("GRANTED", False)


def test_occupied_revokes_and_latches_until_confirm():
    m = make(require_teleop_heartbeat=False)
    m.observe("camera_top", "CLEAR", 0.0)
    assert m.observe("camera_top", "OCCUPIED", 0.1).decision == "REVOKED"
    assert m.permit.latched is True
    assert m.observe("camera_top", "CLEAR", 0.2).decision == "REVOKED"
    accepted, _msg, p = m.confirm("op", 0.2)
    assert accepted is True
    assert (p.decision, p.latched) == ("GRANTED", False)


def test_unknown_alone_does_not_stop():
    m = make(require_teleop_heartbeat=False)
    p = m.observe("camera_top", "UNKNOWN", 0.0)
    assert (p.decision, p.latched) == ("GRANTED", False)


def test_unknown_after_clear_keeps_grants():
    m = make(require_teleop_heartbeat=False)
    m.observe("camera_top", "CLEAR", 0.0)
    p = m.observe("camera_top", "UNKNOWN", 0.1)
    assert (p.decision, p.latched) == ("GRANTED", False)


def test_unknown_after_near_keeps_slow():
    m = make(require_teleop_heartbeat=False)
    m.observe("camera_top", "NEAR", 0.0)
    p = m.observe("camera_top", "UNKNOWN", 0.1)
    assert (p.decision, p.latched) == ("SLOW", False)


def test_unknown_after_occupied_holds_until_clear_and_confirm():
    m = make(require_teleop_heartbeat=False)
    m.observe("camera_top", "CLEAR", 0.0)
    m.observe("camera_top", "OCCUPIED", 0.1)
    p = m.observe("camera_top", "UNKNOWN", 0.2)
    assert (p.decision, p.latched) == ("REVOKED", True)
    accepted, msg, _ = m.confirm("op", 0.2)
    assert accepted is False
    assert "跟丢" in msg
    m.observe("camera_top", "CLEAR", 0.3)
    accepted, _msg, p = m.confirm("op", 0.3)
    assert accepted is True
    assert p.decision == "GRANTED"


def test_near_slows_and_auto_recovers_to_clear():
    m = make(require_teleop_heartbeat=False)
    assert m.observe("camera_top", "NEAR", 0.0).decision == "SLOW"
    assert m.permit.latched is False
    assert m.observe("camera_top", "CLEAR", 0.1).decision == "GRANTED"


def test_stale_observation_revokes_latches_and_blocks_confirm():
    m = make(require_teleop_heartbeat=False)
    m.observe("camera_top", "CLEAR", 0.0)
    p = m.tick(0.31)
    assert (p.decision, p.latched) == ("REVOKED", True)
    accepted, msg, _ = m.confirm("op", 0.32)
    assert accepted is False
    assert "观察" in msg
    m.observe("camera_top", "CLEAR", 0.33)
    accepted, _msg, p = m.confirm("op", 0.33)
    assert accepted is True
    assert p.decision == "GRANTED"


def test_heartbeat_loss_revokes_latches_and_blocks_confirm_until_it_returns():
    m = make()
    m.observe("camera_top", "CLEAR", 0.0)
    m.heartbeat("teleop", 0.0)
    assert m.tick(0.1).decision == "GRANTED"
    p = m.tick(0.31)
    assert (p.decision, p.latched, p.source) == ("REVOKED", True, "teleop")
    assert m.confirm("op", 0.32)[0] is False
    m.heartbeat("teleop", 0.33)
    m.observe("camera_top", "CLEAR", 0.33)
    accepted, _msg, p = m.confirm("op", 0.33)
    assert accepted is True
    assert p.decision == "GRANTED"


def test_estop_latches_clear_estop_keeps_hold_until_confirm():
    m = make(require_teleop_heartbeat=False)
    m.observe("camera_top", "CLEAR", 0.0)
    assert m.press_estop(0.1).decision == "REVOKED"
    p = m.clear_estop(0.2)
    assert (p.decision, p.latched) == ("REVOKED", True)
    accepted, _msg, p = m.confirm("op", 0.2)
    assert accepted is True
    assert p.decision == "GRANTED"


def test_confirm_rejected_while_occupied_then_accepted_after_clear():
    m = make(require_teleop_heartbeat=False)
    m.observe("camera_top", "CLEAR", 0.0)
    m.observe("camera_top", "OCCUPIED", 0.1)
    accepted, msg, _ = m.confirm("op", 0.1)
    assert accepted is False
    assert "区域" in msg
    m.observe("camera_top", "CLEAR", 0.2)
    accepted, _msg, p = m.confirm("op", 0.2)
    assert accepted is True
    assert p.decision == "GRANTED"


def test_confirm_while_near_grants_slow_only():
    m = make(require_teleop_heartbeat=False)
    m.observe("camera_top", "CLEAR", 0.0)
    m.observe("camera_top", "OCCUPIED", 0.1)
    m.observe("camera_top", "NEAR", 0.2)
    accepted, _msg, p = m.confirm("op", 0.2)
    assert accepted is True
    assert p.decision == "SLOW"


def test_confirm_requires_a_fresh_observation_after_startup():
    m = make(require_teleop_heartbeat=False)
    accepted, msg, p = m.confirm("op", 0.0)
    assert accepted is False
    assert p.decision == "REVOKED"


def test_seq_increases_when_permit_changes():
    m = make(require_teleop_heartbeat=False)
    p1 = m.observe("camera_top", "CLEAR", 0.0)
    p2 = m.observe("camera_top", "OCCUPIED", 0.1)
    p3 = m.observe("camera_top", "UNKNOWN", 0.2)
    assert p1.seq < p2.seq < p3.seq


def test_unknown_state_rejected():
    m = make(require_teleop_heartbeat=False)
    with pytest.raises(ValueError):
        m.observe("camera_top", "BOGUS", 0.0)


def test_occupied_debounce_needs_two_consecutive_frames():
    m = make(require_teleop_heartbeat=False, occupied_debounce_frames=2)
    m.observe("camera_top", "CLEAR", 0.0)
    first = m.observe("camera_top", "OCCUPIED", 0.1)
    assert (first.decision, first.latched) == ("SLOW", False)
    second = m.observe("camera_top", "OCCUPIED", 0.2)
    assert (second.decision, second.latched) == ("REVOKED", True)


def test_single_occupied_frame_does_not_leave_lost_memory():
    m = make(require_teleop_heartbeat=False, occupied_debounce_frames=2)
    m.observe("camera_top", "CLEAR", 0.0)
    m.observe("camera_top", "OCCUPIED", 0.1)
    p = m.observe("camera_top", "UNKNOWN", 0.2)
    assert (p.decision, p.latched) == ("GRANTED", False)
