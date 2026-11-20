"""TeleopCore 单测：按键语义、跟踪丢失、双臂模式与心跳序号。

这些测试不 import rclpy，也不需要 XRoboToolkit SDK。
"""

import pytest

from vr_safe_teleop.teleop_core import TeleopCore
from vr_safe_teleop.xrt_adapter import (
    A_BUTTON_INDEX,
    B_BUTTON_INDEX,
    LJ_BUTTON_INDEX,
    X_BUTTON_INDEX,
    Y_BUTTON_INDEX,
)

TRACKED_POSE = (0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0)
OTHER_TRACKED_POSE = (-0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 1.0)


def controller(pose=TRACKED_POSE, **overrides):
    state = {
        "pose": pose,
        "grip": 0.0,
        "trigger": 0.0,
        "axis": (0.0, 0.0),
        "X": False,
        "Y": False,
        "A": False,
        "B": False,
        "menu": False,
        "axis_click": False,
    }
    state.update(overrides)
    return state


def make_core(**kwargs):
    # alpha=1 时输出直接跟随原始位姿，便于比较目标。
    kwargs.setdefault("smooth_alpha", 1.0)
    return TeleopCore(**kwargs)


def assert_pose_equal(first, second):
    assert first is not None and second is not None
    assert first[0] == pytest.approx(second[0])
    assert first[1] == pytest.approx(second[1])


def test_lj_rising_edge_publishes_one_estop():
    core = make_core()
    frame = core.update(controller(axis_click=True), controller())
    assert frame.estop_pressed is True
    assert "LJ" in frame.estop_reason
    assert "系统软件急停" in frame.estop_reason
    assert frame.clear_estop_pressed is False

    # 按住不放只发一次，避免 125 Hz 刷屏。
    held = core.update(controller(axis_click=True), controller())
    assert held.estop_pressed is False

    released = core.update(controller(axis_click=False), controller())
    assert released.estop_pressed is False

    # 松开再按才是新的一次急停。
    pressed_again = core.update(controller(axis_click=True), controller())
    assert pressed_again.estop_pressed is True


def test_y_rising_edge_requests_clear_estop_only_once():
    core = make_core()
    frame = core.update(controller(Y=True), controller())
    assert frame.clear_estop_pressed is True
    assert "Y" in frame.clear_estop_reason
    assert "按钮锁存" in frame.clear_estop_reason
    assert frame.estop_pressed is False

    assert core.update(controller(Y=True), controller()).clear_estop_pressed is False
    assert core.update(controller(Y=False), controller()).clear_estop_pressed is False
    assert core.update(controller(Y=True), controller()).clear_estop_pressed is True


def test_x_and_a_do_not_trigger_estop_or_clear():
    core = make_core()
    frame = core.update(controller(X=True), controller(A=True))
    assert frame.estop_pressed is False
    assert frame.clear_estop_pressed is False
    # X/A 仍然原样进入 Joy，夹爪锁定由下游臂节点解释。
    assert frame.joy_buttons[X_BUTTON_INDEX] == 1
    assert frame.joy_buttons[A_BUTTON_INDEX] == 1
    assert frame.joy_buttons[Y_BUTTON_INDEX] == 0
    assert frame.joy_buttons[LJ_BUTTON_INDEX] == 0


def test_lj_does_not_change_gripper_inputs_or_x_a():
    core = make_core()
    left = controller(
        grip=0.8, trigger=0.6, axis=(0.2, -0.3), X=True
    )
    right = controller(
        pose=OTHER_TRACKED_POSE, grip=0.45, trigger=0.7, axis=(-0.1, 0.4),
        A=True,
    )
    baseline = core.update(left, right)
    baseline_axes = baseline.joy_axes
    baseline_x = baseline.joy_buttons[X_BUTTON_INDEX]
    baseline_a = baseline.joy_buttons[A_BUTTON_INDEX]

    left_with_lj = dict(left, axis_click=True)
    pressed = core.update(left_with_lj, right)
    assert pressed.estop_pressed is True
    # LJ 是系统急停，不是夹爪按键：模拟量、Trigger、X/A 都不应被改写。
    assert pressed.joy_axes == baseline_axes
    assert pressed.joy_buttons[X_BUTTON_INDEX] == baseline_x
    assert pressed.joy_buttons[A_BUTTON_INDEX] == baseline_a
    assert pressed.joy_buttons[LJ_BUTTON_INDEX] == 1


def test_y_does_not_change_gripper_inputs():
    core = make_core()
    left = controller(grip=0.9, trigger=0.4, axis=(0.3, 0.1))
    right = controller(pose=OTHER_TRACKED_POSE, grip=0.2, trigger=0.8)
    baseline = core.update(left, right)
    pressed = core.update(dict(left, Y=True), right)
    assert pressed.clear_estop_pressed is True
    assert pressed.joy_axes == baseline.joy_axes
    assert pressed.estop_pressed is False


def test_lj_wins_over_y_in_the_same_frame():
    core = make_core()
    frame = core.update(controller(axis_click=True, Y=True), controller())
    assert frame.estop_pressed is True
    assert frame.clear_estop_pressed is False
    assert "clear_estop_ignored_while_estop" in frame.events


def test_tracking_loss_stops_targets_and_zeroes_grip_and_axes():
    core = make_core()
    left = controller(grip=0.8, trigger=0.6, axis=(0.4, -0.2))
    right = controller(
        pose=OTHER_TRACKED_POSE, grip=0.7, trigger=0.3, axis=(-0.5, 0.1)
    )
    first = core.update(left, right)
    assert first.targets["left"] is not None
    assert first.targets["right"] is not None
    assert first.tracked == {"left": True, "right": True}

    lost = core.update(controller(pose=()), controller(pose=()))
    assert lost.targets["left"] is None
    assert lost.targets["right"] is None
    assert lost.tracked == {"left": False, "right": False}
    assert lost.joy_axes[0] == 0.0  # 左 Grip
    assert lost.joy_axes[1] == 0.0  # 右 Grip
    assert lost.joy_axes[3] == pytest.approx(0.3)  # Trigger 保持最后有效值
    assert lost.joy_axes[4] == 0.0  # 左摇杆回中
    assert lost.joy_axes[5] == 0.0
    assert "left_tracking_lost" in lost.events
    assert "right_tracking_lost" in lost.events

    acquired = core.update(left, right)
    assert acquired.targets["left"] is not None
    assert "left_tracking_acquired" in acquired.events


def test_lj_and_y_still_work_when_tracking_lost():
    # 安全按键不能依赖位姿跟踪：LJ 仍要能停，Y 仍要能清按钮锁存。
    core = make_core()
    lost_lj = core.update(controller(pose=(), axis_click=True), controller())
    assert lost_lj.estop_pressed is True
    assert lost_lj.targets["left"] is None

    clear_core = make_core()
    lost_y = clear_core.update(controller(pose=(), Y=True), controller())
    assert lost_y.clear_estop_pressed is True
    assert lost_y.targets["left"] is None


def test_x_a_and_b_are_ignored_when_tracking_lost():
    # 无效手柄状态不能误切夹爪或双臂模式。
    core = make_core()
    frame = core.update(
        controller(pose=(), X=True, B=True),
        controller(pose=(), A=True),
    )
    assert frame.joy_buttons[X_BUTTON_INDEX] == 0
    assert frame.joy_buttons[A_BUTTON_INDEX] == 0
    assert frame.joy_buttons[B_BUTTON_INDEX] == 0
    assert core.dual_arm_mode is False


def test_trigger_holds_last_valid_value_on_tracking_loss():
    core = make_core()
    core.update(controller(trigger=0.7), controller())
    lost = core.update(controller(pose=()), controller())
    # 跟踪丢失不能命令夹爪变化，沿用最后有效 Trigger。
    assert lost.joy_axes[2] == pytest.approx(0.7)
    assert lost.joy_axes[0] == 0.0


def test_heartbeat_sequence_is_monotonic_and_independent_of_tracking():
    core = make_core()
    seqs = [core.next_heartbeat_seq() for _ in range(3)]
    assert seqs == [1, 2, 3]
    # 模拟跟踪丢失，心跳序号不受影响（节点用独立定时器发心跳）。
    core.update(controller(pose=()), controller(pose=()))
    assert core.next_heartbeat_seq() == 4
    assert core.heartbeat_seq == 4


def test_dual_arm_mode_drives_both_arms_from_right_controller():
    core = make_core()
    left = controller(grip=0.8, trigger=0.6)
    right = controller(pose=OTHER_TRACKED_POSE, grip=0.35, trigger=0.2)
    right_with_b = dict(right, B=True)

    enabled = core.update(left, right_with_b, yaw_offset=0.0)
    assert_pose_equal(enabled.targets["left"], enabled.targets["right"])
    assert core.dual_arm_mode is True
    # 双臂一起离合：左 Grip 数字/模拟量跟随右 Grip。
    assert enabled.joy_axes[0] == pytest.approx(0.35)
    assert enabled.joy_axes[1] == pytest.approx(0.35)
    # 左 Trigger 仍独立控制左夹爪。
    assert enabled.joy_axes[2] == pytest.approx(0.6)
    assert "dual_arm_enabled" in enabled.events

    # 按住 B 不应重复切换。
    still_held = core.update(left, right_with_b)
    assert_pose_equal(still_held.targets["left"], still_held.targets["right"])
    assert core.dual_arm_mode is True

    # 松开再按一次退出双臂模式，左臂回到左手柄。
    core.update(left, right)
    disabled = core.update(
        controller(pose=(0.9, 0.9, 0.9, 0.0, 0.0, 0.0, 1.0)),
        dict(right, B=True),
    )
    assert core.dual_arm_mode is False
    assert disabled.targets["left"] != disabled.targets["right"]
    assert "dual_arm_disabled" in disabled.events


def test_disabled_side_publishes_no_target_and_neutral_joy():
    core = make_core(enable_left=False)
    left = controller(grip=0.9, X=True, axis=(0.5, 0.5))
    right = controller(pose=OTHER_TRACKED_POSE, grip=0.4)
    frame = core.update(left, right)
    assert frame.targets["left"] is None
    assert frame.tracked["left"] is False
    assert frame.targets["right"] is not None
    assert frame.joy_axes[0] == 0.0
    assert frame.joy_buttons[X_BUTTON_INDEX] == 0


def test_joy_changed_is_reported_for_diagnostics_only():
    # 节点不再用这个字段做发布门控（Joy 必须每 tick 发），它只用于诊断。
    core = make_core()
    left = controller(grip=0.5)
    right = controller(pose=OTHER_TRACKED_POSE, grip=0.2)
    assert core.update(left, right).joy_changed is True
    assert core.update(left, right).joy_changed is False
    assert core.update(dict(left, grip=0.6), right).joy_changed is True


def test_unknown_button_name_is_rejected():
    with pytest.raises(ValueError):
        TeleopCore(estop_button="NO_SUCH_BUTTON")
    with pytest.raises(ValueError):
        TeleopCore(clear_estop_button="LJ_NOT_REAL")


def test_both_sides_disabled_is_rejected():
    with pytest.raises(ValueError):
        TeleopCore(enable_left=False, enable_right=False)


def test_rj_long_press_requests_confirm_once():
    core = make_core(confirm_button="RJ", confirm_hold_s=1.5)
    right = controller(axis_click=True)
    assert core.update(controller(), right, now_s=0.0).confirm_pressed is False
    assert core.update(controller(), right, now_s=1.4).confirm_pressed is False
    fired = core.update(controller(), right, now_s=1.5)
    assert fired.confirm_pressed is True
    assert "RJ" in fired.confirm_reason
    # 按住不放只发一次。
    assert core.update(controller(), right, now_s=1.6).confirm_pressed is False
    # 松开再按是一次新的长按。
    core.update(controller(), controller(axis_click=False), now_s=1.7)
    assert core.update(controller(), right, now_s=1.8).confirm_pressed is False
    assert core.update(controller(), right, now_s=3.4).confirm_pressed is True


def test_rj_works_without_pose_tracking():
    core = make_core(confirm_button="RJ", confirm_hold_s=1.5)
    right = controller(pose=(), axis_click=True)
    core.update(controller(pose=()), right, now_s=0.0)
    assert core.update(controller(pose=()), right, now_s=1.5).confirm_pressed is True
