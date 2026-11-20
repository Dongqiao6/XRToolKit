"""Joy 映射纯函数测试：死区、速度上限、按钮下标（含 -1 禁用）。"""

import pytest

from vr_safe_chassis.joy import BUTTON_ORDER, button_pressed, joy_to_twist


def test_button_order_matches_teleop_layout():
    # ADR 0005 / teleop：LJ = buttons[6]，Y = buttons[1]，X/A 是夹爪锁定。
    assert BUTTON_ORDER.index("X") == 0
    assert BUTTON_ORDER.index("Y") == 1
    assert BUTTON_ORDER.index("A") == 2
    assert BUTTON_ORDER.index("LJ") == 6


def test_disabled_button_index_never_reads_last_button():
    buttons = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    assert button_pressed(buttons, -1) is False
    assert button_pressed(buttons, 99) is False
    assert button_pressed(buttons, 1) is False


def test_button_pressed_reads_valid_index():
    buttons = [0, 1, 0]
    assert button_pressed(buttons, 1) is True
    assert button_pressed(buttons, 0) is False


def test_centered_stick_and_short_axes_are_zero():
    assert joy_to_twist([0.0] * 6, 0.2, 0.5, 0.8) == (0.0, 0.0, 0.0)
    assert joy_to_twist([0.0, 0.0, 0.0, 0.0, 0.1, 0.1], 0.2, 0.5, 0.8) == \
        (0.0, 0.0, 0.0)
    assert joy_to_twist([0.0, 0.0], 0.2, 0.5, 0.8) == (0.0, 0.0, 0.0)


def test_forward_axis_scales_to_linear_limit():
    axes = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    assert joy_to_twist(axes, 0.2, 0.5, 0.8) == (0.5, 0.0, 0.0)


def test_partial_forward_re_normalizes_after_deadzone():
    # (0.6 - 0.2) / (1 - 0.2) = 0.5 → 0.25 m/s
    axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.6]
    linear, linear_y, angular = joy_to_twist(axes, 0.2, 0.5, 0.8)
    assert linear == pytest.approx(0.25)
    assert (linear_y, angular) == (0.0, 0.0)


def test_right_stick_x_turns_left_negative_angular():
    axes = [0.0, 0.0, 0.0, 0.0, 0.5, 0.0]
    linear, _, angular = joy_to_twist(axes, 0.2, 0.5, 0.8)
    assert linear == 0.0
    assert angular == pytest.approx(-0.8 * 0.375)


def test_deadzone_extremes():
    assert joy_to_twist([0.0] * 5 + [0.2], 0.2, 0.5, 0.8) == (0.0, 0.0, 0.0)
    limit = joy_to_twist([0.0] * 5 + [1.0], 0.0, 0.5, 0.8)
    assert limit == (0.5, 0.0, 0.0)
