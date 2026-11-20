"""xrt_adapter 的纯 Python 单测：坐标、四元数、Joy 布局与按键下标。"""

import math

import pytest

from vr_safe_teleop.xrt_adapter import (
    A_BUTTON_INDEX,
    B_BUTTON_INDEX,
    BUTTON_INDEX,
    BUTTON_ORDER,
    DEFAULT_HEADSET_TO_ROBOT,
    LJ_BUTTON_INDEX,
    X_BUTTON_INDEX,
    Y_BUTTON_INDEX,
    joy_payload,
    matrix_3x3,
    normalize_quaternion,
    quaternion_from_matrix,
    quaternion_to_matrix,
    raw_pose_is_valid,
    smooth_pose,
    transform_pose,
)


def test_button_layout_matches_adr_0005():
    # ADR 0005 明确：LJ = Joy.buttons[6]，Y = buttons[1]，X/A 是左右夹爪锁定。
    assert BUTTON_ORDER[6] == "LJ"
    assert BUTTON_ORDER[1] == "Y"
    assert BUTTON_ORDER[0] == "X"
    assert BUTTON_ORDER[2] == "A"
    assert BUTTON_ORDER[3] == "B"
    assert LJ_BUTTON_INDEX == 6
    assert Y_BUTTON_INDEX == 1
    assert X_BUTTON_INDEX == 0
    assert A_BUTTON_INDEX == 2
    assert B_BUTTON_INDEX == 3
    assert BUTTON_INDEX["LJ"] == 6


def test_default_mapping_matches_legacy_axis_conversion():
    position, quaternion = transform_pose(
        (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    )
    assert position == (-3.0, -1.0, 2.0)
    assert quaternion == (1.0, 0.0, 0.0, 0.0)


def test_identity_mapping_preserves_pose():
    identity = matrix_3x3((1, 0, 0, 0, 1, 0, 0, 0, 1))
    position, quaternion = transform_pose(
        (0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0),
        mapping=identity,
    )
    assert position == pytest.approx((0.1, 0.2, 0.3))
    assert quaternion[0] == pytest.approx(1.0)


def test_default_mapping_quaternion_is_proper_rotation():
    quaternion = quaternion_from_matrix(matrix_3x3(DEFAULT_HEADSET_TO_ROBOT))
    assert math.sqrt(sum(value * value for value in quaternion)) == pytest.approx(1.0)


def test_default_mapping_rotates_controller_axes():
    half_sqrt_two = math.sqrt(0.5)
    _, quaternion = transform_pose(
        (0.0, 0.0, 0.0, 0.0, half_sqrt_two, 0.0, half_sqrt_two)
    )
    assert quaternion[0] == pytest.approx(half_sqrt_two)
    assert quaternion[1] == pytest.approx(0.0)
    assert quaternion[2] == pytest.approx(0.0)
    assert quaternion[3] == pytest.approx(half_sqrt_two)


def test_yaw_offset_rotates_motion_about_robot_z():
    position, _ = transform_pose(
        (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
        yaw_offset=math.pi,
    )
    assert position == pytest.approx((3.0, 1.0, 2.0))


def test_yaw_offset_quarter_turn():
    position, _ = transform_pose(
        (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
        yaw_offset=0.5 * math.pi,
    )
    assert position == pytest.approx((1.0, -3.0, 2.0))


def test_quaternion_matrix_round_trip():
    source = normalize_quaternion((0.5, -0.5, 0.5, 0.5))
    matrix = quaternion_to_matrix(source)
    restored = quaternion_from_matrix(matrix)
    assert abs(sum(a * b for a, b in zip(source, restored))) == pytest.approx(1.0)


def test_zero_pose_is_untracked():
    assert not raw_pose_is_valid((0.0,) * 7)
    assert not raw_pose_is_valid(())
    assert not raw_pose_is_valid((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float("nan")))
    assert raw_pose_is_valid((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))


def test_transform_pose_rejects_untracked_pose():
    with pytest.raises(ValueError):
        transform_pose((0.0,) * 7)


def test_invalid_mapping_rejected():
    with pytest.raises(ValueError):
        matrix_3x3((1, 0, 0, 0, 1, 0, 0, 0, 0))
    with pytest.raises(ValueError):
        matrix_3x3((1, 0, 0, 0, 1, 0, 0, 0))


def test_joy_layout_preserves_legacy_axes_and_buttons():
    axes, buttons = joy_payload(
        {
            "grip": 0.8,
            "trigger": 0.25,
            "axis": (0.1, -0.2),
            "X": True,
            "menu": True,
            "axis_click": True,
        },
        {
            "grip": 0.2,
            "trigger": 0.9,
            "axis": (-0.3, 0.4),
            "A": True,
        },
    )
    assert axes == pytest.approx([0.8, 0.2, 0.25, 0.9, 0.1, -0.2, -0.3, 0.4])
    assert len(buttons) == len(BUTTON_ORDER)
    assert buttons[BUTTON_INDEX["X"]] == 1
    assert buttons[BUTTON_INDEX["LJ"]] == 1
    assert buttons[BUTTON_INDEX["LG"]] == 1
    assert buttons[BUTTON_INDEX["RG"]] == 0
    assert buttons[BUTTON_INDEX["RTr"]] == 1
    # 仅这些按钮为 1，其他保持 0，防止布局串位。
    assert [i for i, value in enumerate(buttons) if value] == sorted([
        BUTTON_INDEX["X"], BUTTON_INDEX["A"], BUTTON_INDEX["LJ"],
        BUTTON_INDEX["LG"], BUTTON_INDEX["LThU"], BUTTON_INDEX["RTr"],
    ])


def test_joy_layout_puts_y_at_button_1_and_a_at_button_2():
    _, buttons = joy_payload(
        {"Y": True, "axis_click": False},
        {"A": True},
    )
    assert buttons[Y_BUTTON_INDEX] == 1
    assert buttons[A_BUTTON_INDEX] == 1
    assert buttons[LJ_BUTTON_INDEX] == 0
    assert buttons[X_BUTTON_INDEX] == 0


def test_joy_payload_clamps_analog_values_and_axes():
    axes, _ = joy_payload(
        {"grip": 2.0, "trigger": -1.0, "axis": (3.0, -4.0)},
        {"grip": -1.0, "trigger": 3.0, "axis": ("bad", 0.5)},
    )
    assert axes[0] == 1.0
    assert axes[1] == 0.0
    assert axes[2] == 0.0
    assert axes[3] == 1.0
    assert axes[4] == 1.0
    assert axes[5] == -1.0
    # 非法摇杆按回中处理，不能把坏值透传给下游。
    assert axes[6] == 0.0
    assert axes[7] == 0.0


def test_smoothing_spans_between_samples():
    position, _ = smooth_pose(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
        (1.0, 2.0, 3.0),
        (1.0, 0.0, 0.0, 0.0),
        0.5,
    )
    assert position == pytest.approx((0.5, 1.0, 1.5))


def test_smoothing_handles_quaternion_sign_flip():
    # q 与 -q 是同一旋转；平滑前统一符号，结果不应绕远路。
    position, quaternion = smooth_pose(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
        (0.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0, 0.0),
        0.5,
    )
    assert position == pytest.approx((0.0, 0.0, 0.0))
    assert quaternion[0] == pytest.approx(1.0)
