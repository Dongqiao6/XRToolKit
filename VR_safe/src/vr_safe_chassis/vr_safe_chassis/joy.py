"""手柄输入映射：左摇杆 → 底盘速度，按键下标 → 是否按下。

纯函数，不依赖 ROS，方便单元测试。Joy 布局与 ``vr_safe_teleop`` 及 demo 的
``xrt_adapter.BUTTON_ORDER`` 保持一致：

    axes = [LG, RG, LTr, RTr, left_x, left_y, right_x, right_y]
    buttons = [X, Y, A, B, LThU, RThU, LJ, RJ, LG, RG, LTr, RTr]

X/A 归夹爪锁定，LJ 归 teleop 的系统软件急停；底盘只使用左摇杆与 Y。
"""

import math

BUTTON_ORDER = (
    "X", "Y", "A", "B", "LThU", "RThU",
    "LJ", "RJ", "LG", "RG", "LTr", "RTr",
)

# 与 demo remote_control._joy_to_twist 一致：左摇杆 y 前后、x 转向。
_LEN_MIN_AXES = 6


def button_pressed(buttons, index: int) -> bool:
    """按下标读 Joy 按键；``-1`` 表示禁用，绝不能落到 ``buttons[-1]``。"""
    if index is None or index < 0 or index >= len(buttons):
        return False
    return bool(buttons[index])


def joy_to_twist(axes, joy_deadzone: float, max_linear_speed: float,
                 max_angular_speed: float):
    """把左摇杆映射成 ``(linear_x, linear_y, angular_z)``。

    死区外按 ``(dominant - deadzone) / (1 - deadzone)`` 重新归一化，因此全杆
    行程仍能到速度上限；取绝对值较大的轴，避免斜推时同时给线速度和角速度。
    """
    if len(axes) < _LEN_MIN_AXES:
        return (0.0, 0.0, 0.0)
    try:
        left_x = float(axes[4])
        left_y = float(axes[5])
    except (TypeError, ValueError):
        return (0.0, 0.0, 0.0)
    if not math.isfinite(left_x) or not math.isfinite(left_y):
        return (0.0, 0.0, 0.0)

    dominant = max(abs(left_x), abs(left_y))
    if dominant <= joy_deadzone:
        return (0.0, 0.0, 0.0)

    span = max(1e-6, 1.0 - joy_deadzone)
    scale = min(1.0, max(0.0, (dominant - joy_deadzone) / span))
    if abs(left_y) >= abs(left_x):
        linear = scale * max_linear_speed * (1.0 if left_y >= 0.0 else -1.0)
        return (linear, 0.0, 0.0)
    angular = scale * max_angular_speed * (-1.0 if left_x >= 0.0 else 1.0)
    return (0.0, 0.0, angular)
