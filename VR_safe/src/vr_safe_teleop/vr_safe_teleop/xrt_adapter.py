"""XRoboToolkit 位姿与手柄映射的纯函数集合。

公开的 XRoboToolkit Python SDK 用 ``[x, y, z, qx, qy, qz, qw]`` 表示手柄位姿；
默认坐标变换沿用旧 RemoteCtrl Quest 读取器和官方 XRoboToolkit 遥操作示例的
头显坐标系 → 机器人坐标系映射。

本模块不 import rclpy，也不 import SDK，便于在没有 Quest 的环境里直接单测。
"""

import math


# XRoboToolkit/OpenXR 头显坐标轴 → JAKA 机器人坐标轴。
DEFAULT_HEADSET_TO_ROBOT = (
    0.0, 0.0, -1.0,
    -1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
)

# Joy.buttons 的固定布局。下游臂/底盘节点按这个下标读取，不能随意调换。
BUTTON_ORDER = (
    "X", "Y", "A", "B", "LThU", "RThU",
    "LJ", "RJ", "LG", "RG", "LTr", "RTr",
)

BUTTON_INDEX = {name: index for index, name in enumerate(BUTTON_ORDER)}

# 常用按键下标单独导出，避免桥和测试里散落魔法数字。
X_BUTTON_INDEX = BUTTON_INDEX["X"]
Y_BUTTON_INDEX = BUTTON_INDEX["Y"]
A_BUTTON_INDEX = BUTTON_INDEX["A"]
B_BUTTON_INDEX = BUTTON_INDEX["B"]
LJ_BUTTON_INDEX = BUTTON_INDEX["LJ"]


def clamp(value, low, high):
    """把数值夹到闭区间 ``[low, high]``。"""
    return max(low, min(high, float(value)))


def _mat_vec(matrix, vector):
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def _mat_mul(a, b):
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )


def _transpose(matrix):
    return tuple(tuple(matrix[col][row] for col in range(3)) for row in range(3))


def _determinant(matrix):
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def matrix_3x3(values):
    """用九个行主序数值构造并校验一个合法旋转矩阵。"""
    if len(values) == 3 and all(len(row) == 3 for row in values):
        values = tuple(value for row in values for value in row)
    values = tuple(float(value) for value in values)
    if len(values) != 9:
        raise ValueError("mapping must contain exactly 9 row-major values")
    matrix = (values[0:3], values[3:6], values[6:9])
    if not all(math.isfinite(value) for value in values):
        raise ValueError("mapping contains a non-finite value")
    if abs(_determinant(matrix) - 1.0) > 1e-4:
        raise ValueError("mapping must be a right-handed rotation matrix")
    transpose_product = _mat_mul(_transpose(matrix), matrix)
    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    if any(
        abs(transpose_product[row][col] - identity[row][col]) > 1e-4
        for row in range(3)
        for col in range(3)
    ):
        raise ValueError("mapping must be orthonormal")
    return matrix


def finite_float(value, default):
    """把值转成有限浮点数；失败或 NaN/Inf 时返回 ``default``。"""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def normalize_quaternion(quaternion):
    """返回 ``(w, x, y, z)`` 顺序的单位四元数。"""
    values = tuple(float(value) for value in quaternion)
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must contain four finite values")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in values)


def quaternion_conjugate(quaternion):
    w, x, y, z = quaternion
    return (w, -x, -y, -z)


def quaternion_multiply(first, second):
    """``(w, x, y, z)`` 四元数的 Hamilton 乘积 ``first * second``。"""
    aw, ax, ay, az = first
    bw, bx, by, bz = second
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quaternion_from_matrix(matrix):
    """把合法 3x3 旋转矩阵转成 ``(w, x, y, z)``。"""
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        return normalize_quaternion((
            0.25 * scale,
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
        ))
    if m00 > m11 and m00 > m22:
        scale = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        return normalize_quaternion((
            (m21 - m12) / scale,
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
        ))
    if m11 > m22:
        scale = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        return normalize_quaternion((
            (m02 - m20) / scale,
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
        ))
    scale = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
    return normalize_quaternion((
        (m10 - m01) / scale,
        (m02 + m20) / scale,
        (m12 + m21) / scale,
        0.25 * scale,
    ))


def quaternion_to_matrix(quaternion):
    """把 ``(w, x, y, z)`` 四元数转成 3x3 旋转矩阵。"""
    w, x, y, z = normalize_quaternion(quaternion)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
        (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
        (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
    )


def raw_pose_is_valid(raw_pose):
    """拒绝控制器尚未被跟踪时 SDK 返回的全零哨兵值。"""
    try:
        values = tuple(float(value) for value in raw_pose)
    except (TypeError, ValueError):
        return False
    if len(values) < 7 or not all(math.isfinite(value) for value in values[:7]):
        return False
    quaternion_norm = math.sqrt(sum(value * value for value in values[3:7]))
    return quaternion_norm > 0.5


def transform_pose(
    raw_pose,
    mapping=DEFAULT_HEADSET_TO_ROBOT,
    scale=1.0,
    base_translation=(0.0, 0.0, 0.0),
    base_rotation=(1.0, 0.0, 0.0, 0.0),
    yaw_offset=0.0,
):
    """把 XRoboToolkit 位姿映射到机器人 base 系，不做滤波。

    ``yaw_offset`` 让映射后的位姿绕机器人 base z 轴再旋转。XRoboToolkit 的
    跟踪系在头显会话开始时锚定，朝向不一定与机器人一致：如果头显启动时朝向
    相反，所有手部运动都会转 180°（前后、左右互换，上下不变）。用 ``math.pi``
    可修正该情况，其他角度用于不同朝向。
    """
    if not raw_pose_is_valid(raw_pose):
        raise ValueError("invalid or untracked controller pose")
    matrix = matrix_3x3(mapping)
    position = tuple(float(value) * float(scale) for value in raw_pose[0:3])
    source_quaternion = normalize_quaternion(
        (raw_pose[6], raw_pose[3], raw_pose[4], raw_pose[5])
    )

    mapped_position = _mat_vec(matrix, position)
    mapping_quaternion = quaternion_from_matrix(matrix)
    mapped_quaternion = quaternion_multiply(
        quaternion_multiply(mapping_quaternion, source_quaternion),
        quaternion_conjugate(mapping_quaternion),
    )

    if yaw_offset:
        cos_yaw = math.cos(yaw_offset)
        sin_yaw = math.sin(yaw_offset)
        x, y, z = mapped_position
        mapped_position = (
            cos_yaw * x - sin_yaw * y,
            sin_yaw * x + cos_yaw * y,
            z,
        )
        yaw_quaternion = (
            math.cos(0.5 * yaw_offset),
            0.0,
            0.0,
            math.sin(0.5 * yaw_offset),
        )
        mapped_quaternion = quaternion_multiply(
            quaternion_multiply(yaw_quaternion, mapped_quaternion),
            quaternion_conjugate(yaw_quaternion),
        )

    base_matrix = quaternion_to_matrix(base_rotation)
    rotated_position = _mat_vec(base_matrix, mapped_position)
    output_position = tuple(
        rotated_position[index] + float(base_translation[index])
        for index in range(3)
    )
    output_quaternion = quaternion_multiply(
        normalize_quaternion(base_rotation),
        mapped_quaternion,
    )
    return output_position, normalize_quaternion(output_quaternion)


def smooth_pose(previous, position, quaternion, alpha):
    """沿用旧 Quest 读取器的一阶低通行为。"""
    alpha = clamp(alpha, 0.0, 1.0)
    quaternion = normalize_quaternion(quaternion)
    if previous is None or alpha <= 0.0:
        return tuple(position), quaternion

    previous_position, previous_quaternion = previous
    # 四元数 q 与 -q 表示同一旋转；先统一符号，避免插值绕远路。
    if sum(a * b for a, b in zip(previous_quaternion, quaternion)) < 0.0:
        quaternion = tuple(-value for value in quaternion)
    filtered_position = tuple(
        old + alpha * (new - old)
        for old, new in zip(previous_position, position)
    )
    filtered_quaternion = normalize_quaternion(tuple(
        old + alpha * (new - old)
        for old, new in zip(previous_quaternion, quaternion)
    ))
    return filtered_position, filtered_quaternion


def axis_pair(value):
    """把摇杆输入夹到 ``[-1, 1]``；非法输入按回中处理。"""
    try:
        return (clamp(value[0], -1.0, 1.0), clamp(value[1], -1.0, 1.0))
    except (IndexError, TypeError, ValueError):
        return (0.0, 0.0)


def joy_payload(left, right, grip_threshold=0.5):
    """生成下游 JAKA / AMR 节点消费的固定 Joy 布局。

    ``axes``：[左 Grip, 右 Grip, 左 Trigger, 右 Trigger, 左摇杆 x/y, 右摇杆 x/y]。
    ``buttons`` 按 :data:`BUTTON_ORDER` 排列。LG/RG/LTr/RTr 是超过阈值的数字量，
    方便只认数字按钮的下游节点。
    """
    left_grip = clamp(left.get("grip", 0.0), 0.0, 1.0)
    right_grip = clamp(right.get("grip", 0.0), 0.0, 1.0)
    left_trigger = clamp(left.get("trigger", 0.0), 0.0, 1.0)
    right_trigger = clamp(right.get("trigger", 0.0), 0.0, 1.0)
    left_axis = axis_pair(left.get("axis", (0.0, 0.0)))
    right_axis = axis_pair(right.get("axis", (0.0, 0.0)))
    axes = [
        left_grip,
        right_grip,
        left_trigger,
        right_trigger,
        left_axis[0],
        left_axis[1],
        right_axis[0],
        right_axis[1],
    ]

    button_values = {
        "X": left.get("X", False),
        "Y": left.get("Y", False),
        "A": right.get("A", False),
        "B": right.get("B", False),
        "LThU": left.get("menu", False),
        "RThU": right.get("menu", False),
        "LJ": left.get("axis_click", False),
        "RJ": right.get("axis_click", False),
        "LG": left_grip > grip_threshold,
        "RG": right_grip > grip_threshold,
        "LTr": left_trigger > grip_threshold,
        "RTr": right_trigger > grip_threshold,
    }
    buttons = [1 if button_values[name] else 0 for name in BUTTON_ORDER]
    return axes, buttons
