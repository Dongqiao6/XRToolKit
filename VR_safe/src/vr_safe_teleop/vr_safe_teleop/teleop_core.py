"""遥操作桥的纯逻辑核心。

这个模块不 import rclpy、不 import XRoboToolkit SDK：输入一帧原始手柄数据，
输出目标位姿、Joy、按键事件和心跳序号。把语义集中在这里，就可以在没有
Quest、没有 ROS 的机器上用 pytest 直接验证；ROS 节点只负责读 SDK、发消息
和调用服务。

安全语义（与 ADR 0006/0007 一致）：

- ``LJ`` 上升沿产生一次系统软件急停事件，由节点发 ``ESTOP`` 观察；
- ``Y`` 上升沿产生一次“清按钮锁存”请求，由节点调 ``/vr_safe/clear_estop``；
- ``X``/``A`` 只作为 Joy 按钮透传，夹爪锁定由下游臂节点解释；
- 任一控制器跟踪丢失时该路目标停止发布、Grip/摇杆归零；``X``/``A``/``B`` 也
  回到未按下，``LJ``/``Y`` 这类安全按键仍然有效；心跳由节点的独立定时器发布，
  不受跟踪状态影响。
"""

from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from .xrt_adapter import (
    BUTTON_INDEX,
    BUTTON_ORDER,
    DEFAULT_HEADSET_TO_ROBOT,
    axis_pair,
    clamp,
    finite_float,
    joy_payload,
    matrix_3x3,
    normalize_quaternion,
    raw_pose_is_valid,
    smooth_pose,
    transform_pose,
)

Pose = Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]

_BUTTON_FIELDS = ("X", "Y", "A", "B", "menu", "axis_click")


@dataclass(frozen=True)
class TeleopFrame:
    """一帧桥输出。``targets`` 中 ``None`` 表示这一路本帧不发布目标。"""

    targets: Dict[str, Optional[Pose]]
    tracked: Dict[str, bool]
    joy_axes: Tuple[float, ...]
    joy_buttons: Tuple[int, ...]
    # 仅用于诊断/测试：ROS 节点必须每个 tick 都发 Joy，不能用它做发布门控。
    joy_changed: bool
    estop_pressed: bool = False
    estop_reason: str = ""
    clear_estop_pressed: bool = False
    clear_estop_reason: str = ""
    confirm_pressed: bool = False
    confirm_reason: str = ""
    events: Tuple[str, ...] = ()


class TeleopCore:
    """把原始 Quest 控制器状态翻译成桥输出。"""

    def __init__(
        self,
        mapping=DEFAULT_HEADSET_TO_ROBOT,
        smooth_alpha=0.4,
        pose_scale=1.0,
        grip_threshold=0.5,
        enable_left=True,
        enable_right=True,
        base_translation=(0.0, 0.0, 0.0),
        base_rotation=(1.0, 0.0, 0.0, 0.0),
        estop_button="LJ",
        clear_estop_button="Y",
        dual_arm_button="B",
        confirm_button="RJ",
        confirm_hold_s=1.5,
    ):
        if not enable_left and not enable_right:
            raise ValueError("至少启用一侧控制器（enable_left / enable_right）")
        if float(pose_scale) <= 0.0:
            raise ValueError("pose_scale 必须为正数")

        self.mapping = matrix_3x3(mapping)
        self.smooth_alpha = clamp(smooth_alpha, 0.0, 1.0)
        self.pose_scale = float(pose_scale)
        self.grip_threshold = clamp(grip_threshold, 0.0, 1.0)
        self.enable_left = bool(enable_left)
        self.enable_right = bool(enable_right)
        self.base_translation = tuple(float(value) for value in base_translation)
        if len(self.base_translation) != 3:
            raise ValueError("base_translation 必须包含 3 个数值")
        self.base_rotation = normalize_quaternion(base_rotation)
        self.estop_button = str(estop_button)
        self.clear_estop_button = str(clear_estop_button)
        self.dual_arm_button = str(dual_arm_button)
        self.confirm_button = str(confirm_button)
        self.confirm_hold_s = float(confirm_hold_s)
        if self.confirm_hold_s <= 0.2:
            raise ValueError("confirm_hold_s 必须大于 0.2")
        self._estop_index = self._button_index(self.estop_button)
        self._clear_index = self._button_index(self.clear_estop_button)
        self._dual_arm_index = self._button_index(self.dual_arm_button)
        self._confirm_index = self._button_index(self.confirm_button)

        self._filtered: Dict[str, Optional[Pose]] = {"left": None, "right": None}
        self._pose_valid = {"left": False, "right": False}
        self._last_valid_trigger = {"left": 0.0, "right": 0.0}
        self._last_axes = None
        self._last_buttons = None
        self._dual_arm_mode = False
        self._prev_dual_arm_button = False
        self._prev_estop = False
        self._prev_clear_estop = False
        self._prev_confirm = False
        self._confirm_since = None
        self._confirm_fired = False
        self._last_update_s = 0.0
        self._heartbeat_seq = 0

    # ── 对外查询 ──
    def next_heartbeat_seq(self) -> int:
        """返回下一个心跳序号（从 1 开始，单调递增）。"""
        self._heartbeat_seq += 1
        return self._heartbeat_seq

    @property
    def dual_arm_mode(self) -> bool:
        return self._dual_arm_mode

    @property
    def heartbeat_seq(self) -> int:
        return self._heartbeat_seq

    # ── 主入口 ──
    def update(self, left: Any, right: Any, yaw_offset: float = 0.0,
               now_s: Optional[float] = None) -> TeleopFrame:
        """消费一帧原始控制器状态，生成目标、Joy 和按键事件。

        ``now_s`` 只用于 RJ 长按计时；不传时按 125 Hz 递增，方便纯逻辑测试。
        原始状态字段与 SDK getter 同名：``pose``（7 元组）、``grip``、
        ``trigger``、``axis``、``X``/``Y``/``A``/``B``/``menu``/``axis_click``。
        缺失或非法字段按未跟踪/回中/未按下处理。
        """
        if now_s is None:
            now_s = self._last_update_s + (1.0 / 125.0)
        now_s = float(now_s)
        self._last_update_s = now_s
        events = []
        left_state = (
            self._normalize("left", left) if self.enable_left
            else self._neutral_state()
        )
        right_state = (
            self._normalize("right", right) if self.enable_right
            else self._neutral_state()
        )

        b_pressed = bool(right_state["B"])
        if b_pressed and not self._prev_dual_arm_button:
            self._dual_arm_mode = not self._dual_arm_mode
            # 模式切换后左臂平滑器要从头锚定，避免把上一个控制源的位姿拉过来。
            self._filtered["left"] = None
            events.append(
                "dual_arm_enabled" if self._dual_arm_mode else "dual_arm_disabled"
            )
        self._prev_dual_arm_button = b_pressed

        targets: Dict[str, Optional[Pose]] = {"left": None, "right": None}
        if self._dual_arm_mode:
            # 双臂协同：右手柄同时驱动两臂；两臂仍各自保持独立的滤波状态。
            if self.enable_right:
                targets["right"] = self._target(
                    "right", right_state, yaw_offset, events
                )
            if self.enable_left:
                targets["left"] = self._target(
                    "left", right_state, yaw_offset, events
                )
        else:
            if self.enable_left:
                targets["left"] = self._target(
                    "left", left_state, yaw_offset, events
                )
            if self.enable_right:
                targets["right"] = self._target(
                    "right", right_state, yaw_offset, events
                )

        # 双臂模式下左臂跟随右 Grip，离合一起动作；左 Trigger 仍控制左夹爪。
        joy_left = left_state
        if self._dual_arm_mode:
            joy_left = dict(left_state)
            joy_left["grip"] = right_state["grip"]

        axes, buttons = joy_payload(joy_left, right_state, self.grip_threshold)
        axes = tuple(float(value) for value in axes)
        buttons = tuple(int(value) for value in buttons)
        joy_changed = (
            self._last_axes is None
            or self._last_buttons is None
            or any(
                abs(current - previous) > 1e-6
                for current, previous in zip(axes, self._last_axes)
            )
            or buttons != self._last_buttons
        )
        self._last_axes = axes
        self._last_buttons = buttons

        estop_now = bool(buttons[self._estop_index])
        clear_now = bool(buttons[self._clear_index])
        estop_pressed = estop_now and not self._prev_estop
        clear_pressed = clear_now and not self._prev_clear_estop
        self._prev_estop = estop_now
        self._prev_clear_estop = clear_now

        confirm_now = bool(buttons[self._confirm_index])
        confirm_pressed = False
        confirm_reason = ""
        if confirm_now and not self._prev_confirm:
            self._confirm_since = now_s
            self._confirm_fired = False
        if (confirm_now and self._confirm_since is not None
                and not self._confirm_fired
                and (now_s - self._confirm_since) >= self.confirm_hold_s):
            confirm_pressed = True
            self._confirm_fired = True
            confirm_reason = ("%s 长按 %.1fs：请求人工确认恢复许可"
                              % (self.confirm_button, self.confirm_hold_s))
        if not confirm_now:
            self._confirm_since = None
            self._confirm_fired = False
        self._prev_confirm = confirm_now

        estop_reason = ""
        clear_reason = ""
        if estop_pressed:
            # 急停优先：同一帧同时出现 LJ 与 Y 时，不允许“刚按停就清锁存”。
            estop_reason = "%s 左摇杆按下：系统软件急停" % self.estop_button
            if clear_pressed:
                clear_pressed = False
                events.append("clear_estop_ignored_while_estop")
            confirm_pressed = False
            confirm_reason = ""
        elif clear_pressed:
            clear_reason = "%s 按下：请求解除系统急停按钮锁存" % self.clear_estop_button

        return TeleopFrame(
            targets=targets,
            tracked={
                "left": bool(self._pose_valid["left"]),
                "right": bool(self._pose_valid["right"]),
            },
            joy_axes=axes,
            joy_buttons=buttons,
            joy_changed=joy_changed,
            estop_pressed=estop_pressed,
            estop_reason=estop_reason,
            clear_estop_pressed=clear_pressed,
            clear_estop_reason=clear_reason,
            confirm_pressed=confirm_pressed,
            confirm_reason=confirm_reason,
            events=tuple(events),
        )

    # ── 内部 ──
    @staticmethod
    def _button_index(name: str) -> int:
        try:
            return BUTTON_INDEX[str(name)]
        except KeyError as exc:
            raise ValueError(
                "未知按键 %r；可选：%s" % (name, ", ".join(BUTTON_ORDER))
            ) from exc

    @staticmethod
    def _neutral_state() -> Dict[str, Any]:
        state: Dict[str, Any] = {
            "pose": (),
            "grip": 0.0,
            "trigger": 0.0,
            "axis": (0.0, 0.0),
        }
        state.update({name: False for name in _BUTTON_FIELDS})
        return state

    def _normalize(self, side: str, raw: Any) -> Dict[str, Any]:
        """把 SDK/合成输入规整成内部状态；跟踪丢失时安全侧归零。"""
        if not isinstance(raw, dict):
            raw = {}
        pose = raw.get("pose", ())
        pose_valid = raw_pose_is_valid(pose)

        # Trigger 沿用 demo 行为：跟踪丢失时保持最后一个有效值，不命令夹爪变化。
        trigger_raw = finite_float(raw.get("trigger"), None)
        trigger = (
            self._last_valid_trigger[side]
            if trigger_raw is None
            else clamp(trigger_raw, 0.0, 1.0)
        )
        if pose_valid:
            self._last_valid_trigger[side] = trigger
        else:
            trigger = self._last_valid_trigger[side]

        grip = (
            clamp(finite_float(raw.get("grip"), 0.0), 0.0, 1.0)
            if pose_valid else 0.0
        )
        axis = axis_pair(raw.get("axis", (0.0, 0.0))) if pose_valid else (0.0, 0.0)

        state: Dict[str, Any] = {
            "pose": pose,
            "pose_valid": pose_valid,
            "grip": grip,
            "trigger": trigger,
            "axis": axis,
        }
        for name in _BUTTON_FIELDS:
            state[name] = bool(raw.get(name, False)) if pose_valid else False
        if not pose_valid:
            # 安全按键不依赖位姿跟踪：位姿丢了，操作员仍要能按 LJ 停、按 Y 清锁存。
            # X/A/B 仍然要求跟踪有效，避免用无效手柄状态误切夹爪或双臂模式。
            state["Y"] = bool(raw.get("Y", False))
            state["axis_click"] = bool(raw.get("axis_click", False))
        return state

    def _target(self, side, state, yaw_offset, events):
        if not state["pose_valid"]:
            if self._pose_valid[side]:
                self._filtered[side] = None
                self._pose_valid[side] = False
                events.append("%s_tracking_lost" % side)
            return None

        position, quaternion = transform_pose(
            state["pose"],
            mapping=self.mapping,
            scale=self.pose_scale,
            base_translation=self.base_translation,
            base_rotation=self.base_rotation,
            yaw_offset=yaw_offset,
        )
        filtered = smooth_pose(
            self._filtered[side], position, quaternion, self.smooth_alpha
        )
        self._filtered[side] = filtered
        if not self._pose_valid[side]:
            self._pose_valid[side] = True
            events.append("%s_tracking_acquired" % side)
        return filtered
