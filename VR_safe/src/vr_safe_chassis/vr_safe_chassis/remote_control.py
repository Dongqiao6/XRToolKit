"""底盘 ROS 适配：许可订阅、Joy/Twist、遥控使能服务与周期 keepalive。

安全判定全部在 ``gate.ChassisGate`` 里；本文件只负责把 ROS 输入翻译成门调用，
再把门的 SRP 调用落到 ``SrpClient`` 上。这样单元测试不需要 ROS，也不需要真 AMR。

手柄：
  - 左摇杆 → 底盘线速度/角速度（与 demo 相同的死区与符号）；
  - Y（``Joy.buttons[1]``）→ 解除 AMR 急停并重新使能遥控（deadman）；
  - ``estop_button`` 默认 -1（禁用）：X 归夹爪锁定，不能再当底盘急停；
  - LJ / X / A 不在本节点解释；系统软件急停由 teleop 发 ESTOP 观察。
"""

import time

from geometry_msgs.msg import Twist
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool

from vr_safe_interfaces.msg import MotionPermit

from .estop_srv import EStopStateChecker
from .gate import (PERMIT_GRANTED, PERMIT_REVOKED, PERMIT_SLOW)
from .joy import button_pressed, joy_to_twist

_PERMIT_NAMES = {
    MotionPermit.GRANTED: PERMIT_GRANTED,
    MotionPermit.SLOW: PERMIT_SLOW,
    MotionPermit.REVOKED: PERMIT_REVOKED,
}

_OPERATION_MANUAL = 2


class RemoteControlStateChecker:
    """只读 SRP 的遥控/避障使能状态。"""

    def __init__(self, client):
        self._srp_client = client

    def get_remote_control_status(self):
        system_state = self._srp_client.get_current_system_state()
        if system_state is None:
            return None
        try:
            return int(system_state.operation_state) == _OPERATION_MANUAL
        except (TypeError, ValueError):
            return None

    def get_remote_control_oba_status(self):
        system_state = self._srp_client.get_current_system_state()
        if system_state is None:
            return None
        return bool(getattr(system_state, "manual_control_oba_state", False))

    def check_remote_control_status(self, enabled: bool) -> bool:
        return self.get_remote_control_status() == enabled

    def check_remote_control_oba_status(self, enabled: bool) -> bool:
        return self.get_remote_control_oba_status() == enabled


class RemoteController:
    """底盘遥控的 ROS 外壳。"""

    _TIMEOUT_SEC = 5.0
    _RESULT_CODE_HINTS = {
        30001: "CANCEL_EMERGENCY_CAN_NOT_RECOVER（没有可解除的急停）",
        70001: "MANUAL_CONTROL_NOT_ON（底盘尚未进入手动控制）",
        70002: "START_MANUAL_CONTROL_IN_EMERGENCY（急停未解除）",
        70003: "START_MANUAL_CONTROL_IN_BRAKE_SW（抱闸开关未释放）",
        70004: "MANUAL_CONTROL_MOVEMENT_RUNNING（移动任务执行中）",
        70005: "MANUAL_CONTROL_MISSION_RUNNING（mission 执行中）",
    }

    def __init__(self, node: Node, srp_client, gate, config: dict,
                 *, dry_run: bool = False):
        self._node = node
        self._srp_client = srp_client
        self._gate = gate
        self._dry_run = bool(dry_run)
        chassis = config["chassis"]
        topics = config["topics"]

        self._button_topic = topics["button"]
        self._cmd_vel_topic = topics["cmd_vel"]
        self._permit_topic = topics["permit"]
        self._joy_deadzone = float(chassis["joy_deadzone"])
        self._max_linear_speed = float(chassis["max_linear_speed"])
        self._max_angular_speed = float(chassis["max_angular_speed"])
        self._estop_button = int(chassis["estop_button"])
        self._estop_release_button = int(chassis["estop_release_button"])
        self._max_zero_failures = int(chassis["max_zero_failures"])
        self._twist_watchdog_ms = int(chassis["twist_watchdog_ms"])

        self._state_checker = RemoteControlStateChecker(srp_client)
        self._remote_control_enabled = False
        self._estop_button_prev = False
        self._estop_release_button_prev = False

        # 话题、许可、服务、定时器各自一个 callback group：MultithreadedExecutor
        # 下阻塞的 Y/服务调用不会饿死 100 ms 零速 keepalive，也不会挡住许可更新。
        self._topic_group = MutuallyExclusiveCallbackGroup()
        self._permit_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._timer_group = MutuallyExclusiveCallbackGroup()

        self._remote_control_enabled_srv = node.create_service(
            SetBool,
            topics["remote_control_enabled"],
            self._handle_remote_control_enabled,
            callback_group=self._service_group,
        )
        self._remote_control_oba_enabled_srv = node.create_service(
            SetBool,
            topics["remote_control_oba_enabled"],
            self._handle_remote_control_oba_enabled,
            callback_group=self._service_group,
        )

        self._cmd_vel_subscriber = node.create_subscription(
            Twist,
            self._cmd_vel_topic,
            self._on_twist,
            _input_qos(),
            callback_group=self._topic_group,
        )
        self._button_subscriber = node.create_subscription(
            Joy,
            self._button_topic,
            self._on_joy,
            _input_qos(),
            callback_group=self._topic_group,
        )

        permit_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._permit_subscriber = node.create_subscription(
            MotionPermit,
            self._permit_topic,
            self._on_permit,
            permit_qos,
            callback_group=self._permit_group,
        )

        tick_period = float(chassis["gate_tick_period_ms"]) / 1000.0
        self._tick_timer = node.create_timer(
            tick_period, self._on_tick, callback_group=self._timer_group)

        self._last_decision = None
        self._last_state = self._gate.snapshot().state
        self._last_zero_failures = 0
        self._last_watchdog = False
        self._last_escalated = False

        if self._dry_run:
            # dry_run 假定内存遥控已使能且 deadman 已按过：方便无 AMR 时直接
            # 验证许可门/零速；真实死锁恢复语义由单元测试覆盖。
            now = time.monotonic()
            self._remote_control_enabled = True
            self._gate.set_remote_control_enabled(True, now)
            self._gate.on_rearm(now)
            self._node.get_logger().info(
                "dry_run：使用内存 SRP，遥控默认已使能；仍执行许可门与零速 keepalive")

    # ── 订阅回调 ──

    def _on_permit(self, msg: MotionPermit):
        decision = _PERMIT_NAMES.get(msg.decision)
        if decision is None:
            self._node.get_logger().warning(
                f"忽略未知许可 decision={msg.decision}")
            return
        now = time.monotonic()
        before = self._gate.snapshot().state
        self._gate.on_permit(decision, now)
        if self._dry_run and decision != PERMIT_REVOKED:
            # dry_run 没有真人按 Y；自动 arm 只影响内存底盘，真实模式不走这里。
            self._gate.on_rearm(now)
        after = self._gate.snapshot().state
        if decision != self._last_decision or before != after:
            self._node.get_logger().info(
                "许可 %s（reason=%s source=%s）→ 底盘 %s"
                % (decision, msg.reason or "-", msg.source or "-", after))
        self._last_decision = decision

    def _on_joy(self, msg: Joy):
        self._handle_estop_buttons(msg)
        linear_x, linear_y, angular_z = joy_to_twist(
            msg.axes,
            self._joy_deadzone,
            self._max_linear_speed,
            self._max_angular_speed,
        )
        self._gate.on_cmd_vel(linear_x, linear_y, angular_z, time.monotonic())

    def _on_twist(self, msg: Twist):
        # 可选输入：teleop 若按固定频率发 Twist 可以直接用；不依赖它。
        self._gate.on_cmd_vel(
            msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())

    def _on_tick(self):
        self._gate.tick(time.monotonic())
        self._log_transitions()

    # ── 按键 ──

    def _handle_estop_buttons(self, msg: Joy):
        estop_pressed = button_pressed(msg.buttons, self._estop_button)
        release_pressed = button_pressed(msg.buttons, self._estop_release_button)

        if estop_pressed and not self._estop_button_prev:
            self._trigger_emergency_stop()
        if release_pressed and not self._estop_release_button_prev:
            ok, message = self._release_emergency_stop_and_enable()
            if not ok:
                self._node.get_logger().warning(f"Y 解除/使能遥控失败：{message}")

        self._estop_button_prev = estop_pressed
        self._estop_release_button_prev = release_pressed

    def _trigger_emergency_stop(self):
        """本地 AMR 急停键（默认禁用）；系统急停仍走 supervisor。"""
        now = time.monotonic()
        self._remote_control_enabled = False
        self._gate.set_remote_control_enabled(False, now)
        self._gate.on_emergency_stop("local_button", now)
        # 急停前退出遥控，避免解除后重新 enable 命中 ALREADY_IN_MANUAL_CONTROL。
        try:
            self._srp_client.set_remote_control(False)
        except Exception as exc:  # noqa: BLE001 - 硬件边界
            self._node.get_logger().warning(
                f"急停前退出遥控失败：{self._describe_exception(exc)}")
        try:
            self._srp_client.emergency_stop()
            self._node.get_logger().info("本地急停键已触发 AMR emergency_stop")
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(
                f"AMR emergency_stop 失败：{self._describe_exception(exc)}")

    def _release_emergency_stop_and_enable(self):
        """Y：解除 AMR 急停并重新使能遥控；成功即视为 deadman。"""
        estop_active = self._is_emergency_stopped()
        if estop_active is None:
            self._node.get_logger().warning(
                "尚未收到 AMR 急停状态；直接请求手动控制")
        elif estop_active:
            try:
                self._srp_client.release_emergency_stop()
            except Exception as exc:  # noqa: BLE001
                # 30001 可能表示急停已被清除；只有确认仍在急停时才中止。
                if self._is_emergency_stopped() is not False:
                    self._node.get_logger().error(
                        "解除 AMR 急停失败：%s；检查物理急停、安全激光与抱闸"
                        % self._describe_exception(exc))
                    return False, "解除急停失败"
            if not self._wait_until_emergency_released():
                return False, "等待急停解除超时"
        else:
            self._node.get_logger().info(
                "AMR 当前不在急停；直接使能遥控")

        return self._set_remote_control(True)

    def _is_emergency_stopped(self):
        state = self._srp_client.get_current_system_state()
        if state is None:
            return None
        return EStopStateChecker.is_emergency_stopped(state)

    def _wait_until_emergency_released(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        last_state = None
        while time.monotonic() < deadline:
            state = self._srp_client.get_current_system_state()
            if state is not None:
                last_state = getattr(state, "emergency_state", None)
            if self._is_emergency_stopped() is False:
                return True
            time.sleep(0.1)
        self._node.get_logger().error(
            f"等待急停解除超时，emergency_state 停在 {last_state}")
        return False

    # ── 遥控使能服务 ──

    def _set_remote_control(self, target_enable: bool):
        if self._state_checker.check_remote_control_status(target_enable):
            # 已经是目标状态也算显式 deadman：Y 在 HOLD 期间按一次就能重新 arm。
            self._apply_remote_control_state(target_enable)
            return True, f"遥控已处于 {'使能' if target_enable else '禁用'} 状态"

        if not target_enable:
            # 先关许可门再调 SRP，避免关闭过程中 Joy 线程又下发速度。
            self._apply_remote_control_state(False)

        try:
            self._srp_client.set_remote_control(target_enable)
            if target_enable:
                # 遥控时关闭 OBA，避免避障拦截速度；AMR 自带安全仍是主责。
                try:
                    self._srp_client.set_remote_control_oba(False)
                except Exception as exc:  # noqa: BLE001
                    self._node.get_logger().warning(
                        "关闭遥控 OBA 失败（继续使能遥控）：%s"
                        % self._describe_exception(exc))
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(
                "遥控使能调用失败：%s" % self._describe_exception(exc))
            if not target_enable:
                self._apply_remote_control_state(False)  # 失败也要把门关死
            return False, self._describe_exception(exc)

        deadline = time.monotonic() + self._TIMEOUT_SEC
        while time.monotonic() < deadline:
            time.sleep(0.1)
            if self._state_checker.check_remote_control_status(target_enable):
                self._apply_remote_control_state(target_enable)
                self._node.get_logger().info(
                    f"遥控状态已切换为 {target_enable}")
                return True, f"遥控已设为 {target_enable}"
        if not target_enable:
            self._apply_remote_control_state(False)
        message = (
            f"等待遥控状态切换为 {target_enable} 超时"
            f"（{self._TIMEOUT_SEC:.0f}s）"
        )
        self._node.get_logger().error(message)
        return False, message

    def _apply_remote_control_state(self, enabled: bool):
        now = time.monotonic()
        self._remote_control_enabled = bool(enabled)
        self._gate.set_remote_control_enabled(bool(enabled), now)
        if enabled:
            # 使能成功就是 deadman 动作；许可门仍可能因摇杆未回中保持零速。
            self._gate.on_rearm(now)

    def _handle_remote_control_enabled(self, request, response):
        response.success, response.message = self._set_remote_control(
            bool(request.data))
        return response

    def _handle_remote_control_oba_enabled(self, request, response):
        target_enable = bool(request.data)
        if self._state_checker.check_remote_control_oba_status(target_enable):
            response.success = True
            response.message = f"遥控 OBA 已处于 {target_enable}"
            return response
        try:
            self._srp_client.set_remote_control_oba(target_enable)
        except Exception as exc:  # noqa: BLE001
            message = self._describe_exception(exc)
            self._node.get_logger().error(f"遥控 OBA 调用失败：{message}")
            response.success = False
            response.message = message
            return response

        deadline = time.monotonic() + self._TIMEOUT_SEC
        while time.monotonic() < deadline:
            time.sleep(0.1)
            if self._state_checker.check_remote_control_oba_status(target_enable):
                response.success = True
                response.message = f"遥控 OBA 已设为 {target_enable}"
                return response
        message = f"等待遥控 OBA 切换为 {target_enable} 超时"
        self._node.get_logger().error(message)
        response.success = False
        response.message = message
        return response

    # ── 日志与退出 ──

    def _log_transitions(self):
        snap = self._gate.snapshot()
        if snap.zero_failures > self._last_zero_failures:
            self._node.get_logger().warning(
                "零速下发失败 %d/%d：%s"
                % (snap.zero_failures, self._max_zero_failures,
                   snap.last_error or "未知错误"))
        if snap.escalated and not self._last_escalated:
            self._node.get_logger().error(
                "连续零速失败，已升级 AMR emergency_stop；按 Y 重新使能后再恢复")
        if snap.watchdog_tripped and not self._last_watchdog:
            self._node.get_logger().warning(
                "本地看门狗触发：超过 %d ms 没有新 Twist，底盘已自动零速"
                % self._twist_watchdog_ms)
        if snap.state != self._last_state:
            message = f"底盘状态 {self._last_state} → {snap.state}（{snap.reason}）"
            if snap.state in ("hold", "estop"):
                self._node.get_logger().warning(message)
            else:
                self._node.get_logger().info(message)
        self._last_state = snap.state
        self._last_zero_failures = snap.zero_failures
        self._last_watchdog = snap.watchdog_tripped
        self._last_escalated = snap.escalated

    def shutdown(self):
        now = time.monotonic()
        self._gate.on_shutdown(now)
        if self._remote_control_enabled:
            self._remote_control_enabled = False
            self._gate.set_remote_control_enabled(False, now)
            try:
                self._srp_client.set_remote_control(False)
                self._node.get_logger().info("退出时已关闭 AMR 遥控")
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().warning(
                    "退出时关闭 AMR 遥控失败：%s" % self._describe_exception(exc))

    def _describe_exception(self, exc: Exception) -> str:
        code = getattr(exc, "result_code", None)
        if code is None:
            return f"{type(exc).__name__}: {exc}"
        hint = self._RESULT_CODE_HINTS.get(code)
        return f"{code}" if hint is None else f"{code} {hint}"


def _input_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
