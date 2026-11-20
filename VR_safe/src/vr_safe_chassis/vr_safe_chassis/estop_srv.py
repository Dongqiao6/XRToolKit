"""AMR 急停/解除急停服务（维修/诊断直连路径）。

系统软件急停（Quest LJ）走 teleop → supervisor → REVOKED，不经过这里；本服务
只是保留 demo 的维修入口。服务成功后会回调许可门，让 ChassisState 反映
``estop`` 并阻断速度下发。
"""

import time
from typing import Optional

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger

# 复制自 sros_sdk_py.main_pb2 的急停枚举（只读常量）。
_EMERGENCY_NA = 0
_EMERGENCY_NONE = 1

# 现场排查看得懂的错误码提示。
_RESULT_CODE_HINTS = {
    30001: "CANCEL_EMERGENCY_CAN_NOT_RECOVER（没有可解除的急停）",
    70001: "MANUAL_CONTROL_NOT_ON（底盘尚未进入手动控制）",
    70002: "START_MANUAL_CONTROL_IN_EMERGENCY（急停未解除）",
    70003: "START_MANUAL_CONTROL_IN_BRAKE_SW（抱闸开关未释放）",
    70004: "MANUAL_CONTROL_MOVEMENT_RUNNING（移动任务执行中）",
    70005: "MANUAL_CONTROL_MISSION_RUNNING（mission 执行中）",
}


class EStopStateChecker:
    @staticmethod
    def is_emergency_stopped(sys_state) -> Optional[bool]:
        if sys_state is None:
            return None
        state = int(getattr(sys_state, "emergency_state", _EMERGENCY_NONE))
        return state not in (_EMERGENCY_NA, _EMERGENCY_NONE)


class EStopSrv:
    def __init__(
        self,
        node: Node,
        srp_client,
        *,
        estop_service: str,
        release_service: str,
        on_estop=None,
        on_release=None,
    ):
        self._node = node
        self._srp_client = srp_client
        self._on_estop = on_estop
        self._on_release = on_release
        self._callback_group = ReentrantCallbackGroup()
        node.create_service(
            Trigger, estop_service, self._handle_estop_request,
            callback_group=self._callback_group,
        )
        node.create_service(
            Trigger, release_service, self._handle_release_request,
            callback_group=self._callback_group,
        )

    @staticmethod
    def _describe_result_code(code: int) -> str:
        hint = _RESULT_CODE_HINTS.get(code)
        return f"{code}" if hint is None else f"{code} {hint}"

    @staticmethod
    def _timeout_response(response: Trigger.Response, what: str):
        response.success = False
        response.message = f"{what}失败：等待底盘状态超时"
        return response

    def _handle_estop_request(self, _request, response: Trigger.Response):
        self._node.get_logger().info("收到底盘 emergency_stop 服务请求")
        should_execute = self._should_execute_cmd(True)
        if should_execute is None:
            return self._timeout_response(response, "急停")
        if not should_execute:
            if self._on_estop is not None:
                self._on_estop()
            response.success = True
            response.message = "底盘已处于急停状态"
            return response

        try:
            self._srp_client.emergency_stop()
        except Exception as exc:  # noqa: BLE001 - 硬件边界
            code = getattr(exc, "result_code", None)
            detail = self._describe_result_code(code) if code is not None else str(exc)
            self._node.get_logger().error(f"AMR 急停调用失败：{detail}")
            response.success = False
            response.message = f"急停失败：{detail}"
            return response

        success = self._wait_for_state(expect_estop=True)
        if success is None:
            return self._timeout_response(response, "急停")
        if success and self._on_estop is not None:
            self._on_estop()
        response.success = bool(success)
        response.message = "急停成功" if success else "急停失败：底盘状态未变为急停"
        return response

    def _handle_release_request(self, _request, response: Trigger.Response):
        self._node.get_logger().info("收到底盘 release_emergency_stop 服务请求")
        should_execute = self._should_execute_cmd(False)
        if should_execute is None:
            return self._timeout_response(response, "解除急停")
        if not should_execute:
            if self._on_release is not None:
                self._on_release()
            response.success = True
            response.message = "底盘没有激活的急停，无需解除"
            return response

        try:
            self._srp_client.release_emergency_stop()
        except Exception as exc:  # noqa: BLE001 - 硬件边界
            code = getattr(exc, "result_code", None)
            detail = self._describe_result_code(code) if code is not None else str(exc)
            self._node.get_logger().error(f"AMR 解除急停调用失败：{detail}")
            response.success = False
            response.message = f"解除急停失败：{detail}"
            return response

        success = self._wait_for_state(expect_estop=False)
        if success is None:
            return self._timeout_response(response, "解除急停")
        if success and self._on_release is not None:
            self._on_release()
        response.success = bool(success)
        response.message = "解除急停成功" if success else "解除急停失败：底盘仍在急停"
        return response

    def _should_execute_cmd(self, target: bool, timeout: float = 2.0) -> Optional[bool]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            system_state = self._srp_client.get_current_system_state()
            if system_state is None:
                time.sleep(0.1)
                continue
            active = EStopStateChecker.is_emergency_stopped(system_state)
            return active != target
        return None

    def _wait_for_state(self, expect_estop: bool, timeout: float = 2.0) -> Optional[bool]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            system_state = self._srp_client.get_current_system_state()
            if system_state is None:
                time.sleep(0.1)
                continue
            active = EStopStateChecker.is_emergency_stopped(system_state)
            if expect_estop and active:
                return True
            if not expect_estop and not active:
                return True
            time.sleep(0.1)
        return None
