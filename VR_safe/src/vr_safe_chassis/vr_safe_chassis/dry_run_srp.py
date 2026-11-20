"""dry_run 用的内存 SRP 替身：不连接真 AMR，不 import sros_sdk_py。

它只实现底盘节点用到的那部分 ``SrpClient`` 接口，并维护一份最小 SystemState /
HardwareState 快照。状态字段用 ``SimpleNamespace`` 模拟 protobuf 子消息，数值
遵循 ``sros_sdk_py.main_pb2`` 的枚举（location/battery 的映射见
``state_publisher.py``）。速度单位与真 SRP 一致：mm/s、mrad/s。
"""

from types import SimpleNamespace

# 复制自 sros_sdk_py.main_pb2 的最小枚举常量（只读、不 import protobuf）。
_OPERATION_MANUAL = 2
_EMERGENCY_NONE = 1
_EMERGENCY_TRIGGER = 2
_LOCATION_STATE_RUNNING = 3
_SYS_STATE_IDLE = 2
_MT_NA = 0
_BATTERY_NO_CHARGING = 3


class DryRunSrpClient:
    """内存版 SrpClient；记录所有下发的速度，便于 dry-run 断言。"""

    def __init__(self):
        self.connected = False
        # dry_run 假定内存遥控已使能，方便无 AMR 时直接验证许可门与零速。
        self.remote_control_enabled = True
        self.remote_control_oba_enabled = True
        self.speed_commands = []
        self.emergency_stop_calls = 0
        self._system_state_callbacks = []
        self._hardware_state_callbacks = []

        self._system_state = SimpleNamespace(
            location_pose=SimpleNamespace(
                x=0, y=0, z=0, roll=0, pitch=0, yaw=0, confidence=100
            ),
            mc_state=SimpleNamespace(state=0, v_x=0, v_y=0, w=0),
            movement_state=SimpleNamespace(state=_MT_NA),
            sys_state=_SYS_STATE_IDLE,
            location_state=_LOCATION_STATE_RUNNING,
            emergency_state=_EMERGENCY_NONE,
            operation_state=_OPERATION_MANUAL,
            manual_control_oba_state=True,
            map_name="dry_run",
            station_no=0,
        )
        self._hardware_state = SimpleNamespace(
            battery_state=_BATTERY_NO_CHARGING,
            battery_percentage=100,
            battery_voltage=48000,
            battery_current=0,
            battery_temperature=25,
            battery_remain_capacity=10000,
            battery_nominal_capacity=10000,
            battery_use_cycles=0,
            battery_remain_time=600,
            battery_sn=[],
        )
    # ── 连接 ──

    def connect(self, config=None) -> bool:
        self.connected = True
        return True

    def disconnect(self):
        self.connected = False

    def is_connected(self) -> bool:
        return self.connected

    # ── 状态回调 ──

    def add_system_state_callback(self, callback):
        self._system_state_callbacks.append(callback)

    def add_hardware_state_callback(self, callback):
        self._hardware_state_callbacks.append(callback)

    def fetch_system_state(self):
        for callback in list(self._system_state_callbacks):
            callback(self._system_state)

    def fetch_hardware_state(self):
        for callback in list(self._hardware_state_callbacks):
            callback(self._hardware_state)

    def get_current_system_state(self):
        return self._system_state

    def get_current_hardware_state(self):
        return self._hardware_state

    # ── 遥控 ──

    def set_remote_control(self, enabled: bool):
        self.remote_control_enabled = bool(enabled)
        self._system_state.operation_state = (
            _OPERATION_MANUAL if enabled else 0
        )

    def set_remote_control_oba(self, enabled: bool):
        self.remote_control_oba_enabled = bool(enabled)
        self._system_state.manual_control_oba_state = bool(enabled)

    def set_remote_control_speed(self, linear_x_mm: int, linear_y_mm: int,
                                 angular_mrad: int):
        command = (int(linear_x_mm), int(linear_y_mm), int(angular_mrad))
        self.speed_commands.append(command)
        self._system_state.mc_state.v_x = command[0]
        self._system_state.mc_state.v_y = command[1]
        self._system_state.mc_state.w = command[2]

    def emergency_stop(self):
        self.emergency_stop_calls += 1
        self._system_state.emergency_state = _EMERGENCY_TRIGGER

    def release_emergency_stop(self):
        self._system_state.emergency_state = _EMERGENCY_NONE

    # ── 测试辅助 ──

    @property
    def last_speed_command(self):
        return self.speed_commands[-1] if self.speed_commands else None

    def zero_command_count(self) -> int:
        return sum(1 for command in self.speed_commands if command == (0, 0, 0))
