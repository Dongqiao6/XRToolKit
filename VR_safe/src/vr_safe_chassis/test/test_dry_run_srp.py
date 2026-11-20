"""dry_run 内存 SRP 替身的测试：不接 AMR、不 import sros_sdk_py。"""

from vr_safe_chassis.dry_run_srp import DryRunSrpClient


def test_connect_and_state_callbacks():
    client = DryRunSrpClient()
    assert client.connect(None) is True
    system = []
    hardware = []
    client.add_system_state_callback(system.append)
    client.add_hardware_state_callback(hardware.append)

    client.fetch_system_state()
    client.fetch_hardware_state()
    assert len(system) == 1
    assert len(hardware) == 1
    assert system[0].location_state == 3  # LOCATION_STATE_RUNNING
    assert hardware[0].battery_percentage == 100


def test_speed_commands_update_in_memory_telemetry():
    client = DryRunSrpClient()
    assert client.last_speed_command is None
    client.set_remote_control_speed(120, 0, 180)
    assert client.last_speed_command == (120, 0, 180)
    assert client.zero_command_count() == 0
    assert client.get_current_system_state().mc_state.v_x == 120

    client.set_remote_control_speed(0, 0, 0)
    assert client.zero_command_count() == 1


def test_remote_control_and_emergency_stop_round_trip():
    client = DryRunSrpClient()
    client.set_remote_control(True)
    assert client.get_current_system_state().operation_state == 2
    client.set_remote_control_oba(False)
    assert client.get_current_system_state().manual_control_oba_state is False

    client.emergency_stop()
    assert client.emergency_stop_calls == 1
    assert client.get_current_system_state().emergency_state == 2
    client.release_emergency_stop()
    assert client.get_current_system_state().emergency_state == 1
