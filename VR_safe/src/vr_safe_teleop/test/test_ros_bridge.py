"""ROS 级 dry-run 冒烟测试：心跳、LJ→ESTOP、Y→clear_estop。

不连接 Quest、不初始化 SDK；只需要 ROS 2 + 已构建/已安装的
``vr_safe_interfaces``。没有 ROS 环境时本测试在调用时跳过，纯 Python 测试
仍会正常收集和执行。
"""

import json
import time

import pytest

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import Joy
    from std_srvs.srv import Trigger

    from vr_safe_interfaces.msg import SafetyObservation, TeleopHeartbeat
    from vr_safe_teleop.bridge_node import VrSafeTeleopBridge

    _ROS_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover - 仅在无 ROS/未构建接口时触发
    _ROS_IMPORT_ERROR = exc


def test_dry_run_bridge_publishes_heartbeat_estop_and_clear(tmp_path):
    if _ROS_IMPORT_ERROR is not None:
        pytest.skip(
            "需要 ROS 2 与已构建的 vr_safe_interfaces：%s" % _ROS_IMPORT_ERROR
        )

    frames_path = tmp_path / "teleop_frames.json"
    frames_path.write_text(json.dumps({
        "frames": [
            {
                "t": 0.0,
                "left": {
                    "pose": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
                    "grip": 0.8,
                    "trigger": 0.4,
                    "X": True,
                },
                "right": {},
            },
            {"t": 0.3, "left": {"axis_click": True}},
            {"t": 0.6, "left": {"axis_click": False, "Y": True}},
            {"t": 0.9, "left": {"Y": False}},
        ]
    }), encoding="utf-8")

    if not rclpy.ok():
        rclpy.init()
    probe = rclpy.create_node("vr_safe_teleop_test_probe")
    bridge = None
    executor = SingleThreadedExecutor()
    heartbeats = []
    joys = []
    observations = []
    clear_requests = []
    try:
        probe.create_subscription(
            TeleopHeartbeat, "/vr_safe/teleop/heartbeat", heartbeats.append, 10
        )
        probe.create_subscription(
            SafetyObservation, "/vr_safe/observation", observations.append, 10
        )
        probe.create_subscription(
            Joy, "/vr_safe/teleop/button", joys.append, 10
        )

        def on_clear(_request, response):
            clear_requests.append(_request)
            response.success = True
            response.message = "测试服务：按钮锁存已清除"
            return response

        probe.create_service(Trigger, "/vr_safe/clear_estop", on_clear)

        bridge = VrSafeTeleopBridge(parameter_overrides=[
            Parameter("dry_run", value=True),
            Parameter("dry_run_input", value=str(frames_path)),
            Parameter("dry_run_loop", value=True),
            Parameter("tracking_warn_period_s", value=0.5),
            Parameter("heartbeat_rate_hz", value=20.0),
        ])
        executor.add_node(bridge)
        executor.add_node(probe)

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            got_estop = any(
                message.state == SafetyObservation.ESTOP
                for message in observations
            )
            if (len(heartbeats) >= 5 and len(joys) >= 5
                    and got_estop and clear_requests):
                break
    finally:
        executor.shutdown()
        if bridge is not None:
            bridge.close()
            bridge.destroy_node()
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # 心跳：20 Hz 持续发布，source_id=teleop，seq 单调递增。
    assert len(heartbeats) >= 5
    assert all(message.source_id == "teleop" for message in heartbeats)
    seqs = [message.seq for message in heartbeats]
    assert seqs == sorted(seqs)
    assert len(set(seqs)) == len(seqs)
    assert seqs[0] >= 1

    # Joy：即使内容没变化也按 tick 重复发布，供底盘/arm 看门狗持续喂狗。
    assert len(joys) >= 5
    repeated = any(
        joys[index].axes == joys[index - 1].axes
        and joys[index].buttons == joys[index - 1].buttons
        for index in range(1, len(joys))
    )
    assert repeated, "Joy 内容不变时停止了发布（底盘会触发 200 ms 看门狗）"

    # LJ：一条 source_id=system_estop 的 ESTOP 观察，detail_json 带原因。
    estops = [
        message for message in observations
        if message.state == SafetyObservation.ESTOP
    ]
    assert estops, "没有收到 LJ 触发的 ESTOP 观察"
    estop = estops[0]
    assert estop.source_id == "system_estop"
    assert estop.zone_id == "system"
    assert estop.confidence == pytest.approx(1.0)
    detail = json.loads(estop.detail_json)
    assert detail["button"] == "LJ"
    assert "系统软件急停" in detail["reason"]

    # Y：调用了 /vr_safe/clear_estop，而不是自行发布许可。
    assert clear_requests, "Y 没有调用 clear_estop 服务"
