"""vr_safe_chassis 组合根：连接 AMR（或 dry-run 内存替身）并装配各部件。

启动失败按约定中止：非 dry_run 且连不上 AMR 时打印明确错误并返回非零退出码，
不静默降级成半套底盘。
"""

import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .config import ConfigError, apply_parameter_overrides, load_config
from .estop_srv import EStopSrv
from .gate import ChassisGate
from .remote_control import RemoteController
from .state_publisher import ChassisStatePublisher, StatePublisher


class ChassisStartupError(RuntimeError):
    """底盘无法按配置启动（缺 SDK、连不上 AMR、参数冲突等）。"""


class ChassisNode(Node):
    def __init__(self):
        super().__init__(
            "vr_safe_chassis",
            automatically_declare_parameters_from_overrides=True,
        )
        if not self.has_parameter("config"):
            self.declare_parameter("config", "")
        config_path = self.get_parameter("config").value
        config_path = "" if config_path is None else str(config_path)

        self.cfg = load_config(config_path)
        self.cfg = apply_parameter_overrides(
            self.cfg, self._collect_parameter_overrides())
        chassis = self.cfg["chassis"]
        topics = self.cfg["topics"]
        self._dry_run = bool(chassis["dry_run"])
        self._last_sync_warning = 0.0

        self._srp_client = self._create_client(chassis)

        self._gate = ChassisGate(
            self._srp_client,
            slow_scale=chassis["slow_scale"],
            zero_keepalive_s=float(chassis["zero_keepalive_ms"]) / 1000.0,
            twist_watchdog_s=float(chassis["twist_watchdog_ms"]) / 1000.0,
            permit_watchdog_s=float(chassis["permit_watchdog_ms"]) / 1000.0,
            max_zero_failures=chassis["max_zero_failures"],
            max_linear_mps=chassis["max_linear_speed"],
            max_angular_rps=chassis["max_angular_speed"],
            speed_resend_s=float(chassis["speed_resend_ms"]) / 1000.0,
        )

        self._remote_controller = RemoteController(
            self, self._srp_client, self._gate, self.cfg,
            dry_run=self._dry_run)
        self._state_publisher = StatePublisher(
            self,
            self._srp_client,
            frame_id=chassis["frame_id"],
            parent_frame_id=chassis["parent_frame_id"],
            odom_frame_id=chassis["odom_frame_id"],
            odom_topic=topics["odom"],
            odom_use_integration=chassis["odom_use_integration"],
            system_state_topic=topics["system_state"],
            battery_state_topic=topics["battery_state"],
        )
        self._estop_srv = EStopSrv(
            self,
            self._srp_client,
            estop_service=topics["emergency_stop"],
            release_service=topics["release_emergency_stop"],
            on_estop=self._on_amr_estop,
            on_release=self._on_amr_release,
        )
        self._chassis_state_publisher = ChassisStatePublisher(
            self,
            self._gate,
            topic=topics["state"],
            rate_hz=chassis["state_publish_rate_hz"],
            frame_id=chassis["frame_id"],
            dry_run=self._dry_run,
        )
        self._sync_timer = self.create_timer(
            float(chassis["sync_state_period_s"]), self._sync_state)

        self.get_logger().info(
            "vr_safe_chassis 已启动：dry_run=%s，permit=%s，button=%s，state=%s，"
            "零速周期=%dms，Twist 看门狗=%dms，slow_scale=%.2f，速度上限="
            "%.2fm/s %.2frad/s"
            % (
                self._dry_run,
                topics["permit"],
                topics["button"],
                topics["state"],
                chassis["zero_keepalive_ms"],
                chassis["twist_watchdog_ms"],
                chassis["slow_scale"],
                chassis["max_linear_speed"],
                chassis["max_angular_speed"],
            ))

    # ── 启动 ──

    def _create_client(self, chassis):
        if self._dry_run:
            from .dry_run_srp import DryRunSrpClient

            client = DryRunSrpClient()
            client.connect(None)
            self.get_logger().info(
                "dry_run：使用内存 SRP 替身，不 import/连接 sros_sdk_py")
            return client

        ip = str(chassis["connect_ip"]).strip()
        if not ip:
            raise ChassisStartupError("chassis.connect_ip 为空，无法连接 AMR")
        try:
            from sros_sdk_py import SrpClient, SrpConnectionConfig
        except ImportError as exc:  # pragma: no cover - 依赖环境
            raise ChassisStartupError(
                "缺少 sros_sdk_py，无法连接真 AMR；无 AMR 测试请设 "
                "chassis.dry_run=true") from exc

        try:
            connection_config = SrpConnectionConfig(ip=ip)
            username = str(chassis["connect_username"] or "")
            passwd = str(chassis["connect_passwd"] or "")
            if username:
                connection_config.username = username
            if passwd:
                connection_config.passwd = passwd

            client = SrpClient()
            connected = client.connect(connection_config)
        except Exception as exc:  # noqa: BLE001 - SDK 异常也按启动失败处理
            raise ChassisStartupError(
                f"连接 AMR {ip} 失败：{type(exc).__name__}: {exc}") from exc

        if not connected:
            raise ChassisStartupError(
                f"无法连接 AMR {ip}；按约定中止启动（检查 IP/网络/账号）")
        self.get_logger().info(f"已连接 AMR {ip}")
        return client

    def _collect_parameter_overrides(self) -> dict:
        """收集显式 ROS 参数；扁平键名与 chassis.yaml 的字段一致。"""
        overrides = {}
        for section in ("chassis", "topics"):
            for key in self.cfg[section]:
                if self.has_parameter(key):
                    overrides[key] = self.get_parameter(key).value
        return overrides

    # ── 周期与回调 ──

    def _sync_state(self):
        try:
            self._srp_client.fetch_system_state()
            self._srp_client.fetch_hardware_state()
        except Exception as exc:  # noqa: BLE001 - 一次同步失败不应杀定时器
            now = time.monotonic()
            if now - self._last_sync_warning >= 5.0:
                self._last_sync_warning = now
                self.get_logger().warning(f"同步 AMR 状态失败：{exc}")

    def _on_amr_estop(self):
        self._gate.on_emergency_stop("service", time.monotonic())
        self.get_logger().warning(
            "AMR 急停服务已生效；底盘保持零速，按 Y 解除急停并重新使能")

    def _on_amr_release(self):
        if not self._gate.on_rearm(time.monotonic()):
            self.get_logger().warning(
                "AMR 急停已解除，但遥控尚未使能；按 Y 或调用 "
                "/vr_safe/chassis/remote_control_enabled 重新使能")

    # ── 退出 ──

    def shutdown(self):
        try:
            self._remote_controller.shutdown()
        except Exception as exc:  # noqa: BLE001 - 退出路径尽力而为
            print(f"关闭遥控失败：{exc}", file=sys.stderr)
        try:
            self._srp_client.disconnect()
        except Exception as exc:  # noqa: BLE001
            print(f"断开 AMR 失败：{exc}", file=sys.stderr)


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = ChassisNode()
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except (ChassisStartupError, ConfigError) as exc:
        print(f"[vr_safe_chassis] 启动失败：{exc}", file=sys.stderr)
        exit_code = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
