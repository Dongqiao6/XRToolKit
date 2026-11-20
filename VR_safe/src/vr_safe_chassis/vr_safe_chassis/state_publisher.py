"""底盘状态发布：SystemState / BatteryState / odom / TF 与 ChassisState。

从 demo ``state_publisher.py`` 复制，去掉 lidar 与 action 相关逻辑；不 import
``sros_sdk_py``，proto 字段按鸭子类型读取，因此 dry_run 的内存状态也能走同一
条发布路径。

数值映射来自 ``sros_sdk_py.main_pb2``：
  - LocationState：ZERO=0、NONE=1、INITIALING=2、RUNNING=3、RELOCATING=4、
    ERROR=5；ROS 消息常量与之相同，未知值映射为 UNKNOWN(0x20)。
  - BatteryState：NA=0、CHARGING=2、NO_CHARGING=3；ROS 消息里 CHARGING=1，
    因此必须显式转换，不能直接把 proto 数值写进消息。
  - MovementTask：MT_NA=0、MT_FINISHED=5。
  - SysState：TASK_NAV_PAUSED=10、TASK_MANUAL_PAUSED=19。
"""

import json
import math
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.publisher import Publisher
from tf2_ros import TransformBroadcaster, TransformStamped

from vr_safe_chassis_interfaces.msg import BatteryState as BatteryStateMsg
from vr_safe_chassis_interfaces.msg import SystemState as SystemStateMsg
from vr_safe_interfaces.msg import ChassisState

from .estop_srv import EStopStateChecker
from .remote_control import RemoteControlStateChecker
from .utils.ros_bridge import srp_to_ros_unit, to_ros_pose

_LOCATION_STATE_TO_MSG = {
    0: SystemStateMsg.LOCATION_STATE_ZERO,
    1: SystemStateMsg.LOCATION_STATE_NONE,
    2: SystemStateMsg.LOCATION_STATE_INITIALING,
    3: SystemStateMsg.LOCATION_STATE_RUNNING,
    4: SystemStateMsg.LOCATION_STATE_RELOCATING,
    5: SystemStateMsg.LOCATION_STATE_ERROR,
}
_PROTO_BATTERY_NA = 0
_PROTO_BATTERY_CHARGING = 2
_PROTO_BATTERY_NO_CHARGING = 3
_PROTO_MT_NA = 0
_PROTO_MT_FINISHED = 5
_PROTO_SYS_TASK_NAV_PAUSED = 10
_PROTO_SYS_TASK_MANUAL_PAUSED = 19


class StatePublisher:
    """把 SRP 回调翻译成 ROS 状态、里程计和 TF。"""

    def __init__(
        self,
        node: Node,
        client,
        *,
        frame_id: str,
        parent_frame_id: str,
        odom_frame_id: str,
        odom_topic: str,
        odom_use_integration: bool,
        system_state_topic: str,
        battery_state_topic: str,
    ):
        self._node = node
        self._srp_client = client
        self._frame_id = frame_id
        self._parent_frame_id = parent_frame_id
        self._odom_frame_id = odom_frame_id
        self._odom_topic = odom_topic
        self._odom_use_integration = bool(odom_use_integration)
        self._remote_control_state_checker = RemoteControlStateChecker(client)
        self._tf_broadcaster = TransformBroadcaster(node)
        self._publish_lock = threading.RLock()
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._last_odom_time_ns = None
        self._odom_initialized = False

        self._sys_state_publisher = node.create_publisher(
            SystemStateMsg, system_state_topic, 10)
        self._battery_state_publisher = node.create_publisher(
            BatteryStateMsg, battery_state_topic, 10)
        self._odom_publisher = node.create_publisher(
            Odometry, odom_topic, 10)

        client.add_system_state_callback(self.publish_sys_state)
        client.add_hardware_state_callback(self.publish_battery_state)

    # ── SystemState ──

    @staticmethod
    def _location_state_to_msg(value) -> int:
        try:
            state = int(value)
        except (TypeError, ValueError):
            return SystemStateMsg.LOCATION_STATE_UNKNOWN
        return _LOCATION_STATE_TO_MSG.get(
            state, SystemStateMsg.LOCATION_STATE_UNKNOWN)

    def publish_sys_state(self, system_state):
        if system_state is None:
            return

        current_pose = to_ros_pose(system_state.location_pose)
        msg = SystemStateMsg()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._parent_frame_id
        msg.current_pose = current_pose
        msg.current_pose.header = msg.header

        msg.remote_control_active = bool(
            self._remote_control_state_checker.get_remote_control_status())
        msg.remote_control_oba_active = bool(
            self._remote_control_state_checker.get_remote_control_oba_status())
        msg.estop_active = bool(
            EStopStateChecker.is_emergency_stopped(system_state))

        msg.current_map_name = str(getattr(system_state, "map_name", ""))
        msg.current_station_id = int(getattr(system_state, "station_no", 0))
        msg.location_confidence_score = max(
            0, min(255, int(getattr(system_state.location_pose, "confidence", 0))))
        msg.location_state = self._location_state_to_msg(
            getattr(system_state, "location_state", -1))

        mc_state = system_state.mc_state
        msg.linear_velocity_x = srp_to_ros_unit(
            float(getattr(mc_state, "v_x", 0.0)))
        msg.linear_velocity_y = srp_to_ros_unit(
            float(getattr(mc_state, "v_y", 0.0)))
        msg.angular_velocity = srp_to_ros_unit(
            float(getattr(mc_state, "w", 0.0)))

        movement_state = getattr(system_state, "movement_state", None)
        movement = int(getattr(movement_state, "state", _PROTO_MT_NA))
        msg.executing_movement_task = movement not in (
            _PROTO_MT_NA, _PROTO_MT_FINISHED)
        sys_state = int(getattr(system_state, "sys_state", 0))
        msg.movement_task_manual_paused = (
            sys_state == _PROTO_SYS_TASK_MANUAL_PAUSED)
        msg.movement_task_obstacle_paused = (
            sys_state == _PROTO_SYS_TASK_NAV_PAUSED)

        if not rclpy.ok():
            self._node.get_logger().debug("ROS context 已失效，跳过 SystemState 发布")
            return
        try:
            # SRP 回调可能来自网络线程；发布和 TF 里程计状态需要串行化。
            with self._publish_lock:
                self._sys_state_publisher.publish(msg)
                self._publish_tf(
                    system_state.location_pose,
                    msg.linear_velocity_x,
                    msg.linear_velocity_y,
                    msg.angular_velocity,
                )
        except Exception as exc:  # noqa: BLE001 - 发布失败不能杀回调线程
            self._node.get_logger().error(f"SystemState 发布失败：{exc}")

    # ── BatteryState ──

    @staticmethod
    def _battery_state_to_msg(value) -> int:
        try:
            state = int(value)
        except (TypeError, ValueError):
            return BatteryStateMsg.STATE_UNKNOWN
        if state == _PROTO_BATTERY_NA:
            return BatteryStateMsg.STATE_NA
        if state == _PROTO_BATTERY_CHARGING:
            return BatteryStateMsg.STATE_CHARING
        if state == _PROTO_BATTERY_NO_CHARGING:
            return BatteryStateMsg.STATE_NO_CHARING
        return BatteryStateMsg.STATE_UNKNOWN

    def publish_battery_state(self, hardware_state):
        if hardware_state is None:
            return
        msg = BatteryStateMsg()
        msg.remaining_percentage = int(
            getattr(hardware_state, "battery_percentage", 0))
        msg.remaining_capacity = int(
            getattr(hardware_state, "battery_remain_capacity", 0))
        msg.remaining_time = int(
            getattr(hardware_state, "battery_remain_time", 0))
        msg.state = self._battery_state_to_msg(
            getattr(hardware_state, "battery_state", -1))
        msg.voltage = int(getattr(hardware_state, "battery_voltage", 0))
        msg.current = int(getattr(hardware_state, "battery_current", 0))
        msg.temperature = int(getattr(hardware_state, "battery_temperature", 0))
        msg.nominal_capacity = int(
            getattr(hardware_state, "battery_nominal_capacity", 0))
        msg.use_cycles = int(getattr(hardware_state, "battery_use_cycles", 0))
        msg.sn = list(getattr(hardware_state, "battery_sn", []))

        if not rclpy.ok():
            self._node.get_logger().debug("ROS context 已失效，跳过 BatteryState 发布")
            return
        try:
            self._battery_state_publisher.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f"BatteryState 发布失败：{exc}")

    # ── odom / TF ──

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quaternion_from_yaw(yaw: float):
        half_yaw = yaw * 0.5
        return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

    @staticmethod
    def _compose_2d(x1, y1, yaw1, x2, y2, yaw2):
        cos_yaw = math.cos(yaw1)
        sin_yaw = math.sin(yaw1)
        return (
            x1 + cos_yaw * x2 - sin_yaw * y2,
            y1 + sin_yaw * x2 + cos_yaw * y2,
            yaw1 + yaw2,
        )

    @staticmethod
    def _inverse_2d(x, y, yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            -(cos_yaw * x + sin_yaw * y),
            -(-sin_yaw * x + cos_yaw * y),
            -yaw,
        )

    def _publish_tf(self, system_pose, vx: float, vy: float, w: float):
        if system_pose is None:
            return

        now = self._node.get_clock().now()
        now_ns = now.nanoseconds
        ros_pose = to_ros_pose(system_pose)
        map_position = ros_pose.pose.position
        map_orientation = ros_pose.pose.orientation
        map_yaw = self._yaw_from_quaternion(
            map_orientation.x,
            map_orientation.y,
            map_orientation.z,
            map_orientation.w,
        )

        if not self._odom_use_integration or not self._odom_initialized:
            self._odom_x = map_position.x
            self._odom_y = map_position.y
            self._odom_yaw = map_yaw
            self._odom_initialized = True
            self._last_odom_time_ns = now_ns
        elif self._last_odom_time_ns is not None and now_ns > self._last_odom_time_ns:
            dt = (now_ns - self._last_odom_time_ns) / 1e9
            cos_yaw = math.cos(self._odom_yaw)
            sin_yaw = math.sin(self._odom_yaw)
            self._odom_x += (vx * cos_yaw - vy * sin_yaw) * dt
            self._odom_y += (vx * sin_yaw + vy * cos_yaw) * dt
            self._odom_yaw += w * dt
            self._last_odom_time_ns = now_ns

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = now.to_msg()
        odom_to_base.header.frame_id = self._odom_frame_id
        odom_to_base.child_frame_id = self._frame_id
        odom_to_base.transform.translation.x = self._odom_x
        odom_to_base.transform.translation.y = self._odom_y
        odom_to_base.transform.translation.z = 0.0
        qx, qy, qz, qw = self._quaternion_from_yaw(self._odom_yaw)
        odom_to_base.transform.rotation.x = qx
        odom_to_base.transform.rotation.y = qy
        odom_to_base.transform.rotation.z = qz
        odom_to_base.transform.rotation.w = qw

        if self._odom_use_integration:
            odom_to_base_inv = self._inverse_2d(
                self._odom_x, self._odom_y, self._odom_yaw)
            map_to_odom = self._compose_2d(
                map_position.x,
                map_position.y,
                map_yaw,
                odom_to_base_inv[0],
                odom_to_base_inv[1],
                odom_to_base_inv[2],
            )
        else:
            map_to_odom = (0.0, 0.0, 0.0)

        map_to_odom_tf = TransformStamped()
        map_to_odom_tf.header.stamp = now.to_msg()
        map_to_odom_tf.header.frame_id = self._parent_frame_id
        map_to_odom_tf.child_frame_id = self._odom_frame_id
        map_to_odom_tf.transform.translation.x = map_to_odom[0]
        map_to_odom_tf.transform.translation.y = map_to_odom[1]
        map_to_odom_tf.transform.translation.z = 0.0
        qx, qy, qz, qw = self._quaternion_from_yaw(map_to_odom[2])
        map_to_odom_tf.transform.rotation.x = qx
        map_to_odom_tf.transform.rotation.y = qy
        map_to_odom_tf.transform.rotation.z = qz
        map_to_odom_tf.transform.rotation.w = qw

        self._tf_broadcaster.sendTransform([map_to_odom_tf, odom_to_base])

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self._odom_frame_id
        odom_msg.child_frame_id = self._frame_id
        odom_msg.pose.pose.position.x = self._odom_x
        odom_msg.pose.pose.position.y = self._odom_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = odom_to_base.transform.rotation.x
        odom_msg.pose.pose.orientation.y = odom_to_base.transform.rotation.y
        odom_msg.pose.pose.orientation.z = odom_to_base.transform.rotation.z
        odom_msg.pose.pose.orientation.w = odom_to_base.transform.rotation.w
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = w
        self._odom_publisher.publish(odom_msg)


class ChassisStatePublisher:
    """按固定频率发布许可门快照，供 supervisor / 日志 / dry-run 观察。"""

    def __init__(self, node: Node, gate, *, topic: str, rate_hz: float,
                 frame_id: str, dry_run: bool):
        self._node = node
        self._gate = gate
        self._frame_id = frame_id
        self._dry_run = bool(dry_run)
        self._publisher: Publisher = node.create_publisher(ChassisState, topic, 10)
        node.create_timer(1.0 / float(rate_hz), self.publish)

    def publish(self):
        snap = self._gate.snapshot()
        msg = ChassisState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.state = snap.state
        msg.command_blocked = bool(snap.command_blocked)
        msg.hold_active = bool(snap.hold_active)
        msg.linear_mps = float(snap.linear_mps)
        msg.angular_rps = float(snap.angular_rps)
        msg.detail_json = json.dumps(
            {
                "dry_run": self._dry_run,
                "permit": snap.permit,
                "reason": snap.reason,
                "armed": bool(snap.armed),
                "remote_control_enabled": bool(snap.remote_control_enabled),
                "stick_centered": bool(snap.stick_centered),
                "zero_failures": int(snap.zero_failures),
                "escalated": bool(snap.escalated),
                "watchdog": bool(snap.watchdog_tripped),
                "last_error": snap.last_error,
                "slow_scale": float(snap.slow_scale),
                "linear_y_mps": float(snap.linear_y_mps),
            },
            ensure_ascii=False,
        )
        try:
            self._publisher.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f"ChassisState 发布失败：{exc}")
