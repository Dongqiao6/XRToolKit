"""VR_safe Quest 遥操作桥：目标、Joy、心跳与系统软件急停。

与 demo 的 ``xrt_teleop`` 相比，本节点：

- 只发 ``/vr_safe/teleop/*``，不再使用旧的 ``/rc_ctrl/*``；
- 以固定频率向 supervisor 发 ``TeleopHeartbeat``（默认 20 Hz，跟踪丢失也发）；
- 把 ``LJ`` 上升沿翻译成 ``ESTOP`` 观察，把 ``Y`` 上升沿翻译成
  ``/vr_safe/clear_estop`` 服务调用；
- ``X``/``A`` 夹爪锁定与右 ``B`` 双臂模式保持 demo 语义；
- 支持 ``dry_run`` + JSON 合成输入，无需 Quest/PC-service 即可验证。
"""

import json
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from vr_safe_interfaces.msg import MotionPermit, SafetyObservation, TeleopHeartbeat
from vr_safe_interfaces.srv import ConfirmResume

from .quest_source import create_source
from .teleop_core import TeleopCore
from .xrt_adapter import BUTTON_INDEX, DEFAULT_HEADSET_TO_ROBOT

_EVENT_MESSAGES = {
    "left_tracking_acquired": "左控制器跟踪恢复，开始发布左目标",
    "right_tracking_acquired": "右控制器跟踪恢复，开始发布右目标",
    "left_tracking_lost": "左控制器跟踪丢失：停止左目标发布，Grip/摇杆归零",
    "right_tracking_lost": "右控制器跟踪丢失：停止右目标发布，Grip/摇杆归零",
    "dual_arm_enabled": "右 B：双臂协同模式开启（右手柄同时驱动双臂）",
    "dual_arm_disabled": "右 B：双臂协同模式关闭",
    "clear_estop_ignored_while_estop": "同一帧内 LJ 优先，忽略 Y 的解锁请求",
}

_SYSTEM_ESTOP_SOURCE = "system_estop"


class VrSafeTeleopBridge(Node):
    """把 XRoboToolkit 输入翻译成 VR_safe 遥操作契约。"""

    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            "vr_safe_teleop_bridge", parameter_overrides=parameter_overrides
        )
        self._read_parameters()
        self._closed = False
        self._clear_pending = False
        self._last_tracking_warning = 0.0
        self._last_io_warning = 0.0
        self._last_yaw_offset = None

        self._core = TeleopCore(
            mapping=self.headset_to_robot,
            smooth_alpha=self.smooth_alpha,
            pose_scale=self.pose_scale,
            grip_threshold=self.grip_threshold,
            enable_left=self.enable_left,
            enable_right=self.enable_right,
            base_translation=self.tf_translation,
            base_rotation=self.tf_rotation,
            estop_button=self.estop_button,
            clear_estop_button=self.clear_estop_button,
            dual_arm_button=self.dual_arm_button,
            confirm_button=self.confirm_button,
            confirm_hold_s=self.confirm_hold_s,
        )
        # dry_run 路径不 import SDK；真实路径失败时抛出带指引的 RuntimeError。
        self._source = create_source(
            self.dry_run, self.dry_run_input, self.dry_run_loop
        )

        target_qos = _qos(ReliabilityPolicy.RELIABLE)
        observation_qos = _qos(ReliabilityPolicy.RELIABLE)
        heartbeat_qos = _qos(self.heartbeat_reliability)

        self.pub_left = self.create_publisher(
            PoseStamped, self.left_target_topic, target_qos
        )
        self.pub_right = self.create_publisher(
            PoseStamped, self.right_target_topic, target_qos
        )
        self.pub_button = self.create_publisher(
            Joy, self.button_topic, target_qos
        )
        self.pub_heartbeat = self.create_publisher(
            TeleopHeartbeat, self.heartbeat_topic, heartbeat_qos
        )
        self.pub_observation = self.create_publisher(
            SafetyObservation, self.observation_topic, observation_qos
        )
        self.clear_estop_client = self.create_client(
            Trigger, self.clear_estop_service
        )
        self.confirm_client = self.create_client(
            ConfirmResume, self.confirm_service
        )
        self.pub_confirm_status = self.create_publisher(
            String, self.confirm_status_topic, target_qos
        )
        self._confirm_pending = False
        self._confirm_started = 0.0
        self.permit_received = False
        self.permit_decision = None
        self.permit_latched = False
        permit_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            MotionPermit, self.permit_topic, self._on_permit, permit_qos)

        self.create_timer(1.0 / self.publish_rate, self._tick)
        self.create_timer(1.0 / self.heartbeat_rate_hz, self._publish_heartbeat)

        self.get_logger().info(
            "VR_safe 遥操作桥已启动：目标 %.1f Hz，心跳 %.1f Hz（source_id=%s，"
            "QoS=%s）"
            % (self.publish_rate, self.heartbeat_rate_hz,
               self.heartbeat_source_id, self.heartbeat_qos_name)
        )
        self.get_logger().info(
            "话题：left=%s right=%s button=%s heartbeat=%s observation=%s"
            % (self.left_target_topic, self.right_target_topic,
               self.button_topic, self.heartbeat_topic, self.observation_topic)
        )
        self.get_logger().info(
            "按键：LJ(%d)=系统急停，Y(%d)=清按钮锁存，RJ(%d)=长按 %.1fs 确认恢复，"
            "右B(%d)=双臂模式；X(%d)/A(%d)=夹爪锁定由下游臂节点解释"
            % (BUTTON_INDEX[self.estop_button],
               BUTTON_INDEX[self.clear_estop_button],
               BUTTON_INDEX[self.confirm_button], self.confirm_hold_s,
               BUTTON_INDEX[self.dual_arm_button],
               BUTTON_INDEX[self.gripper_left_button],
               BUTTON_INDEX[self.gripper_right_button])
        )
        self.get_logger().info(
            "Quest MAC %s（仅诊断；设备发现由 PC-service 负责）"
            % (self.quest_mac or "（未配置）")
        )
        if self.dry_run:
            self.get_logger().warning(
                "dry_run=true：不连接 Quest SDK，使用合成输入%s"
                % ("（文件 %s）" % self.dry_run_input
                   if self.dry_run_input else "（中性输入）")
            )
        if self.heartbeat_reliability != ReliabilityPolicy.RELIABLE:
            self.get_logger().warning(
                "心跳以 BEST_EFFORT 发布；现有 supervisor 用 RELIABLE 订阅，"
                "会收不到心跳。仅在 supervisor 同步改成 BEST_EFFORT 后使用。"
            )

    # ── 参数 ──
    def _read_parameters(self):
        self.left_target_topic = self._string_parameter(
            "left_target_topic", "/vr_safe/teleop/left_target")
        self.right_target_topic = self._string_parameter(
            "right_target_topic", "/vr_safe/teleop/right_target")
        self.button_topic = self._string_parameter(
            "button_topic", "/vr_safe/teleop/button")
        self.heartbeat_topic = self._string_parameter(
            "heartbeat_topic", "/vr_safe/teleop/heartbeat")
        self.observation_topic = self._string_parameter(
            "observation_topic", "/vr_safe/observation")
        self.clear_estop_service = self._string_parameter(
            "clear_estop_service", "/vr_safe/clear_estop")
        self.confirm_service = self._string_parameter(
            "confirm_service", "/vr_safe/confirm_resume")
        self.confirm_status_topic = self._string_parameter(
            "confirm_status_topic", "/vr_safe/teleop/confirm_status")
        self.permit_topic = self._string_parameter(
            "permit_topic", "/vr_safe/permit")

        for name, value in (
            ("left_target_topic", self.left_target_topic),
            ("right_target_topic", self.right_target_topic),
            ("button_topic", self.button_topic),
            ("heartbeat_topic", self.heartbeat_topic),
            ("observation_topic", self.observation_topic),
            ("clear_estop_service", self.clear_estop_service),
            ("confirm_service", self.confirm_service),
            ("confirm_status_topic", self.confirm_status_topic),
            ("permit_topic", self.permit_topic),
        ):
            _validate_vr_safe_name(name, value)
        if self.left_target_topic == self.right_target_topic:
            raise ValueError("左右目标话题不能相同")

        self.publish_rate = self._positive_float("publish_rate", 125.0)
        self.heartbeat_rate_hz = self._positive_float("heartbeat_rate_hz", 20.0)
        self.heartbeat_source_id = self._string_parameter(
            "heartbeat_source_id", "teleop")
        self.heartbeat_qos_name = self._string_parameter(
            "heartbeat_qos_reliability", "reliable").lower()
        reliability_map = {
            "reliable": ReliabilityPolicy.RELIABLE,
            "best_effort": ReliabilityPolicy.BEST_EFFORT,
        }
        if self.heartbeat_qos_name not in reliability_map:
            raise ValueError(
                "heartbeat_qos_reliability 只能是 reliable 或 best_effort"
            )
        self.heartbeat_reliability = reliability_map[self.heartbeat_qos_name]

        self.smooth_alpha = self._clamped_float("smooth_alpha", 0.4)
        self.pose_scale = self._positive_float("pose_scale", 1.0)
        self.grip_threshold = self._clamped_float("grip_threshold", 0.5)
        self.robot_base_frame = self._string_parameter(
            "robot_base_frame", "world")
        self.headset_to_robot = self._vector_parameter(
            "headset_to_robot", DEFAULT_HEADSET_TO_ROBOT, 9)
        self.tf_translation = self._vector_parameter(
            "tf_translation", (0.0, 0.0, 0.0), 3)
        self.tf_rotation = self._vector_parameter(
            "tf_rotation", (1.0, 0.0, 0.0, 0.0), 4)
        self.declare_parameter(
            "headset_yaw_offset_deg", 0.0, ParameterDescriptor(dynamic_typing=True)
        )

        self.enable_left = self._bool_parameter("enable_left", True)
        self.enable_right = self._bool_parameter("enable_right", True)
        if not self.enable_left and not self.enable_right:
            raise ValueError("enable_left 与 enable_right 不能同时为 false")

        self.estop_button = self._button_parameter("estop_button", "LJ")
        self.clear_estop_button = self._button_parameter(
            "clear_estop_button", "Y")
        self.dual_arm_button = self._button_parameter("dual_arm_button", "B")
        self.confirm_button = self._button_parameter("confirm_button", "RJ")
        self.confirm_hold_s = self._positive_float("confirm_hold_s", 1.5)
        self.gripper_left_button = self._button_parameter(
            "gripper_left_button", "X")
        self.gripper_right_button = self._button_parameter(
            "gripper_right_button", "A")

        self.dry_run = self._bool_parameter("dry_run", False)
        self.dry_run_input = self._string_parameter(
            "dry_run_input", "", allow_empty=True)
        self.dry_run_loop = self._bool_parameter("dry_run_loop", True)

        self.quest_mac = self._string_parameter(
            "quest_mac", "50:99:03:0a:38:78", allow_empty=True)
        self.tracking_warn_period_s = self._positive_float(
            "tracking_warn_period_s", 5.0)

    def _string_parameter(self, name, default, allow_empty=False):
        value = self.declare_parameter(name, default).value
        if not isinstance(value, str):
            raise ValueError("%s 必须是字符串" % name)
        if not allow_empty and not value:
            raise ValueError("%s 不能为空" % name)
        return value

    def _positive_float(self, name, default):
        value = self._float_parameter(name, default)
        if value <= 0.0:
            raise ValueError("%s 必须是正数" % name)
        return value

    def _clamped_float(self, name, default):
        return max(0.0, min(1.0, self._float_parameter(name, default)))

    def _float_parameter(self, name, default):
        value = self.declare_parameter(name, float(default)).value
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("%s 必须是数值" % name) from exc
        if not math.isfinite(result):
            raise ValueError("%s 必须是有限数值" % name)
        return result

    def _bool_parameter(self, name, default):
        value = self.declare_parameter(name, bool(default)).value
        if not isinstance(value, bool):
            raise ValueError("%s 必须是布尔值" % name)
        return value

    def _vector_parameter(self, name, default, length):
        values = self.declare_parameter(name, list(default)).value
        try:
            result = tuple(float(value) for value in values)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "%s 必须包含 %d 个数值" % (name, length)
            ) from exc
        if len(result) != length or not all(
            math.isfinite(value) for value in result
        ):
            raise ValueError("%s 必须包含 %d 个有限数值" % (name, length))
        return result

    def _button_parameter(self, name, default):
        value = self._string_parameter(name, default)
        if value not in BUTTON_INDEX:
            raise ValueError("%s 是未知按键 %r" % (name, value))
        return value

    # ── 周期：目标 / Joy / 急停 ──
    def _tick(self):
        if self._closed:
            return
        if (self._confirm_pending
                and time.monotonic() - self._confirm_started > 3.0):
            self._confirm_pending = False
            self._publish_confirm_status("恢复请求超时：supervisor 未响应")
            self.get_logger().warning("confirm_resume 请求超时")
        try:
            raw = self._source.read()
        except Exception as exc:  # SDK 读取异常不能把心跳带走
            raw = {}
            self._warn_throttled(
                "_last_io_warning",
                "读取 Quest 输入失败：%s；保持心跳，目标暂停" % exc,
            )
        yaw_offset = self._yaw_offset()
        try:
            frame = self._core.update(
                raw.get("left", {}), raw.get("right", {}), yaw_offset,
                now_s=time.monotonic(),
            )
        except Exception as exc:
            self._warn_throttled(
                "_last_io_warning",
                "遥操作帧处理失败：%s；本帧目标暂停" % exc,
            )
            return

        stamp = self.get_clock().now().to_msg()
        for side, publisher in (
            ("left", self.pub_left), ("right", self.pub_right)
        ):
            target = frame.targets.get(side)
            if target is not None:
                publisher.publish(self._pose_message(target, stamp))

        # 每个 tick 都发 Joy，不能只在变化时发：底盘按 Joy 左摇杆换算速度，
        # 200 ms 没有新 Joy/Twist 会触发本地看门狗清零，只在变化时发会变成
        # “推一下停一下”。按键边沿由底盘/arm 下游自己判断，重复 Joy 无副作用。
        self._publish_joy(frame, stamp)
        if frame.estop_pressed:
            self._publish_estop(frame.estop_reason, stamp)
        if frame.clear_estop_pressed:
            self._call_clear_estop(frame.clear_estop_reason)
        if frame.confirm_pressed:
            self._call_confirm(frame.confirm_reason)

        for event in frame.events:
            message = _EVENT_MESSAGES.get(event)
            if message:
                self.get_logger().warning(message)
            else:
                self.get_logger().info(event)

        if not frame.tracked["left"] and not frame.tracked["right"]:
            now = time.monotonic()
            if now - self._last_tracking_warning >= self.tracking_warn_period_s:
                self.get_logger().warning(
                    "没有已跟踪的 Quest 控制器；检查 PC-service、Quest 应用、"
                    "跟踪与 Send；心跳仍在继续"
                )
                self._last_tracking_warning = now

    def _pose_message(self, target, stamp):
        position, quaternion = target
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.robot_base_frame
        message.pose.position.x = position[0]
        message.pose.position.y = position[1]
        message.pose.position.z = position[2]
        message.pose.orientation.w = quaternion[0]
        message.pose.orientation.x = quaternion[1]
        message.pose.orientation.y = quaternion[2]
        message.pose.orientation.z = quaternion[3]
        return message

    def _publish_joy(self, frame, stamp):
        joy = Joy()
        joy.header.stamp = stamp
        joy.axes = list(frame.joy_axes)
        joy.buttons = list(frame.joy_buttons)
        self.pub_button.publish(joy)

    def _publish_estop(self, reason, stamp):
        message = SafetyObservation()
        message.header.stamp = stamp
        message.header.frame_id = self.robot_base_frame
        message.source_id = _SYSTEM_ESTOP_SOURCE
        message.zone_id = "system"
        message.state = SafetyObservation.ESTOP
        message.confidence = 1.0
        message.evidence = 0
        message.detail_json = json.dumps(
            {
                "reason": reason,
                "button": self.estop_button,
                "button_index": BUTTON_INDEX[self.estop_button],
                "bridge": "vr_safe_teleop",
                "dry_run": bool(self.dry_run),
                "heartbeat_seq": self._core.heartbeat_seq,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.pub_observation.publish(message)
        self.get_logger().error(
            "系统软件急停：%s；已发布 ESTOP 观察，等待 supervisor 锁存"
            % reason
        )

    def _call_clear_estop(self, reason):
        if self._clear_pending:
            self.get_logger().warning(
                "上一笔 clear_estop 请求尚未返回，忽略本次 %s"
                % self.clear_estop_button
            )
            return
        if not self.clear_estop_client.service_is_ready():
            self.get_logger().error(
                "clear_estop 服务不可用：%s；%s 未生效"
                % (self.clear_estop_service, self.clear_estop_button)
            )
            return
        self._clear_pending = True
        self.get_logger().warning(
            "%s；只解除 LJ 按钮锁存，许可恢复仍需 confirm_resume" % reason
        )
        future = self.clear_estop_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_clear_estop_done)

    def _on_clear_estop_done(self, future):
        self._clear_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error("clear_estop 调用失败：%s" % exc)
            return
        if response.success:
            self.get_logger().info(
                "clear_estop 成功：%s（许可恢复仍需 confirm_resume）"
                % response.message
            )
        else:
            self.get_logger().warning(
                "clear_estop 被拒绝：%s" % response.message
            )

    def _on_permit(self, msg):
        self.permit_received = True
        self.permit_decision = int(msg.decision)
        self.permit_latched = bool(msg.latched)

    def _call_confirm(self, reason):
        if not self.permit_received:
            self._publish_confirm_status("恢复失败：还没收到 supervisor 许可")
            return
        if self.permit_decision != MotionPermit.REVOKED or not self.permit_latched:
            self._publish_confirm_status("当前没有需要恢复的许可")
            return
        if self._confirm_pending:
            self.get_logger().warning(
                "上一笔 confirm_resume 请求尚未返回，忽略本次 RJ 长按")
            return
        if not self.confirm_client.service_is_ready():
            self._publish_confirm_status("恢复失败：supervisor 服务未就绪")
            self.get_logger().error(
                "confirm_resume 服务不可用：%s" % self.confirm_service)
            return
        self._confirm_pending = True
        self._confirm_started = time.monotonic()
        self.get_logger().warning(reason)
        req = ConfirmResume.Request()
        req.confirmed_by = "quest_gamepad"
        req.note = reason
        future = self.confirm_client.call_async(req)
        future.add_done_callback(self._on_confirm_done)

    def _on_confirm_done(self, future):
        self._confirm_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self._publish_confirm_status("恢复失败：%s" % exc)
            self.get_logger().error("confirm_resume 调用失败：%s" % exc)
            return
        if response.accepted:
            self._publish_confirm_status(
                "许可已恢复：请松开并重新按 Grip / 底盘重新使能")
            self.get_logger().info(
                "confirm_resume 成功：%s" % response.message)
        else:
            self._publish_confirm_status(
                "恢复被拒绝：%s" % response.message)
            self.get_logger().warning(
                "confirm_resume 被拒绝：%s" % response.message)

    def _publish_confirm_status(self, text):
        msg = String()
        msg.data = str(text)
        self.pub_confirm_status.publish(msg)

    # ── 周期：心跳 ──
    def _publish_heartbeat(self):
        if self._closed:
            return
        message = TeleopHeartbeat()
        message.header.stamp = self.get_clock().now().to_msg()
        message.source_id = self.heartbeat_source_id
        message.seq = self._core.next_heartbeat_seq()
        self.pub_heartbeat.publish(message)

    # ── 辅助 ──
    def _yaw_offset(self):
        try:
            degrees = float(
                self.get_parameter("headset_yaw_offset_deg").value
            )
        except (TypeError, ValueError):
            degrees = 0.0
        if not math.isfinite(degrees):
            degrees = 0.0
        if degrees != self._last_yaw_offset:
            self.get_logger().info(
                "头显 yaw 偏移：%.1f°（绕机器人 base z 轴）" % degrees
            )
            self._last_yaw_offset = degrees
        return math.radians(degrees)

    def _warn_throttled(self, attr, message):
        now = time.monotonic()
        if now - getattr(self, attr) >= 2.0:
            self.get_logger().error(message)
            setattr(self, attr, now)

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._source.close()
        except Exception:
            pass


def _qos(reliability):
    return QoSProfile(
        reliability=reliability,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _validate_vr_safe_name(parameter_name, value):
    if not isinstance(value, str) or not value:
        raise ValueError("%s 不能为空" % parameter_name)
    if "/rc_ctrl" in value:
        raise ValueError(
            "%s 不能再用 demo 的 /rc_ctrl/*（收到 %r）" % (parameter_name, value)
        )
    if not value.startswith("/vr_safe/"):
        raise ValueError(
            "%s 必须位于 /vr_safe/ 下（收到 %r）" % (parameter_name, value)
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = VrSafeTeleopBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (RuntimeError, ValueError, InvalidParameterTypeException) as exc:
        print("[vr_safe_teleop] 错误：%s" % exc, file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
