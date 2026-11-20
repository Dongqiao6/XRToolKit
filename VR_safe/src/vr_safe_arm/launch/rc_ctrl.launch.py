"""单臂启动（vr_safe_arm）。

一个进程只连一台 JAKA，因此双臂要由 dual_arm.launch.py 起两个本 launch。
默认话题遵循 VR_safe 契约：
  target  /vr_safe/teleop/{left,right}_target
  button  /vr_safe/teleop/button
  permit  /vr_safe/permit
  state   /vr_safe/arm/{left,right}/state
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_ip(arm):
    return "192.168.71.36" if arm == "right" else "192.168.71.37"


def _launch_setup(context, *args, **kwargs):
    arm = LaunchConfiguration("arm_name").perform(context).strip() or "left"
    if arm not in ("left", "right"):
        raise RuntimeError(f"arm_name 必须是 left 或 right，收到 {arm!r}")

    robot_ip = LaunchConfiguration("robot_ip").perform(context).strip()
    if not robot_ip:
        robot_ip = _default_ip(arm)

    target_topic = LaunchConfiguration("target_topic").perform(context).strip()
    if not target_topic:
        target_topic = f"/vr_safe/teleop/{arm}_target"

    state_topic = LaunchConfiguration("state_topic").perform(context).strip()
    if not state_topic:
        state_topic = f"/vr_safe/arm/{arm}/state"

    is_right = arm == "right"
    node = Node(
        package="vr_safe_arm",
        executable="rc_ctrl_node",
        name=f"vr_safe_arm_{arm}",
        output="screen",
        parameters=[{
            "arm_name": arm,
            "robot_ip": robot_ip,
            "target_topic": target_topic,
            "button_topic": LaunchConfiguration("button_topic"),
            "permit_topic": LaunchConfiguration("permit_topic"),
            "state_topic": state_topic,
            "dry_run": LaunchConfiguration("dry_run"),
            "slow_scale": LaunchConfiguration("slow_scale"),
            "target_watchdog_ms": LaunchConfiguration("target_watchdog_ms"),
            "permit_watchdog_ms": LaunchConfiguration("permit_watchdog_ms"),
            "state_publish_rate_hz": LaunchConfiguration("state_publish_rate_hz"),
            "grip_axis": 1 if is_right else 0,
            "gripper_axis": 3 if is_right else 2,
            "use_rpy_ctrl": LaunchConfiguration("use_rpy_ctrl"),
            "enable_y_limit": LaunchConfiguration("enable_y_limit"),
            "debug": LaunchConfiguration("debug"),
            "servo_blackbox_dir": LaunchConfiguration("servo_blackbox_dir"),
        }],
        on_exit=Shutdown(reason=f"vr_safe_arm_{arm} exited"),
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm_name", default_value="left",
                              description="left 或 right；决定默认话题与 Grip 轴"),
        DeclareLaunchArgument("robot_ip", default_value="",
                              description="JAKA 控制器 IP；留空时 left=192.168.71.37, "
                                          "right=192.168.71.36"),
        DeclareLaunchArgument("target_topic", default_value="",
                              description="留空时 /vr_safe/teleop/{arm}_target"),
        DeclareLaunchArgument("button_topic", default_value="/vr_safe/teleop/button"),
        DeclareLaunchArgument("permit_topic", default_value="/vr_safe/permit"),
        DeclareLaunchArgument("state_topic", default_value="",
                              description="留空时 /vr_safe/arm/{arm}/state"),
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="true 时跳过 JAKA/PGI 调用，仅验证安全逻辑"),
        DeclareLaunchArgument("slow_scale", default_value="0.3",
                              description="SLOW 时平移/姿态步长缩放系数"),
        DeclareLaunchArgument("target_watchdog_ms", default_value="200.0"),
        DeclareLaunchArgument("permit_watchdog_ms", default_value="300.0",
                              description="supervisor 许可失联多久后 HOLD"),
        DeclareLaunchArgument("state_publish_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("use_rpy_ctrl", default_value="true"),
        DeclareLaunchArgument("enable_y_limit", default_value="true"),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("servo_blackbox_dir", default_value="log/teleop",
                              description="每次 servo_p 的 CSV 落盘目录；空字符串关闭"),
        OpaqueFunction(function=_launch_setup),
    ])
