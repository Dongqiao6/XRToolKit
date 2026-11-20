"""双臂启动（vr_safe_arm）。

JAKA SDK 每进程只允许一条连接，所以这里必须起两个独立进程：
  vr_safe_arm_left  -> robot_left_ip
  vr_safe_arm_right -> robot_right_ip
一个进程连两台臂不受支持。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("robot_left_ip", default_value="192.168.71.37",
                              description="左臂 JAKA 控制器 IP"),
        DeclareLaunchArgument("robot_right_ip", default_value="192.168.71.36",
                              description="右臂 JAKA 控制器 IP"),
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="true 时两个进程都不连接 JAKA，仅验证安全逻辑"),
        DeclareLaunchArgument("permit_topic", default_value="/vr_safe/permit"),
        DeclareLaunchArgument("slow_scale", default_value="0.3"),
        DeclareLaunchArgument("button_topic", default_value="/vr_safe/teleop/button"),
        DeclareLaunchArgument("target_watchdog_ms", default_value="200.0"),
        DeclareLaunchArgument("permit_watchdog_ms", default_value="300.0"),
        DeclareLaunchArgument("state_publish_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("use_rpy_ctrl", default_value="true"),
        DeclareLaunchArgument("enable_y_limit", default_value="true"),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("servo_blackbox_dir", default_value="log/teleop"),
    ]

    common = {
        "dry_run": LaunchConfiguration("dry_run"),
        "permit_topic": LaunchConfiguration("permit_topic"),
        "slow_scale": LaunchConfiguration("slow_scale"),
        "button_topic": LaunchConfiguration("button_topic"),
        "target_watchdog_ms": LaunchConfiguration("target_watchdog_ms"),
        "permit_watchdog_ms": LaunchConfiguration("permit_watchdog_ms"),
        "state_publish_rate_hz": LaunchConfiguration("state_publish_rate_hz"),
        "use_rpy_ctrl": LaunchConfiguration("use_rpy_ctrl"),
        "enable_y_limit": LaunchConfiguration("enable_y_limit"),
        "debug": LaunchConfiguration("debug"),
        "servo_blackbox_dir": LaunchConfiguration("servo_blackbox_dir"),
    }

    rc_ctrl_launch = PathJoinSubstitution(
        [FindPackageShare("vr_safe_arm"), "launch", "rc_ctrl.launch.py"])

    left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rc_ctrl_launch),
        launch_arguments={
            "arm_name": "left",
            "robot_ip": LaunchConfiguration("robot_left_ip"),
            **common,
        }.items(),
    )
    right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rc_ctrl_launch),
        launch_arguments={
            "arm_name": "right",
            "robot_ip": LaunchConfiguration("robot_right_ip"),
            **common,
        }.items(),
    )

    return LaunchDescription(args + [left, right])
