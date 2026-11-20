"""Dry-run：supervisor + fake 双臂/底盘 + 场景脚本，不需要任何硬件。"""

import os

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    root = os.environ.get("VR_SAFE_ROOT", os.getcwd())
    config = os.path.join(root, "config", "vr_safe.yaml")
    if not os.path.isfile(config):
        raise RuntimeError(
            f"找不到安全配置 {config}；请先 source scripts/env.sh 或在 VR_safe/ 下运行")
    return LaunchDescription([
        Node(
            package="vr_safe_supervisor",
            executable="supervisor",
            name="vr_safe_supervisor",
            output="screen",
            parameters=[{"config": config}],
        ),
        Node(package="vr_safe_fakes", executable="fake_arm", name="fake_arm_left",
             output="screen", parameters=[{"arm": "left", "dry_run": True}]),
        Node(package="vr_safe_fakes", executable="fake_arm", name="fake_arm_right",
             output="screen", parameters=[{"arm": "right", "dry_run": True}]),
        Node(package="vr_safe_fakes", executable="fake_chassis", name="fake_chassis",
             output="screen"),
        Node(package="vr_safe_fakes", executable="scenario", name="dry_run_scenario",
             output="screen", on_exit=Shutdown(reason="dry-run scenario finished")),
    ])
