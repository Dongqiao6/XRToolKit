"""真机组合启动：teleop + perception + supervisor + 双臂 + 可选底盘。

dry_run:=true 时 teleop/双臂/底盘走各自的 dry-run 路径；perception 仍会开相机，
没有相机就持续发 UNKNOWN（许可保持 REVOKED）。只需验证无硬件启动时用
start_perception:=false。
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _root():
    return os.environ.get("VR_SAFE_ROOT", os.getcwd())


def generate_launch_description():
    root = _root()
    dry_run = LaunchConfiguration("dry_run")
    start_amr = LaunchConfiguration("start_amr")
    start_perception = LaunchConfiguration("start_perception")
    start_teleop = LaunchConfiguration("start_teleop")
    start_arms = LaunchConfiguration("start_arms")

    args = [
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="各执行端走 dry-run，不连硬件"),
        DeclareLaunchArgument("start_amr", default_value="true",
                              description="是否启动 AMR 底盘"),
        DeclareLaunchArgument("start_perception", default_value="true"),
        DeclareLaunchArgument("start_teleop", default_value="true"),
        DeclareLaunchArgument("start_arms", default_value="true"),
        DeclareLaunchArgument("allow_uncalibrated", default_value="false",
                              description="开发降级：外参无效也启动感知"),
        DeclareLaunchArgument("show_camera", default_value="false",
                              description="开实时相机窗口并让感知发压缩图"),
        DeclareLaunchArgument("config_dir", default_value=os.path.join(root, "config")),
    ]

    cfg = LaunchConfiguration("config_dir")
    supervisor = Node(
        package="vr_safe_supervisor", executable="supervisor",
        name="vr_safe_supervisor", output="screen",
        parameters=[{"config": PathJoinSubstitution([cfg, "vr_safe.yaml"])}],
    )
    teleop = Node(
        package="vr_safe_teleop", executable="teleop_bridge",
        output="screen",
        condition=IfCondition(start_teleop),
        parameters=[
            PathJoinSubstitution([cfg, "teleop.yaml"]),
            {"dry_run": ParameterValue(dry_run, value_type=bool)},
        ],
    )

    def _perception(context):
        lc = context.launch_configurations
        if str(lc.get("start_perception", "true")).lower() not in ("true", "1"):
            return []
        args = ["--config", os.path.join(lc["config_dir"], "perception.yaml")]
        if str(lc.get("allow_uncalibrated", "false")).lower() in ("true", "1"):
            args.append("--allow-uncalibrated")
        if str(lc.get("show_camera", "false")).lower() in ("true", "1"):
            args.append("--publish-camera")
        return [Node(
            package="vr_safe_perception", executable="observation_node",
            name="vr_safe_observation", output="screen", arguments=args,
        )]
    arms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("vr_safe_arm"), "launch", "dual_arm.launch.py"])),
        condition=IfCondition(start_arms),
        launch_arguments={
            "dry_run": dry_run,
            "permit_topic": "/vr_safe/permit",
            "slow_scale": "0.3",
            "button_topic": "/vr_safe/teleop/button",
        }.items(),
    )
    chassis = Node(
        package="vr_safe_chassis", executable="chassis_node",
        name="vr_safe_chassis", output="screen",
        condition=IfCondition(start_amr),
        parameters=[
            {"config": PathJoinSubstitution([cfg, "chassis.yaml"])},
            {"dry_run": ParameterValue(dry_run, value_type=bool)},
        ],
        on_exit=Shutdown(reason="vr_safe_chassis exited"),
    )

    viewer = Node(
        package="vr_safe_camera_view", executable="camera_view",
        name="vr_safe_camera_view", output="screen",
        condition=IfCondition(LaunchConfiguration("show_camera")),
    )

    return LaunchDescription(args + [supervisor, teleop, OpaqueFunction(
        function=_perception), viewer, arms, chassis])
