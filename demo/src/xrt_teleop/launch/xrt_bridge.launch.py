import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _project_python_prefix():
    venv = os.environ.get("XRT_VENV", "")
    interpreter = os.path.join(venv, "bin", "python")
    if not venv or not os.path.isfile(interpreter):
        raise RuntimeError(
            "XRT_VENV is not set or invalid; source demo/setup.zsh before launching"
        )
    return [interpreter]


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("xrt_teleop"), "config", "xrt_bridge.yaml"
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate", default_value="125.0",
        description="Pose and Joy publication rate in Hz"
    )
    smooth_alpha_arg = DeclareLaunchArgument(
        "smooth_alpha", default_value="0.4",
        description="Pose low-pass coefficient: 0 disables filtering, 1 follows raw"
    )
    grip_threshold_arg = DeclareLaunchArgument(
        "grip_threshold", default_value="0.5",
        description="Grip value used to classify the digital LG/RG buttons"
    )
    headset_yaw_offset_arg = DeclareLaunchArgument(
        "headset_yaw_offset_deg", default_value="0.0",
        description=(
            "Extra yaw (deg) about the robot base z axis applied to headset "
            "poses; 180 corrects a headset tracking frame that was anchored "
            "facing the opposite way"
        )
    )

    bridge_node = Node(
        package="xrt_teleop",
        executable="xrt_bridge",
        name="xrt_teleop_bridge",
        output="screen",
        prefix=_project_python_prefix(),
        on_exit=Shutdown(reason="XRoboToolkit bridge exited"),
        parameters=[
            config_file,
            {
                "publish_rate": ParameterValue(
                    LaunchConfiguration("publish_rate"), value_type=float
                ),
                "smooth_alpha": ParameterValue(
                    LaunchConfiguration("smooth_alpha"), value_type=float
                ),
                "grip_threshold": ParameterValue(
                    LaunchConfiguration("grip_threshold"), value_type=float
                ),
                "headset_yaw_offset_deg": ParameterValue(
                    LaunchConfiguration("headset_yaw_offset_deg"), value_type=float
                ),
            },
        ],
    )

    return LaunchDescription([
        publish_rate_arg,
        smooth_alpha_arg,
        grip_threshold_arg,
        headset_yaw_offset_arg,
        bridge_node,
    ])
