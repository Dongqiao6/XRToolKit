import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    bridge_node = Node(
        package="xrt_teleop",
        executable="xrt_bridge",
        name="xrt_teleop_bridge",
        output="screen",
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
            },
        ],
    )

    return LaunchDescription([
        publish_rate_arg,
        smooth_alpha_arg,
        grip_threshold_arg,
        bridge_node,
    ])
