"""Full vision teleoperation: camera bridge + both servo nodes."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    root = os.environ.get('VISION_TELEOP_ROOT', os.getcwd())
    bridge_python = os.path.join(root, '.venv', 'bin', 'python')

    args = [
        DeclareLaunchArgument('start_bridge', default_value='true',
                              description='Start the D435 bridge in this launch'),
        DeclareLaunchArgument('bridge_python', default_value=bridge_python),
        DeclareLaunchArgument('robot_left_ip', default_value='192.168.71.37'),
        DeclareLaunchArgument('robot_right_ip', default_value='192.168.71.36'),
        DeclareLaunchArgument('use_rpy_ctrl', default_value='false'),
        DeclareLaunchArgument('enable_y_limit', default_value='true'),
    ]

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('vision_rc'), 'launch', 'rc_ctrl_dual.launch.py'])
        ),
        launch_arguments={
            'robot_left_ip': LaunchConfiguration('robot_left_ip'),
            'robot_right_ip': LaunchConfiguration('robot_right_ip'),
            'use_rpy_ctrl': LaunchConfiguration('use_rpy_ctrl'),
            'enable_y_limit': LaunchConfiguration('enable_y_limit'),
        }.items(),
    )

    bridge = ExecuteProcess(
        cmd=[LaunchConfiguration('bridge_python'), '-m', 'vision_bridge.node'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_bridge')),
    )

    return LaunchDescription(args + [servo, bridge])
