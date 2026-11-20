"""Two rc_ctrl_node instances bound to the /vision/rc_ctrl topics.

Nothing here knows about the camera: it is the same servo layer the Quest 3
demo uses, only fed by vision_bridge instead of xrt_teleop.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _common(prefix):
    return {
        'button_topic': LaunchConfiguration('button_topic'),
        'use_rpy_ctrl': LaunchConfiguration('use_rpy_ctrl'),
        'enable_y_limit': LaunchConfiguration('enable_y_limit'),
        'debug': LaunchConfiguration('debug'),
        'gripper_force': LaunchConfiguration('gripper_force'),
        'gripper_speed': LaunchConfiguration('gripper_speed'),
        'gripper_rs485_channel': LaunchConfiguration('gripper_rs485_channel'),
        'gripper_configure_tio': LaunchConfiguration('gripper_configure_tio'),
        'gripper_use_button': LaunchConfiguration('gripper_use_button'),
    }


def generate_launch_description():
    args = [
        DeclareLaunchArgument('robot_left_ip', default_value='192.168.71.37'),
        DeclareLaunchArgument('robot_right_ip', default_value='192.168.71.36'),
        DeclareLaunchArgument('button_topic', default_value='/vision/rc_ctrl/button'),
        DeclareLaunchArgument('use_rpy_ctrl', default_value='false',
                              description='Vision teleop is position-only by default'),
        DeclareLaunchArgument('enable_y_limit', default_value='true'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('gripper_force', default_value='20'),
        DeclareLaunchArgument('gripper_speed', default_value='40'),
        DeclareLaunchArgument('gripper_rs485_channel', default_value='1'),
        DeclareLaunchArgument('gripper_configure_tio', default_value='false'),
        DeclareLaunchArgument('gripper_use_button', default_value='false'),
    ]

    left = Node(
        package='vision_rc',
        executable='rc_ctrl_node',
        name='rc_ctrl_left',
        output='screen',
        parameters=[dict(
            arm_name='left',
            robot_ip=LaunchConfiguration('robot_left_ip'),
            target_topic='/vision/rc_ctrl/left_target',
            grip_axis=0,
            gripper_axis=2,
            **_common('left'),
        )],
    )
    right = Node(
        package='vision_rc',
        executable='rc_ctrl_node',
        name='rc_ctrl_right',
        output='screen',
        parameters=[dict(
            arm_name='right',
            robot_ip=LaunchConfiguration('robot_right_ip'),
            target_topic='/vision/rc_ctrl/right_target',
            grip_axis=1,
            gripper_axis=3,
            **_common('right'),
        )],
    )
    return LaunchDescription(args + [left, right])
