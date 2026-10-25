import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- XRoboToolkit Quest 3 reader ----
    # Publishes /rc_ctrl/left_target, /rc_ctrl/right_target and /rc_ctrl/button
    # from the Quest controllers; RC_ctrl subscribes to these to servo the arms.
    xrt_publish_rate_arg = DeclareLaunchArgument(
        'xrt_publish_rate', default_value='125.0',
        description='XRoboToolkit pose and Joy publication rate in Hz')
    xrt_smooth_alpha_arg = DeclareLaunchArgument(
        'xrt_smooth_alpha', default_value='0.4',
        description='XRoboToolkit pose low-pass coefficient (0 disables filtering)')
    xrt_grip_threshold_arg = DeclareLaunchArgument(
        'xrt_grip_threshold', default_value='0.5',
        description='Threshold used for the digital LG/RG and LTr/RTr Joy buttons')

    # ---- Dual-arm RC control (left + right rc_ctrl_node) ----
    robot_left_ip_arg = DeclareLaunchArgument(
        'robot_left_ip', default_value='192.168.71.37',
        description='IP address of the left JAKA robot')
    robot_right_ip_arg = DeclareLaunchArgument(
        'robot_right_ip', default_value='192.168.71.36',
        description='IP address of the right JAKA robot')
    gripper_force_arg = DeclareLaunchArgument(
        'gripper_force', default_value='20',
        description='PGI force percentage (20..100)')
    gripper_speed_arg = DeclareLaunchArgument(
        'gripper_speed', default_value='100',
        description='PGI speed percentage (1..100)')
    gripper_baudrate_arg = DeclareLaunchArgument(
        'gripper_baudrate', default_value='115200',
        description='PGI RS485 baud rate (default 115200)')
    gripper_rs485_channel_arg = DeclareLaunchArgument(
        'gripper_rs485_channel', default_value='1',
        description='JAKA SDK RS485 channel: 0=RS485H/TIO channel 1, 1=RS485L/TIO channel 2')
    gripper_slave_id_arg = DeclareLaunchArgument(
        'gripper_slave_id', default_value='1', description='PGI Modbus slave ID')
    gripper_open_position_arg = DeclareLaunchArgument(
        'gripper_open_position', default_value='1000',
        description='PGI open target in per mille (0..1000)')
    gripper_closed_position_arg = DeclareLaunchArgument(
        'gripper_closed_position', default_value='0',
        description='PGI closed target in per mille (0..1000)')
    gripper_linear_arg = DeclareLaunchArgument(
        'gripper_linear', default_value='true',
        description='Use linear (analog axes) control for gripper by default')
    gripper_trigger_deadband_arg = DeclareLaunchArgument(
        'gripper_trigger_deadband', default_value='0.02',
        description='Minimal change in analog trigger to send a new Grip()')
    gripper_initialize_arg = DeclareLaunchArgument(
        'gripper_initialize', default_value='true',
        description='Initialize PGI grippers during node startup')
    gripper_initialize_command_arg = DeclareLaunchArgument(
        'gripper_initialize_command', default_value='1',
        description='PGI initialization command: 1 or 165 (0xA5)')
    gripper_initialize_delay_arg = DeclareLaunchArgument(
        'gripper_initialize_delay_ms', default_value='2000',
        description='Wait time after PGI initialization command')
    gripper_configure_tio_arg = DeclareLaunchArgument(
        'gripper_configure_tio', default_value='false',
        description='Configure TIO via SDK; false uses the JAKA App saved configuration')
    gripper_enable_tio_power_arg = DeclareLaunchArgument(
        'gripper_enable_tio_power', default_value='true',
        description='Enable JAKA TIO voltage output for the grippers')
    gripper_tio_voltage_arg = DeclareLaunchArgument(
        'gripper_tio_voltage', default_value='0',
        description='JAKA TIO output voltage: 0=24 V, 1=12 V')
    gripper_use_button_arg = DeclareLaunchArgument(
        'gripper_use_button', default_value='false',
        description='Use only Joy.buttons; default also accepts digital trigger fallback')
    gripper_button_index_arg = DeclareLaunchArgument(
        'gripper_button_index', default_value='-1',
        description='Joy.buttons index; -1 selects left LTr=10 or right RTr=11')
    use_rpy_ctrl_arg = DeclareLaunchArgument(
        'use_rpy_ctrl', default_value='true',
        description='true = 6D teleop including orientation; false = position only')
    rotation_gain_arg = DeclareLaunchArgument(
        'rotation_gain', default_value='1.0',
        description='Fraction of remaining controller rotation applied per cycle (0..1)')
    rotation_deadband_arg = DeclareLaunchArgument(
        'rotation_deadband_rad', default_value='0.005',
        description='Ignore controller orientation changes below this angle in radians')
    rotation_max_step_arg = DeclareLaunchArgument(
        'rotation_max_step_rad', default_value='0.005',
        description='Maximum commanded rotation per 8 ms servo cycle in radians')
    rotation_sign_arg = DeclareLaunchArgument(
        'rotation_sign', default_value='1',
        description='Robot rotation direction: 1 or -1, applied to all rotation axes')
    enable_y_limit_arg = DeclareLaunchArgument(
        'enable_y_limit', default_value='true',
        description='true = clamp arm TCP y (left > -110 mm, right < 110 mm)')

    # ---- AMR chassis ----
    connect_ip_arg = DeclareLaunchArgument(
        'connect_ip', default_value='192.168.71.50',
        description='AMR control host IP')
    connect_username_arg = DeclareLaunchArgument(
        'connect_username', default_value='',
        description='AMR control host username')
    connect_passwd_arg = DeclareLaunchArgument(
        'connect_passwd', default_value='',
        description='AMR control host password')

    xrt_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('xrt_teleop'), 'launch', 'xrt_bridge.launch.py'])
        ]),
        launch_arguments={
            'publish_rate': LaunchConfiguration('xrt_publish_rate'),
            'smooth_alpha': LaunchConfiguration('xrt_smooth_alpha'),
            'grip_threshold': LaunchConfiguration('xrt_grip_threshold'),
        }.items(),
    )

    rc_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('jaka_driver'), 'launch', 'RC_ctrl.launch.py'])
        ]),
        launch_arguments={
            'robot_left_ip': LaunchConfiguration('robot_left_ip'),
            'robot_right_ip': LaunchConfiguration('robot_right_ip'),
            'gripper_force': LaunchConfiguration('gripper_force'),
            'gripper_speed': LaunchConfiguration('gripper_speed'),
            'gripper_baudrate': LaunchConfiguration('gripper_baudrate'),
            'gripper_rs485_channel': LaunchConfiguration('gripper_rs485_channel'),
            'gripper_slave_id': LaunchConfiguration('gripper_slave_id'),
            'gripper_open_position': LaunchConfiguration('gripper_open_position'),
            'gripper_closed_position': LaunchConfiguration('gripper_closed_position'),
            'gripper_linear': LaunchConfiguration('gripper_linear'),
            'gripper_trigger_deadband': LaunchConfiguration('gripper_trigger_deadband'),
            'gripper_initialize': LaunchConfiguration('gripper_initialize'),
            'gripper_initialize_command': LaunchConfiguration('gripper_initialize_command'),
            'gripper_initialize_delay_ms': LaunchConfiguration('gripper_initialize_delay_ms'),
            'gripper_configure_tio': LaunchConfiguration('gripper_configure_tio'),
            'gripper_enable_tio_power': LaunchConfiguration('gripper_enable_tio_power'),
            'gripper_tio_voltage': LaunchConfiguration('gripper_tio_voltage'),
            'gripper_use_button': LaunchConfiguration('gripper_use_button'),
            'gripper_button_index': LaunchConfiguration('gripper_button_index'),
            'use_rpy_ctrl': LaunchConfiguration('use_rpy_ctrl'),
            'rotation_gain': LaunchConfiguration('rotation_gain'),
            'rotation_deadband_rad': LaunchConfiguration('rotation_deadband_rad'),
            'rotation_max_step_rad': LaunchConfiguration('rotation_max_step_rad'),
            'rotation_sign': LaunchConfiguration('rotation_sign'),
            'enable_y_limit': LaunchConfiguration('enable_y_limit'),
        }.items(),
    )

    amr_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('sr_amr_control'), 'launch', 'amr_control.launch.py'])
        ]),
        launch_arguments={
            'connect_ip': LaunchConfiguration('connect_ip'),
            'connect_username': LaunchConfiguration('connect_username'),
            'connect_passwd': LaunchConfiguration('connect_passwd'),
        }.items(),
    )

    return LaunchDescription([
        xrt_publish_rate_arg,
        xrt_smooth_alpha_arg,
        xrt_grip_threshold_arg,
        robot_left_ip_arg,
        robot_right_ip_arg,
        gripper_force_arg,
        gripper_speed_arg,
        gripper_baudrate_arg,
        gripper_rs485_channel_arg,
        gripper_slave_id_arg,
        gripper_open_position_arg,
        gripper_closed_position_arg,
        gripper_linear_arg,
        gripper_trigger_deadband_arg,
        gripper_initialize_arg,
        gripper_initialize_command_arg,
        gripper_initialize_delay_arg,
        gripper_configure_tio_arg,
        gripper_enable_tio_power_arg,
        gripper_tio_voltage_arg,
        gripper_use_button_arg,
        gripper_button_index_arg,
        use_rpy_ctrl_arg,
        rotation_gain_arg,
        rotation_deadband_arg,
        rotation_max_step_arg,
        rotation_sign_arg,
        enable_y_limit_arg,
        connect_ip_arg,
        connect_username_arg,
        connect_passwd_arg,
        xrt_bridge_launch,
        rc_ctrl_launch,
        amr_control_launch,
    ])
