# Quest 3 + XRoboToolKit 控制 JAKA 双臂与 AMR 底盘

本工程是 `RemoteCtrl-master` 的 XRoboToolKit 迁移版。Quest 3 不再通过旧 APK 的
adb/logcat 文本传输位姿，而是运行 `XRoboToolkit-Quest-1.0.1.apk`，由
`XRoboToolkit-PC-Service` 接收头显数据；`xrt_teleop` 再把数据转换成本工程原有的
ROS 2 控制话题。JAKA 双臂驱动、PGI 夹爪控制、AMR 底盘接口及设备地址保持原样。

Quest 3 的 MAC 地址为 `50:99:03:0a:38:78`。XRoboToolkit 使用 PC-service 自动
发现设备，日常遥操作不需要配置 MAC；该地址用于设备识别、库存记录和网络排查。

## 修改后最短运行流程

首次使用先安装 XRoboToolkit Python SDK，再编译工程。以下命令在 zsh 中执行：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./install_xrt_sdk.zsh
./clean.zsh
./build.zsh
source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"
ros2 pkg prefix jaka_driver
```

最后一条命令必须输出 `jaka_driver` 的安装路径。如果没有输出，说明编译或工作区加载
失败，不要继续启动机械臂。

运行时使用三个终端：

**终端 1：启动 XRoboToolkit PC-service**

```zsh
cd /opt/apps/roboticsservice
./runService.sh
```

**Quest 3：发送数据**

不需要每次手工执行 adb。下面的 `run_teleop.zsh` 会自动连接 adb、启动 Quest 上的
XRoboToolkit APK，并加载 ROS 工作区。第一次建议用 USB 连接一次 Quest，让脚本建立
无线 adb 并保存 IP；无线 adb 保持开启时可直接无线启动。Quest 重启后若无线调试失效，
再用 USB 连接运行一次即可。也可以手动在头显中打开 `XRoboToolkit`。

在 APK 中输入 JAKA 主机的局域网 IP，开启控制器 Tracking 和 Send。这里不再运行旧的
`quest_reader`，也不再需要旧 APK 的 adb logcat。

**终端 2：启动 JAKA 双臂和夹爪**

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"

./run_teleop.zsh \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  gripper_rs485_channel:=1 \
  gripper_force:=20 \
  gripper_speed:=40 \
  gripper_configure_tio:=false
```

`run_teleop.zsh` 会依次启动 Quest APK、加载 ROS/zsh 环境，再执行
`ros2 launch jaka_driver dual_arm_teleop.launch.py`。

按住左手柄 Grip 控制左臂，按住右手柄 Grip 控制右臂，松开即停止。左手 Trigger
控制左夹爪，右手 Trigger 控制右夹爪。首次测试应空载、低速，并先分别测试左右臂。

如果需要同时启动 AMR 底盘：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"

ros2 launch robot_bringup RC_robot_bringup.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  connect_ip:=192.168.71.50 \
  gripper_rs485_channel:=1 \
  gripper_configure_tio:=false
```

启动前必须给左、右 JAKA 控制器上电。左臂未上电时会出现
`left login FAILED ... ERR_INVALID_HANDLER`；请上电后重新启动整套 launch。

以后每次运行，如果工作区代码和编译选项没有变化，可直接执行
`./run_teleop.zsh ...`。如果自动 adb 启动失败，可设置 Quest IP 后重试：

```zsh
QUEST_IP=<Quest IP> ./run_teleop.zsh
```

APK 已经打开时也可以先执行
`source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"`，再运行对应的
`ros2 launch` 命令。

## 目录

```text
demo/
├── src/
│   ├── xrt_teleop/          # XRoboToolkit SDK -> ROS 2 位姿/按键桥
│   ├── jaka_driver/         # JAKA 双臂伺服与 PGI 夹爪驱动
│   ├── jaka_msgs/           # JAKA 消息和服务
│   ├── sr_amr_control/      # AMR 底盘 ROS 2 控制
│   ├── sr_amr_interfaces/   # AMR 消息/动作接口
│   └── robot_bringup/       # 双臂 + 底盘整体启动
├── vendor/                  # AMR Python SDK wheel
├── .venv/                   # 工程私有 Python 环境（不提交 Git）
├── build.zsh                # 编译 ROS 2 工作区
├── clean.zsh                # 删除编译和运行生成物
├── setup.zsh                # 加载 ROS 与工作区
├── install_xrt_sdk.zsh      # 安装 xrobotoolkit_sdk
├── launch_quest.zsh         # 自动连接 adb 并启动 Quest APK
├── run_teleop.zsh           # 一键启动 Quest + 双臂遥操作
├── xrt_receive_test.cpp     # PC-service 原始数据诊断程序
├── Makefile
└── .gitignore
```

`demo` 是改动后的工程；`RemoteCtrl-master` 原工程没有被修改。

## 环境与编译产物隔离

- Python 运行环境固定在 `demo/.venv`。
- `xrobotoolkit-sdk`、`sros-sdk-py` 和 pip 下载缓存都在 `demo` 内。
- ROS 2 本身仍使用 `/opt/ros/humble`；venv 通过 `--system-site-packages` 只读访问
  ROS 2 的系统 Python 包。
- 节点启动时设置 `PYTHONNOUSERSITE=1`，不会读取或写入用户的全局
  `~/.local/lib/python*/site-packages`。
- colcon 输出固定在 `demo/build`、`demo/install`、`demo/log`。
- colcon 和 ROS 缓存固定在 `demo/.colcon`、`demo/.ros`、`demo/log/ros`。
- `XRToolKit/build`、`XRToolKit/install`、`XRToolKit/log` 不再使用。
- `build.zsh` 检测到仓库根目录存在这些编译目录时会拒绝编译，要求先执行清理。
- xrt bridge 和 AMR Python 节点由 `demo/.venv/bin/python` 显式启动，不依赖系统
  Python 的第三方包。

清理所有编译和运行生成物，但保留工程 venv：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./clean.zsh
```

如需连同 venv 一起彻底删除：

```zsh
./clean.zsh --venv
```

## 数据链路

```text
Quest 3 XRoboToolkit APK
        │ Wi-Fi
XRoboToolkit PC-service (JAKA 主机)
        │ xrobotoolkit_sdk
    xrt_teleop
        ├─ /rc_ctrl/left_target   (geometry_msgs/PoseStamped)
        ├─ /rc_ctrl/right_target  (geometry_msgs/PoseStamped)
        └─ /rc_ctrl/button        (sensor_msgs/Joy)
                ├─ jaka_driver/rc_ctrl_node x2 -> JAKA 左/右臂 + PGI 夹爪
                └─ sr_amr_control             -> AMR 底盘
```

桥接节点以 125 Hz 读取 SDK，沿用旧工程的位姿低通滤波、坐标映射、手柄按键布局和
ROS 话题。控制消费端无需感知输入已经由旧 APK 换为 XRoboToolKit。

以下命令均按 zsh 编写，使用 ROS 和工作区生成的 `setup.zsh`。

## 固定设备信息

| 设备 | 地址/参数 |
| --- | --- |
| 左 JAKA | `192.168.71.37` |
| 右 JAKA | `192.168.71.36` |
| AMR 底盘 | `192.168.71.50` |
| Quest 3 MAC | `50:99:03:0a:38:78` |
| Quest APK 包名 | `com.xrobotoolkit.client.quest` |

以上 JAKA 和底盘地址沿用原工程，未作修改。

## 安装

以下步骤以 JAKA 主机为 Ubuntu 22.04 + ROS 2 Humble + ARM64 为例。

### 1. 安装对应架构的 PC-service

JAKA 主机是 `aarch64/arm64`，必须安装 PC-service 的 ARM64 包，不能使用
`amd64/x64` 包。

```zsh
uname -m                              # 应为 aarch64
dpkg --print-architecture             # 应为 arm64
```

从
<https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases>
下载 `XRoboToolkit-PC-Service_1.0.0.0_arm64.deb` 后安装：

```zsh
sudo apt-get update
sudo apt-get install -y build-essential
sudo dpkg -i XRoboToolkit-PC-Service_1.0.0.0_arm64.deb
sudo apt-get -f install
```

安装 Python SDK（`xrobotoolkit_sdk`）：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./install_xrt_sdk.zsh
"$HOME/Documents/dj_main/XRToolKit/demo/.venv/bin/python" \
  -c "import xrobotoolkit_sdk; print('XRT SDK OK')"
```

安装脚本优先从 PyPI 安装官方预编译的 `xrobotoolkit-sdk`；如果预编译包不可用，
再使用或克隆
`demo/.deps/XRoboToolkit-PC-Service-Pybind` 中的源码执行
`demo/.venv/bin/python -m pip install .`。官方当前版本不再提供
`setup_ubuntu.sh`。

安装脚本会自动创建 `demo/.venv`，并同时安装 AMR SDK；不会运行
`pip install --user`。

### 2. 安装 AMR SDK 与 ROS 依赖

AMR wheel 已由 `install_xrt_sdk.zsh` 安装到 `demo/.venv`，不需要再安装到系统
Python。可这样验证：

```zsh
"$HOME/Documents/dj_main/XRToolKit/demo/.venv/bin/python" \
  -c "import xrobotoolkit_sdk, sros_sdk_py; print('project env OK')"
```

JAKA SDK 所在的 `demo/src/jaka_driver/lib/libjakaAPI.so` 是 ARM64 版本，并且与复制
过来的 JAKA 头文件匹配。若更换机械臂控制器 SDK，必须同时替换匹配版本的库，并保持
架构为 `aarch64`：

```zsh
file "$HOME/Documents/dj_main/XRToolKit/demo/src/jaka_driver/lib/libjakaAPI.so"
```

### 3. 编译

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./clean.zsh
./build.zsh
source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"
```

JAKA SDK 是 ARM64 二进制，不能在普通 x86_64 电脑上链接出可在 JAKA 主机运行的节点。
`install/setup.zsh` 是 `colcon build` 成功后生成的，不能在编译前 source。

## 启动

### 1. 启动 PC-service

保持该终端运行：

```zsh
cd /opt/apps/roboticsservice
./runService.sh
```

### 2. 在 Quest 3 中启动并发送

1. 在头显中打开 `XRoboToolkit`。
2. 输入 JAKA 主机的局域网 IP，可用 `hostname -I` 查看。
3. 打开控制器 Tracking 和 Send。
4. 确认头显与 JAKA 主机在同一可互访网段，并允许 PC-service 所需端口通过防火墙。

推荐直接使用自动 adb 启动脚本。该脚本优先使用已连接设备，其次读取
`demo/.quest_ip`，最后按 MAC 地址查询 ARP 表：

```zsh
"$HOME/Documents/dj_main/XRToolKit/demo/launch_quest.zsh"
```

第一次连接时建议接 USB；脚本会保存 Quest IP 并尝试建立 `5555` 无线 adb。后续运行
不需要再手工执行 adb 命令。若网关隔离广播或 ARP 查询失败，可以显式指定：

```zsh
QUEST_IP=192.168.x.x "$HOME/Documents/dj_main/XRToolKit/demo/launch_quest.zsh"
```

### 3. 先验证 XRoboToolkit 数据

不启动机械臂，先确认头显数据能到 PC-service：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit"
make -C demo run
```

终端出现 `[DATA] First Quest state received` 后，在头显中移动手柄并按键。更详细的
PC-service SDK 路径检查：

```zsh
make -C demo info
make -C demo XRT_SERVICE_ROOT=/opt/apps/roboticsservice
```

### 4. 启动双臂与夹爪

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"

ros2 launch jaka_driver dual_arm_teleop.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  gripper_rs485_channel:=1 \
  gripper_force:=20 \
  gripper_speed:=40 \
  gripper_configure_tio:=false
```

默认 `use_rpy_ctrl:=false`，与旧工程一致，只让手柄平移量控制机械臂位置，机械臂保持
当前姿态。确认位置控制稳定后，可显式加入 `use_rpy_ctrl:=true` 同步手柄姿态。

如果不希望启动时初始化夹爪产生动作：

```zsh
ros2 launch jaka_driver dual_arm_teleop.launch.py \
  gripper_initialize:=false
```

### 5. 启动双臂 + AMR 底盘

```zsh
ros2 launch robot_bringup RC_robot_bringup.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  connect_ip:=192.168.71.50 \
  gripper_rs485_channel:=1 \
  gripper_configure_tio:=false
```

## 手柄映射

| Quest 输入 | ROS Joy 位置 | 功能 |
| --- | --- | --- |
| 左手 Grip | `axes[0]` | 按住控制左 JAKA；松开停止 |
| 右手 Grip | `axes[1]` | 按住控制右 JAKA；松开停止 |
| 左 Trigger | `axes[2]` | 左 PGI 夹爪，0 张开，1 闭合 |
| 右 Trigger | `axes[3]` | 右 PGI 夹爪，0 张开，1 闭合 |
| 左摇杆 | `axes[4..5]` | AMR 前后移动 / 转向 |
| 右摇杆 | `axes[6..7]` | 保留并发布 |
| X / Y | `buttons[0..1]` | 原 AMR 节点的急停 / 解除急停输入 |
| A / B | `buttons[2..3]` | 发布为 Joy 按键 |
| 菜单键 | `buttons[4..5]` | 发布为 `LThU/RThU` |
| 摇杆按下 | `buttons[6..7]` | 发布为 `LJ/RJ` |
| Grip | `buttons[8..9]` | 数字量状态 |
| Trigger | `buttons[10..11]` | 数字量状态 |

左右臂的 Grip 判定阈值为 `0.5`。夹爪默认使用模拟量 Trigger；如改用数字前端按钮，
可启动时加 `gripper_use_button:=true`。

## 坐标与标定

默认使用旧 Quest reader 和 XRoboToolkit 官方 teleop 示例相同的映射：

```text
(x, y, z) -> (-z, -x, y)
```

配置位于 `demo/src/xrt_teleop/config/xrt_bridge.yaml`：

- `headset_to_robot`：3x3 右手正交旋转矩阵，按行排列。
- `tf_translation`：Quest 原点在机器人基坐标系中的平移，单位米。
- `tf_rotation`：Quest 到机器人基坐标系的旋转，顺序为 `[w, x, y, z]`。
- `pose_scale`：平移缩放，默认 `1.0`。
- `smooth_alpha`：0 表示不平滑，越大越跟手。

先保持默认值测试。只有在方向或零点不符合现场安装关系时才修改标定参数。

## 检查话题

```zsh
ros2 topic hz /rc_ctrl/button
ros2 topic echo /rc_ctrl/button --once
ros2 topic echo /rc_ctrl/left_target --once
ros2 topic echo /rc_ctrl/right_target --once
```

头显未跟踪到手柄时，`xrt_teleop` 不发布对应位姿，并发布 Grip=0 使机械臂停止；
Trigger 保持最后一次有效值，避免跟踪中断时夹爪突然动作。Joy 只在值变化时发布，
与旧 `quest_reader` 的行为一致。

## 安全注意事项

- 首次运行前清空双臂运动范围，确认急停可用，人员不要进入机械臂工作空间。
- 先分别测试左臂和右臂，使用低速度、低夹爪力度，再双人复核后同时操作。
- Grip 是离合器：按住时才跟踪，松开后立即停止增量伺服。
- `gripper_initialize:=true` 会让夹爪执行初始化动作；维修或首次接线时先确认安全。
- 修改 `use_rpy_ctrl`、坐标映射或夹爪方向后，先脱离负载测试。
- AMR 底盘的 X/Y 急停逻辑沿用原 `sr_amr_control` 包，上线前必须按现场安全流程验证。

## 常见问题

**`/opt/ros/humble/setup.bash:.:11` 指向当前工作区的 `setup.sh`**

这说明当前终端继承了错误的 `COLCON_CURRENT_PREFIX`，并不一定是 ROS 安装损坏。
在 zsh 中先清除该变量，再从干净环境加载：

```zsh
unset COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.zsh
```

随后执行 `demo/build.zsh`，再 source `demo/setup.zsh`。如果仍然报错，请新开
一个 zsh 终端后执行上述命令，避免旧终端的 `AMENT_PREFIX_PATH` 和 `PYTHONPATH` 继续
污染环境。

**`source: no such file or directory: /path/to/XRToolKit/demo/install/setup.bash`**

`/path/to/XRToolKit` 是占位符，不能原样执行；且本工程现在使用 `.zsh`。先编译，再
使用实际路径：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./build.zsh
source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"
```

**`Package 'jaka_driver' not found`**

这通常表示只加载了 `/opt/ros/humble`，没有加载 `demo/setup.zsh`，或者工作区
从未成功编译。确认下面三条命令分别有输出：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
source "$HOME/Documents/dj_main/XRToolKit/demo/setup.zsh"
ros2 pkg prefix jaka_driver
```

**`colcon: error: unrecognized arguments: --log-base`**

`--log-base` 是 colcon 的全局参数，必须写在 `build` 前面。当前 `build.zsh` 已按以下
顺序调用：

```zsh
colcon \
  --log-base "$PWD/log/build" \
  build \
  --build-base "$PWD/build" \
  --install-base "$PWD/install" \
  --packages-up-to xrt_teleop jaka_driver sr_amr_control robot_bringup
```

如果终端仍报这个错误，说明执行的 `build.zsh` 还是旧版本。检查
`grep -n -- "--log-base" build.zsh`，必要时重新同步工程文件后执行
`./clean.zsh && ./build.zsh`。

**`No module named xrobotoolkit_sdk`**

先执行下面的安装脚本，再用工程 venv 中的 Python 验证：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./install_xrt_sdk.zsh
"$HOME/Documents/dj_main/XRToolKit/demo/.venv/bin/python" \
  -c "import xrobotoolkit_sdk; print('XRT SDK OK')"
```

验证通过后重新执行 `run_teleop.zsh`。不要使用 `/usr/bin/python3` 或
`pip install --user` 验证，本工程运行节点只使用 `demo/.venv`。

**安装时出现 `bash: setup_ubuntu.sh: No such file or directory`**

这是旧安装流程残留的错误。最新版脚本不再调用该文件，而是执行：

```zsh
"$HOME/Documents/dj_main/XRToolKit/demo/.venv/bin/python" \
  -m pip install --upgrade xrobotoolkit-sdk
```

然后通过 `demo/install_xrt_sdk.zsh` 验证 `demo/.venv` 能导入包。

**左臂日志出现 `ERR_INVALID_HANDLER`**

这是左 JAKA 控制器未上电或网络不可达时登录失败。先给左右控制柜上电，确认
`192.168.71.37` 可 ping 通，再重新启动整套 launch。bridge 节点退出时，launch 会同时
关闭双臂节点，因此随后出现的 `rcl context is invalid` 是关停造成的连带日志。

**PC-service 启动失败或在 x86 上运行 ARM64 SDK**

检查 `uname -m`、PC-service 的 `.deb` 架构，以及
`file demo/src/jaka_driver/lib/libjakaAPI.so`。JAKA 主机上两者都必须是 ARM64。

**有 Joy 数据但没有位姿**

确认 Quest 中已开启 Controller Tracking 和 Send，手柄已被头显识别，且 PC-service
与头显处于可互访网络。

**机械臂不动**

确认对应 Grip 已按住、`/rc_ctrl/{left,right}_target` 有数据、JAKA 登录和使能日志
正常，并检查 `enable_y_limit` 与机械臂当前 TCP 位置。

**夹爪不动**

确认 `gripper_rs485_channel` 与实际 TIO 接线一致（默认 `1` = RS485L/TIO 通道 2），
确认 PGI 站号和波特率，并检查 JAKA App 中的 TIO 配置。
