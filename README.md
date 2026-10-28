# Quest 3 + XRoboToolKit 控制 JAKA 双臂与 AMR 底盘

本工程是 `RemoteCtrl-master` 的 XRoboToolKit 迁移版。Quest 3 不再通过旧 APK 的
adb/logcat 文本传输位姿，而是运行 `XRoboToolkit-Quest-1.0.1.apk`，由
`XRoboToolkit-PC-Service` 接收头显数据；`xrt_teleop` 再把数据转换成本工程原有的
ROS 2 控制话题。JAKA 双臂现在支持位置 XYZ + 姿态 RPY 的 6 维遥操作，PGI 夹爪、
AMR 底盘接口及设备地址保持原样。

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

按住左手柄 Grip 控制左臂，按住右手柄 Grip 控制右臂，松开即停止。按住 Grip 后，
手柄的平移和旋转会同时映射到机械臂末端；左手 Trigger 控制左夹爪，右手 Trigger
控制右夹爪。首次测试应空载、低速，并先分别测试左右臂。

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

如果还需要把 D405 全局画面显示到 Quest 中，另开一个终端运行 D405 视频发送端，并在
Remote Vision 中选择 `REALSENSE_D405`。`run_d405_vision.zsh` 只负责视频，不会启动
JAKA 遥操作；`run_teleop.zsh` 必须同时在另一个终端运行。完整步骤见“D405 低时延
远程视觉”章节。

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
├── setup_d405_profile.zsh   # 向 APK 注入 REALSENSE_D405 视频源
├── install_vision_deps.zsh  # 安装 GStreamer/V4L2 视频依赖
├── run_d405_vision.zsh      # 启动 D405 H.264 发送端
├── d405_vision/             # D405 设备发现、视频源、发送端和配置
├── run_teleop.zsh           # 一键启动 Quest + 双臂遥操作
├── diagnose_xrt.zsh         # 原始手柄位姿、Grip 和 Trigger 诊断
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

远程视觉使用另一条独立链路，不经过 PC-service 的位姿 TCP 通道：

```text
Intel RealSense D405
        │ USB / V4L2
D405 GStreamer 编码器（JAKA 主机）
        │ H.264 / TCP 13579 + 12345
XRoboToolkit Remote Vision（Quest 3）
```

以下命令均按 zsh 编写，使用 ROS 和工作区生成的 `setup.zsh`。

## 固定设备信息

| 设备 | 地址/参数 |
| --- | --- |
| 左 JAKA | `192.168.71.37` |
| 右 JAKA | `192.168.71.36` |
| AMR 底盘 | `192.168.71.50` |
| Quest 3 MAC | `50:99:03:0a:38:78` |
| Quest APK 包名 | `com.xrobotoolkit.client.quest` |
| D405 彩色流 | USB V4L2，默认 `1280x720@30` |
| Remote Vision 控制端口 | TCP `13579` |

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

默认 `use_rpy_ctrl:=true`，即同时控制位置和姿态。姿态采用相对控制：按下 Grip 的
瞬间同时记录手柄位置与姿态作为零点，之后手臂跟随手柄的相对平移和相对旋转；松开
Grip 后位置和姿态都停止更新。

第一次验证旋转时，建议先使用较低旋转增益和角速度：

```zsh
./run_teleop.zsh \
  rotation_scale:=0.3 \
  rotation_gain:=0.3 \
  rotation_max_step_rad:=0.001
```

如果只需要原来的 XYZ 位置控制，可显式关闭姿态：

```zsh
./run_teleop.zsh use_rpy_ctrl:=false
```

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

左右臂 Grip 使用 `0.6` 按下、`0.4` 松开的回差。夹爪默认使用模拟量 Trigger；如改用数字前端按钮，
可启动时加 `gripper_use_button:=true`。

## 6D 旋转控制

旋转桥接链路如下：

```text
Quest 手柄四元数
  -> headset_to_robot 坐标变换
  -> 与 Grip 按下时的姿态求相对旋转
  -> 取最短旋转并限制每周期角度
  -> JAKA servo_p(INCR) 的 rx/ry/rz
```

手柄每帧姿态已在 `xrt_teleop` 中转成机器人基坐标系。默认只映射一半的手柄相对旋转，
并在每个控制周期以较小的角速度跟随。JAKA 驱动会保留尚未完成的旋转，避免快速转动
时直接丢失目标。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rpy_ctrl` | `true` | `true` 为 6D；`false` 只控制 XYZ |
| `rotation_scale` | `0.5` | 手柄相对旋转映射到机械臂的比例，范围 0..1 |
| `rotation_gain` | `0.5` | 每个周期执行剩余旋转的比例，范围 0..1 |
| `rotation_deadband_rad` | `0.005` | 姿态死区，约 0.29° |
| `rotation_max_step_rad` | `0.0015` | 每 8 ms 最大旋转，约 10.7°/s |
| `rotation_sign` | `1` | 全部旋转轴的统一方向；方向整体相反时设为 `-1` |
| `grip_press_threshold` | `0.6` | Grip 达到该值才进入控制 |
| `grip_release_threshold` | `0.4` | Grip 低于该值才释放控制 |

旋转使用机器人基坐标系的增量 RPY。若手柄绕某个轴转动时机械臂三个旋转轴都朝相反
方向运动，可先用 `rotation_sign:=-1` 验证；若只有某两个轴交换或部分方向异常，应
修改 `xrt_bridge.yaml` 的 `headset_to_robot` / `tf_rotation`，不要把 `rotation_sign`
当作单轴反号参数。

本实现参考了 `vr-teleop-kit` 的 clutch-relative 映射、旋转比例缩放和松开后保持
最后目标的思想。当前 JAKA 版本仍使用控制器自身的 `servo_p` 完成笛卡尔到关节的 IK；
`vr-teleop-kit` 的 MuJoCo 解耦 IK 与 DK1 几何直接绑定，不能仅替换 IP 后用于 S5。

现场首测步骤：

1. 机械臂空载，爪内无工件，周围留出完整旋转空间。
2. 先只测试左臂，使用 `rotation_scale:=0.3`、`rotation_gain:=0.3` 和
   `rotation_max_step_rad:=0.001`。
3. 按住 Grip，分别做小角度 roll、pitch、yaw，确认方向和轴对应关系。
4. 松开 Grip 后确认位置和姿态都停止。
5. 确认无误后再测试右臂，再逐步提高旋转比例和速度。

## 松开 Grip 后的保持

JAKA `servo_p` 是直接面向伺服的连续控制接口，只发送一次零增量不能保证队列耗尽后
完全静止。当前驱动在松开 Grip 后继续以 125 Hz 发送全零增量，等价于持续保持最后一
个稳定目标；按下 Grip 后立即停止保持流并开始新的相对运动。

Grip 使用 `0.6` 按下、`0.4` 松开的回差，避免模拟量在 `0.5` 附近波动时反复触发
离合器。若现场 Grip 数值范围不同，可同时调整：

```zsh
grip_press_threshold:=0.6
grip_release_threshold:=0.4
```

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
- 姿态方向由同一坐标矩阵变换；旋转增量随后由 JAKA 驱动发送。

先保持默认值测试。只有在方向或零点不符合现场安装关系时才修改标定参数。

## D405 低时延远程视觉

现有 release APK 的 Remote Vision 可在运行时读取：

```text
/sdcard/Android/data/<APK 包名>/files/video_source.yml
```

因此不需要先重编 APK。工程提供 `REALSENSE_D405` 视频源配置，并沿用 APK 已验证的
H.264 TCP 和 `ZEDMINI` 解码路径。

D405 是单目 RGB 相机，而当前 APK 的平面 Remote Vision 使用左右眼分屏贴图。发送端
会把同一幅 D405 图像复制成左右完全相同的 `2560x720` 侧并排画面，双眼中看到的图像
一致，不会把单幅图像错误地拆给左右眼。它不提供真实立体深度。

### D405 最短运行流程

首次安装依赖并注入视频源：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./install_vision_deps.zsh
./launch_quest.zsh
./setup_d405_profile.zsh
```

`video_source_d405.yml` 修改后必须重新执行 `./setup_d405_profile.zsh`，否则 Quest
仍会使用 APK 数据目录中的旧配置。

自动识别 D405 的彩色节点并实际协商视频管线：

```zsh
./run_d405_vision.zsh --list-devices
./run_d405_vision.zsh --check
```

保持 PC-service 和遥操作运行，在另一个终端启动视频：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./run_d405_vision.zsh
```

最后在 Quest 中打开 Remote Vision，选择 `REALSENSE_D405`，点击 `Listen`，输入运行
`run_d405_vision.zsh` 的 JAKA **主机** IP 并点击 `Confirm`。不能填写 JAKA 控制器
`192.168.71.36/37`、底盘 `192.168.71.50`，也不能填写 Quest 自己的 IP。

### 1. 安装视频依赖

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./install_vision_deps.zsh
```

这会安装 GStreamer、`x264enc`、V4L2 工具和 Python GI 绑定。

### 2. 确认 D405 设备节点

JAKA 主机连接多个摄像头时，不能把 `/dev/video0` 当成固定彩色节点。本次现场机器的
`/dev/video0` 实际只提供 `Z16` 深度流，强制作为 `YUY2` 使用会在 GStreamer 中出现
`reason not-negotiated (-4)`，Quest 只显示 Remote Vision 的绿色占位框。

发送端默认使用 `--device auto`，通过 RealSense D405 的 USB 产品 ID `0b5b` 找到 D405，
再排除 `Z16` 等深度格式，只选择 `YUYV/YUY2`、`UYVY` 或 `MJPG` 彩色节点。其他型号的
RealSense 和普通 USB 摄像头不会进入候选列表。

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./run_d405_vision.zsh --list-devices
```

输出应包含类似内容：

```text
/dev/video0 | D405 | USB 8086:0b5b | ... | no color | formats=Z16
/dev/video2 | D405 | USB 8086:0b5b | ... | YUY2 | formats=Z16, YUYV, MJPG
Selected D405 color source: /dev/video2 (YUY2)
```

实际编号以上述输出为准。只有需要覆盖自动识别时才显式指定：

```zsh
./run_d405_vision.zsh --device /dev/video2
```

下面的 `--check` 会打开所选彩色节点并实际完成一次 GStreamer 协商，不再只是打印
管线文本：

```zsh
./run_d405_vision.zsh --check
```

D405 建议先使用 `1280x720@30`。默认输出为 Remote Vision 所需的
`2560x720@30`、4 Mbps。

`--capture-format AUTO` 会优先使用低延迟的 `YUY2`，没有 `YUY2` 时使用其他已报告
的彩色格式。通常不需要手工指定；需要覆盖时才使用：

```zsh
./run_d405_vision.zsh --capture-format MJPG
```

### 3. 向 Quest APK 添加 D405 选项

先运行 `launch_quest.zsh` 建立 adb 连接，然后执行：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./setup_d405_profile.zsh
```

脚本会自动选择 `com.xrobotoolkit.client.quest` 或
`com.xrobotoolkit.client`，写入 `video_source.yml` 并重新启动 APK。Remote Vision
下拉框中应出现：

```text
REALSENSE_D405
```

如果该选项没有出现，可以继续选择原来的 `ZEDMINI`，发送端也兼容该类型。

`REALSENSE_D405` 的 `camera` 类型在配置中保持为官方发送端和 APK 已验证的 `ZED`；
`REALSENSE_D405` 只是 Remote Vision 中显示和选择的名字。发送端日志会显示实际请求
类型，不需要把 `camera` 改成 `REALSENSE_D405`。

### 4. 启动 D405 发送端

在 JAKA 主机上另开终端：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./run_d405_vision.zsh
```

看到下面日志表示 D405 彩色节点和编码器均已就绪，正在等待 Quest：

```text
Selected /dev/video2 (...), capture=YUY2
In Quest Remote Vision, enter JAKA host IP 192.168.1.20 (...)
Host IPv4 addresses: eth0=192.168.71.10, wlan0=192.168.1.20
Waiting for Quest Remote Vision on 0.0.0.0:13579
```

优先填写上面 `In Quest Remote Vision, enter JAKA host IP` 给出的地址。多网卡机器不能
直接凭网卡名称猜，JAKA 控制网段的 `192.168.71.x` 通常无法被 Quest 访问。

Quest 点击 Confirm 后，发送端必须依次出现：

```text
Remote Vision control connected ...
Control command: OPEN_CAMERA ...
OPEN_CAMERA request: camera=ZED target=192.168.x.x:12345 ...
Streaming D405 to 192.168.x.x:12345
Video sent: frames=1 bytes=... last_frame=... bytes
```

排查时按最后出现的日志判断：

- 没有 `Remote Vision control connected`：Quest 中填错主机 IP，或者 TCP `13579` 被防火墙
  阻止。
- 有连接但没有 `OPEN_CAMERA`：APK 与视频源配置不匹配，重新运行
  `./setup_d405_profile.zsh` 后重启 APK。
- 有 `OPEN_CAMERA` 但没有 `Streaming D405`：发送端选择了错误的 V4L2 节点或 GStreamer
  协商失败，运行 `./run_d405_vision.zsh --list-devices` 和 `--check`。
- 有持续更新的 `Video sent: frames=...` 但 Quest 仍是绿色：网络和相机发送已经正常，
  问题位于 APK 的 H.264 解码/显示路径。先运行下面的测试画面定位。

不打开 D405，只发送标准测试画面：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./run_d405_vision.zsh --test-source
```

测试画面能显示而 D405 不能显示时，问题在相机采集/编码参数；两者都是绿色时，检查
Quest IP 输入、Remote Vision 配置或更换 `ZEDMINI` 项。不要继续反复修改 JAKA 遥操作
代码，因为视频链路与机械臂控制相互独立。

需要确认 APK 解码状态时，在 JAKA 主机另开终端：

```zsh
adb logcat -c
adb logcat | grep -E "MediaDecoder|MediaCodec|RemoteCamera|StartListen|OPEN_CAMERA"
```

正常应看到 `MediaDecoder server started on port 12345` 和
`MediaCodec ... started successfully`；若只看到 `MediaCodec not ready` 或
`Failed to create MediaCodec decoder`，应保存这段日志后再调整编码参数。

如果 CPU 负载较高或 Wi-Fi 延迟大，可降到 `848x480@30` 采集并按需降低码率：

```zsh
./run_d405_vision.zsh \
  --source-width 848 \
  --source-height 480 \
  --fps 30 \
  --bitrate 2000000
```

Jetson/Orin 主机如已安装 NVIDIA GStreamer 编码器，可尝试：

```zsh
./run_d405_vision.zsh --encoder nvv4l2h264enc
```

### 5. 在 Quest 中打开

1. 保持 PC-service 和遥操作正常运行。
2. 在 Quest 的 Remote Vision 中选择 `REALSENSE_D405`。
3. 点击 Listen，输入发送端终端打印的 `In Quest Remote Vision, enter JAKA host IP`。
4. 点击 Confirm。

视频窗口固定在头显前方，因此移动或转头时始终显示 D405 的全局视角。D405 安装方向
决定画面是否倒置；若需要旋转 180°，后续应在发送端增加旋转参数，而不是旋转头显
整体坐标。

### 网络和延迟

- Remote Vision 控制连接使用 TCP `13579`。
- 编码后的 H.264 视频由主机主动连接头显提供的 TCP 端口，通常为 `12345`。
- 保持 JAKA 主机有线连接，Quest 使用 5 GHz Wi-Fi，关闭 Wi-Fi 省电。
- 发送端使用 `zerolatency`、无 B 帧、1 帧以内的关键帧间隔和丢弃旧帧队列。
- 该视频链路与手柄到 PC-service 的位姿链路相互独立，不会阻塞遥操作。

## 检查话题

先排查 Quest 原始输入。运行前停止 `run_teleop.zsh`，避免两个进程同时初始化 SDK：

```zsh
cd "$HOME/Documents/dj_main/XRToolKit/demo"
./diagnose_xrt.zsh
```

未跟踪时输出 `untracked`；抓住手柄时应看到姿态数值持续变化，握住 Grip 时对应
`grip` 接近 `1.000`，按 Trigger 时 `trigger` 接近 `1.000`。按 `Ctrl-C` 结束，再
重新启动 `run_teleop.zsh`。

遥操作启动后，用另一个终端检查 ROS 话题：

```zsh
ros2 topic echo /rc_ctrl/button --once
ros2 topic hz /rc_ctrl/right_target
ros2 topic hz /rc_ctrl/left_target
ros2 topic echo /rc_ctrl/left_target --once
ros2 topic echo /rc_ctrl/right_target --once
```

头显未跟踪到手柄时，`xrt_teleop` 不发布对应位姿，并发布 Grip=0 使机械臂停止；
Trigger 保持最后一次有效值，避免跟踪中断时夹爪突然动作。Joy 只在值变化时发布，
与旧 `quest_reader` 的行为一致。因此不要在静止时用
`ros2 topic hz /rc_ctrl/button` 判断链路是否正常。按住 Grip 后 `rc_ctrl` 日志应出现
对应臂的 `gripped`；松开后应停止更新目标并发送零增量保持。

## 安全注意事项

- 首次运行前清空双臂运动范围，确认急停可用，人员不要进入机械臂工作空间。
- 先分别测试左臂和右臂，使用低速度、低夹爪力度，再双人复核后同时操作。
- Grip 是离合器：按住时才跟踪，松开后立即停止增量伺服。
- 默认启用 6D 姿态控制；首次调试旋转时必须空载并限制
  `rotation_scale` / `rotation_gain` / `rotation_max_step_rad`。
- `gripper_initialize:=true` 会让夹爪执行初始化动作；维修或首次接线时先确认安全。
- 修改 `rotation_sign`、坐标映射或夹爪方向后，先脱离负载测试。
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

先确认 `run_teleop.zsh` 正在运行；D405 发送端本身不会控制机械臂。然后依次检查：

1. `./diagnose_xrt.zsh` 中对应手柄不是 `untracked`，移动手柄时姿态变化，握住 Grip
   时数值从 0 变为接近 1。
2. `ros2 topic hz /rc_ctrl/right_target`（或 `left_target`）有接近 125 Hz 的数据。
3. `rc_ctrl` 日志包含 `gripped`，没有 `servo_p failed`、`ERR_NOT_ENABLED` 或碰撞错误。
4. `enable_y_limit` 与当前 TCP 位置没有阻止向双臂中心线方向的运动。

若原始 `grip` 变化但话题没有变化，检查 APK 的 Send 和 PC-service；若 `/rc_ctrl/button`
变化且有 `gripped`，但姿态话题没有数据，检查 Controller Tracking；若二者都正常但仍
不动，检查 JAKA 登录、上电、使能、伺服模式和安全停止状态。

**夹爪不动**

确认 `gripper_rs485_channel` 与实际 TIO 接线一致（默认 `1` = RS485L/TIO 通道 2），
确认 PGI 站号和波特率，并检查 JAKA App 中的 TIO 配置。
