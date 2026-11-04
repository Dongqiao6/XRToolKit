# Quest 3 遥操作 JAKA 双臂与 AMR

本仓库把 Quest 3 / XRoboToolkit 的位姿与按键数据转换为 ROS 2 控制话题，
驱动 JAKA 双臂、PGI 夹爪和 AMR 底盘。默认支持 XYZ + RPY 六维遥操作。

项目同时记录伺服黑匣子：每次 `servo_p` 都保存发出的增量、目标位置、实际
TCP、六个关节角、控制器错误码和 SDK 返回值。停止运行后可直接生成诊断报告。

## 快速开始

### 1. 环境

目标环境为 JAKA 主机上的 Ubuntu 22.04、ROS 2 Humble 和 ARM64。主机架构必须是
`aarch64`，JAKA SDK 不能链接到普通的 x86_64 机器上运行。

安装 XRoboToolkit PC-service 的 ARM64 包：

```zsh
sudo apt-get update
sudo apt-get install -y build-essential
sudo dpkg -i XRoboToolkit-PC-Service_1.0.0.0_arm64.deb
sudo apt-get -f install
```

发行包从
<https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases>
下载。然后安装 Python SDK、AMR SDK 和项目虚拟环境：

```zsh
cd /path/to/XRToolKit/demo
./install_xrt_sdk.zsh
```

### 2. 编译

```zsh
cd /path/to/XRToolKit/demo
./clean.zsh
./build.zsh
source ./setup.zsh
ros2 pkg prefix jaka_driver
```

最后一条命令必须输出当前工作区中 `jaka_driver` 的安装路径。没有输出时不要启动
机械臂。

### 3. 启动 PC-service

终端 1：

```zsh
cd /opt/apps/roboticsservice
./runService.sh
```

保持该终端运行。

### 4. 启动 Quest、双臂和可选 AMR

终端 2：

不启动 AMR：

```zsh
cd /path/to/XRToolKit/demo
./run_teleop.zsh \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  gripper_rs485_channel:=1 \
  gripper_force:=20 \
  gripper_speed:=40 \
  gripper_configure_tio:=false \
  use_rpy_ctrl:=false \
  start_amr:=false
```

同时启动 AMR：

```zsh
cd /path/to/XRToolKit/demo
./run_teleop.zsh \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  connect_ip:=192.168.71.50 \
  gripper_rs485_channel:=1 \
  gripper_force:=20 \
  gripper_speed:=40 \
  gripper_configure_tio:=false \
  use_rpy_ctrl:=false \
  start_amr:=true
```

脚本会启动 Quest 中的 XRoboToolkit，并确认 APK 进程和无线 adb 均正常。
如果 JAKA 主机与 Quest 之间仍有 USB 数据线，脚本会提示拔线，等待 USB 断开并
复检无线连接后才启动遥操作；拔线后按 Enter 继续。无线或 APK 状态异常时，
机械臂不会启动。

`start_amr` 默认为 `false`，只启动双臂；设为 `true` 时同时启动 AMR。
README 的启动命令默认设置 `use_rpy_ctrl:=false`，只控制位置；需要六维姿态时改为 `true`。
启动 AMR 后，先按一次**左手柄 Y** 开启底盘遥控，再推**左手柄摇杆**：
上下控制前进/后退，左右控制转向。**左手柄 X** 是软件急停。

首次自动连接建议插一次 USB。若无线 adb 不可用，可显式指定 Quest IP：

```zsh
QUEST_IP=192.168.1.100 ./run_teleop.zsh
```

启动前必须给左右 JAKA 控制器上电。出现
`ERR_INVALID_HANDLER` 时，先检查控制器电源和网络，再重新启动。

## 操作方式

| Quest 输入 | 功能 |
| --- | --- |
| 左/右 Grip | 按住控制对应手臂，松开后保持最后目标 |
| 左/右 Trigger | 控制对应 PGI 夹爪 |
| 左手柄摇杆 | AMR 启动后：上下前进/后退，左右转向 |
| 左手柄 X / Y | AMR 软件急停 / 解除急停并开启遥控 |

按住 Grip 后，手柄的相对平移和旋转会映射到机械臂末端。首次测试应空载、低速，
并先分别验证左右臂。

启用六维姿态控制：

```zsh
./run_teleop.zsh use_rpy_ctrl:=true
```

降低首次旋转测试的比例和速度：

```zsh
./run_teleop.zsh \
  rotation_scale:=0.3 \
  rotation_gain:=0.3 \
  rotation_max_step_rad:=0.001
```

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_rpy_ctrl` | `false` | README 示例默认只控制 XYZ；设为 `true` 启用 RPY |
| `rotation_scale` | `0.5` | 手柄旋转到机械臂旋转的比例 |
| `rotation_gain` | `0.5` | 每个周期执行剩余旋转的比例 |
| `rotation_max_step_rad` | `0.0015` | 每周期最大旋转量 |
| `grip_press_threshold` | `0.6` | Grip 按下阈值 |
| `grip_release_threshold` | `0.4` | Grip 释放阈值 |
| `enable_y_limit` | `true` | 限制双臂越过中线的 Y 方向运动 |
| `start_amr` | `false` | `true` 时同时启动移动底盘 |
| `connect_ip` | `192.168.71.50` | AMR 控制器地址 |

## 伺服记录与一页报告

记录默认开启，不需要额外命令。每次启动会在当前运行目录下创建：

```text
log/teleop/left_<epoch-ms>.csv
log/teleop/right_<epoch-ms>.csv
```

从 `demo/` 启动时，完整路径就是 `demo/log/teleop/`。CSV 同时包含：

- 每个伺服周期的指令增量、目标 TCP 和实际 TCP
- 六个关节角及反馈是否有效
- 控制器 `errcode`、`servo_p` 返回值和采集耗时
- 控制器报警产生/清除的独立事件行

测试结束后按 `Ctrl-C`，生成全部报告：

```zsh
cd /path/to/XRToolKit
python3 demo/tools/teleop_report.py demo/log/teleop
```

也可以只分析一个文件：

```zsh
python3 demo/tools/teleop_report.py demo/log/teleop/right_1761888123456.csv
```

报告只使用 Python 标准库，不需要 ROS、机械臂或重新编译。每只手臂依次输出：

| 项目 | 含义 |
| --- | --- |
| `ALARM` | 报警时间和控制器错误码 |
| `SEND` | 伺服发送间隔、漏发和重叠窗口 |
| `LAG` | 指令目标与实际 TCP 偏差较大的时间段 |
| `STALL` | 有运动指令但 TCP 几乎不动的时间段 |
| `FB` / `JRATE` | 关节反馈刷新周期和各关节峰值角速度 |

报警发生后不要立刻删除 CSV；报告会把报警前后的伺服轨迹放在同一会话中分析。

## 固定设备地址

| 设备 | 默认地址 |
| --- | --- |
| 左 JAKA | `192.168.71.37` |
| 右 JAKA | `192.168.71.36` |
| AMR 底盘 | `192.168.71.50` |
| Quest 3 MAC | `50:99:03:0a:38:78` |

Quest 3 使用 PC-service 自动发现，不需要每天配置 MAC。

## 目录与数据流

```text
Quest 3 -> XRoboToolkit PC-service -> xrt_teleop
                                      |-- /rc_ctrl/{left,right}_target
                                      `-- /rc_ctrl/button
                                             |-- jaka_driver -> JAKA + PGI
                                             `-- sr_amr_control -> AMR
```

```text
demo/
├── src/
│   ├── xrt_teleop/          # Quest SDK -> ROS 2 桥
│   ├── jaka_driver/         # JAKA 双臂、夹爪和伺服记录
│   ├── robot_bringup/       # 双臂 + AMR 整体启动
│   └── ...
├── tools/teleop_report.py   # CSV 黑匣子离线报告
├── build.zsh
├── clean.zsh
├── setup.zsh
├── install_xrt_sdk.zsh
├── launch_quest.zsh
└── run_teleop.zsh
```

编译产物固定在 `demo/build`、`demo/install` 和 `demo/log`。Python 依赖固定在
`demo/.venv`，不会写入用户的全局 Python 环境。

## 常见问题

**`Package 'jaka_driver' not found`**

工作区未编译，或当前终端没有加载它：

```zsh
cd /path/to/XRToolKit/demo
./build.zsh
source ./setup.zsh
ros2 pkg prefix jaka_driver
```

**`No module named xrobotoolkit_sdk`**

重新执行 `./install_xrt_sdk.zsh`，并确认运行节点使用的是 `demo/.venv`。

**有 Joy 数据但没有手臂位姿**

确认 Quest 中的 Controller Tracking 和 Send 已打开，手柄已被识别，且头显与
JAKA 主机网络互通。

**机械臂不动**

检查对应 Grip 是否按住、目标话题是否有数据、JAKA 登录和使能日志是否正常，以及
`enable_y_limit` 是否挡住了当前运动方向。

**夹爪不动**

核对 `gripper_rs485_channel`、PGI 站号、波特率和 JAKA App 中的 TIO 配置。

## 安全

- 首次运行前清空双臂运动范围，确认急停可用，人员不要进入工作空间。
- 先分别测试左右臂，确认方向和零点正确后再同时操作。
- 首次测试 6D 姿态时保持空载，并降低旋转比例和最大步长。
- `gripper_initialize:=true` 会使夹爪执行初始化动作，接线或维修后先确认安全。
- AMR 急停逻辑上线前必须按现场安全流程单独验证。
