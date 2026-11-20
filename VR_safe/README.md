# VR_safe

人机共存工位的安全遥操作工程：Quest 3 同时遥操作 JAKA 双臂、PGI 夹爪和 AMR 底盘；相机判定人员进入保护区时，双臂和底盘一起停。

本工程与 `../demo`、`../camera_zone`、`../Vision_Teleop` **运行期完全独立**：自己的 `.venv`、ROS 2 工作空间、配置、模型和启动脚本；只复制需要的源码，不改也不 import 兄弟目录。

## 快速开始

### 真机

终端 1，启动 PC-service（每次重启主机后都要做，保持不关）：

```bash
cd /opt/apps/roboticsservice
./runService.sh
```

终端 2，启动 VR_safe：

```bash
cd ~/Documents/dj_main/XRToolKit/VR_safe
./scripts/start.sh --real --allow-uncalibrated
```

`--allow-uncalibrated` 只用于当前外参 `ok:false` 的开发现场；外参有效后去掉。启动后前台每秒显示 `permit/obs/arm/chassis` 状态，默认打开相机窗口。

停止：

```bash
./scripts/stop.sh
```

前台运行时也可以直接 Ctrl-C。

查看状态 / 手动确认：

```bash
python3 scripts/status.py --once
python3 scripts/confirm_resume.py --by 操作员
```

### 无硬件（dry-run / 测试）

```bash
cd ~/Documents/dj_main/XRToolKit/VR_safe
./scripts/bootstrap.sh
./scripts/start.sh --dry-run     # fake 双臂/底盘，跑完自动退出
./scripts/test_all.sh            # 单元 + ctest + dry-run + launch 冒烟
```

dry-run 覆盖 `CLEAR/NEAR/OCCUPIED`、恢复、心跳丢失、急停等安全闭环，最后打印 `PASS`。

## 手柄（真机）

| 输入 | 功能 |
|---|---|
| 左/右 Grip | 对应臂离合 |
| 左/右 Trigger | 对应夹爪 |
| 左摇杆 | 底盘线速度/角速度 |
| X / A | 左右夹爪锁定 |
| 左摇杆按下（LJ） | 系统软件急停 |
| 右摇杆按下（RJ）长按 1.5s | 请求人工确认恢复许可（CLEAR/NEAR 时有效） |
| Y | 解除 LJ 按钮锁存（不恢复许可） |
| 右 B | 双臂协同模式 |

## 目录

```
VR_safe/
  .venv/                  # Python 依赖；不用 --system-site-packages
  src/
    vr_safe_interfaces/   # 安全契约
    vr_safe_supervisor/   # 唯一许可、锁存、审计
    vr_safe_fakes/        # dry-run 的 fake 源/汇 + 场景
    vr_safe_bringup/      # launch / 配置
    vr_safe_perception/   # 相机观察源（工单 03）
    vr_safe_teleop/       # Quest 源 + 心跳 + 急停（工单 04）
    vr_safe_arm/          # 双臂 C++ 执行端（工单 05）
    vr_safe_chassis*/     # AMR 执行端（工单 06）
    vr_safe_camera_view/  # 实时相机窗口 + 恢复提示（工单 08）
  config/                 # 安全/遥操作/底盘/感知配置 + 标定
  models/                 # MediaPipe 模型
  scripts/                # bootstrap / env / start / stop / preflight
  docs/                   # 设计、spec、ADR、工单
  build/ install/ log/    # 本工程构建与运行产物
```

## 实时相机画面

真机启动时如果有 `DISPLAY`/`WAYLAND_DISPLAY`，默认打开 `VR_safe camera` 窗口：

- 红框 = 许可 `REVOKED`；黄框 = `SLOW`；绿框 = `GRANTED`。
- 顶部显示许可状态、原因和观察状态；绿色多边形是保护区。
- 人进入区域时区域变红/填充，底部提示恢复方式。
- `REVOKED` 且当前 `CLEAR/NEAR` 时提示“按 RJ 长按 1.5s 恢复许可”；
  区域有人或“人进过区后跟丢”时提示不能恢复。
- 画面会画出所有检测到的人的简单骨架：区外绿、`NEAR` 黄、`OCCUPIED` 红。
- `UNKNOWN` 单独出现不停止（灰字提示），只有当前 `OCCUPIED` 或“人进过区后跟丢”才锁存。
- `--no-camera-view` 关闭；`--camera-view` 强制开启（无显示会自动跳过）。

## Quest 准备（真机）

PC-service 已启动后，`start.sh --real` 会先调用 `scripts/launch_quest.sh`：通过 USB 或无线 adb 找到 Quest 3，必要时安装并拉起 `vendor/XRoboToolkit-Quest-1.0.1.apk`。首次建议 USB 连一次；之后脚本会缓存 `.quest_ip` 走无线。找不到 Quest 时 `start.sh` 会拒绝启动，避免“臂已使能但没有输入”。

```bash
QUEST_IP=192.168.1.105 ./scripts/launch_quest.sh   # 手动补启动
QUEST_WAIT_UNPLUG=1 ./scripts/launch_quest.sh      # 有 USB 时等拔线（首次建议）
QUEST_DATA_TIMEOUT=30 ./scripts/start.sh --real --allow-uncalibrated  # 改等待秒数
./scripts/start.sh --real --allow-uncalibrated --no-quest  # APK 已在跑，跳过 adb
# 只跑 supervisor + 感知、不使能双臂：
ros2 launch vr_safe_bringup vr_safe.launch.py start_arms:=false start_teleop:=false start_amr:=false
```

头显里必须打开 **Tracking** 和 **Send**；日志出现 `没有已跟踪的 Quest 控制器` 就是这一步没完成。

### Quest 没连上怎么查

1. `adb devices -l` 是否为空？为空 = 电脑根本没看到 Quest，先 USB 连一次或设置 `QUEST_IP`。
2. `.quest_ip` 里的 IP 是否过期？删掉它或 `QUEST_IP=<新 IP>` 重试。
3. `pgrep -af RoboticsServiceProcess` 是否有输出？没有 = PC-service 没跑。
4. 头显里 XRoboToolkit 是否在前台，Tracking / Send 是否打开？
5. 日志 `没有已跟踪的 Quest 控制器` 每 5 s 一条，说明桥活着但头显数据没到；不是安全链路故障。
6. `start.sh --real` 现在会先跑 `launch_quest.sh`，失败就拒绝启动；APK 已在跑时可用 `--no-quest` 跳过 adb。

## 运行形态

- 真机默认**前台**：终端先打印 1/5–5/5 阶段，之后每秒一行
  `permit=... obs=... L=... R=... chassis=...`；原始日志写文件。
  Ctrl-C 会停止全部节点。
- `--background` 转后台，PID 写 `log/vr_safe.pid`，用 `scripts/stop.sh` 停。
- 日志按启动时间保存：`log/run_YYYYmmdd_HHMMSS.log`，`log/latest.log` /
  `log/vr_safe.log` 指向最新；只保留最近 10 次。
- `--no-quest` 只跳过“adb 拉起 APK”，仍会等 Quest target；APK 已在跑时用。

### 按 Grip / 摇杆不动怎么查

先看前台状态行的 `permit`：

- `permit=REVOKED`：安全锁存。等 `obs=CLEAR`，运行
  `python3 scripts/confirm_resume.py --by 操作员`；然后**松开再按 Grip**，
  底盘要摇杆回中并用 Y 重新使能遥控。`Y` 只清 LJ 按钮锁存，**不会恢复许可**。
- `permit=REVOKED(latched):看不清...` 且 `obs=UNKNOWN`：这是“人进过区域后跟丢”，
  不是普通看不清；等画面重新明确为 `CLEAR/NEAR` 后再按 RJ。
- `obs=UNKNOWN` 但 `permit=GRANTED`：普通看不清，按当前策略不停止，注意确认现场。
- `permit=REVOKED:区域被占`：人/手在保护区内，机器人本来就不该动。
- 许可 `REVOKED` 时夹爪也会冻结（保持最后位置/力），这是有意的。

## 安全行为

- 观察、许可、执行三层分离；`vr_safe_supervisor` 是唯一许可持有者。
- `CLEAR→GRANTED`；`NEAR→SLOW`（0.3×）；`OCCUPIED` 或“人进过区后跟丢”→`REVOKED`。
- 普通 `UNKNOWN` 不停止；观察超时/感知源消失/心跳丢失/系统急停仍 `REVOKED`。
- `REVOKED` 锁存，只能 `confirm_resume` 解除；区域有人/跟丢时拒绝；`NEAR` 只恢复到 `SLOW`。
- 双臂恢复后必须重新按 Grip；底盘恢复后必须摇杆回中并重新使能遥控。
- 夹爪在 HOLD/SLOW 下保持最后位置，不自动张开。
- supervisor 失联：臂 300 ms 没有新许可、底盘 300 ms 没有新许可，各自本地看门狗 HOLD；不依赖最后一条 GRANTED。

## 标定

- 区域：`config/calibration/zone_ground.json`（polygon）或 `zone_config.json`（plane/reach）。
- 外参：`config/calibration/extrinsic_calib.json`；`ok:false` 时生产拒绝启动，开发用 `--allow-uncalibrated`。
- 相机/外参重标后要重启 supervisor 和感知节点。

## 状态

- 设计：[docs/design.md](docs/design.md)、[docs/spec.md](docs/spec.md)、[GLOSSARY.md](GLOSSARY.md)、[docs/adr/](docs/adr/)
- 工单：[docs/tickets/](docs/tickets/)
- 已实现并测试：安全契约、supervisor、感知、Quest 遥操作、双臂 C++、底盘、dry-run 闭环、真机 launch 冒烟。
- 实现状态与证据：[docs/status.md](docs/status.md)；HIL 清单：[docs/hil-checklist.md](docs/hil-checklist.md)。
- 真机 HIL 尚未执行：需要现场相机、JAKA 和 AMR，并先重标外参（当前 `ok:false`）。

## 已知限制

- HIL 未在本机执行：需要在现场接真相机、JAKA、AMR，并保持物理急停在旁。
- 保护区在 JAKA base 系，随底盘移动；第一版不做世界系补偿。
- 顶置相机有后方/侧方盲区；AMR 控制器自带 lidar/bumper/急停仍是底盘碰撞主责。
- `xrobotoolkit_sdk` 需要单独安装/打包；`sros_sdk_py` 的 protobuf 版本可能和媒体依赖冲突，bootstrap 要单独解。
- 第一版 AMR 只做遥控/状态/零速，不含到站/路径/充电/定位。
