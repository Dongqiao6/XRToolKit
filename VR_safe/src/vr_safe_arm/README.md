# vr_safe_arm — 双臂执行端（工单 05）

`vr_safe_arm` 是 VR_safe 的 JAKA 双臂执行端。每个进程只连一台控制器，订阅
`/vr_safe/permit`，把 `GRANTED / SLOW / REVOKED` 落实为 `RUN / SLOW / HOLD`，
并以 10 Hz 发布 `ArmState`。

基线代码从 `Vision_Teleop/src/vision_rc` 复制（`rc_ctrl_node.cpp`、PGI-140 协议/
驱动、`jaka_driver` 头文件、`libjakaAPI.so`）。运行期不引用 `Vision_Teleop/`，
也不修改它。

> **JAKA SDK 每进程只允许一条连接。** 双臂必须由 `dual_arm.launch.py` 起两个
> 进程；不要尝试在一个进程里 `login` 两台臂。

## 1. 构建

```bash
cd VR_safe
colcon build --packages-select vr_safe_arm
source install/setup.bash
```

`lib/libjakaAPI.so` 约 49 MB，安装到 `lib/vr_safe_arm/`；可执行文件用
`INSTALL_RPATH=$ORIGIN` 在同目录解析它，不依赖兄弟工程。

## 2. 启动

双臂（无硬件时加 `dry_run:=true`）：

```bash
ros2 launch vr_safe_arm dual_arm.launch.py \
  robot_left_ip:=192.168.71.37 \
  robot_right_ip:=192.168.71.36 \
  dry_run:=false \
  permit_topic:=/vr_safe/permit \
  slow_scale:=0.3
```

单臂（调试用）：

```bash
ros2 launch vr_safe_arm rc_ctrl.launch.py arm_name:=right robot_ip:=192.168.71.36
```

`dual_arm.launch.py` 支持 `robot_left_ip`、`robot_right_ip`、`dry_run`、
`permit_topic`、`slow_scale`，以及 `button_topic`、`target_watchdog_ms`、
`state_publish_rate_hz`、`use_rpy_ctrl`、`enable_y_limit`、`debug`、
`servo_blackbox_dir`。

## 3. 话题与 QoS

| 方向 | 话题 | 类型 | QoS |
|---|---|---|---|
| 订阅 | `/vr_safe/permit`（`permit_topic`） | `vr_safe_interfaces/MotionPermit` | RELIABLE + TRANSIENT_LOCAL，depth 1 |
| 订阅 | `/vr_safe/teleop/{left,right}_target` | `geometry_msgs/PoseStamped` | RELIABLE，depth 1 |
| 订阅 | `/vr_safe/teleop/button` | `sensor_msgs/Joy` | RELIABLE，depth 1 |
| 发布 | `/vr_safe/arm/{left,right}/state` | `vr_safe_interfaces/ArmState` | RELIABLE，depth 1，10 Hz |

`ArmState` 字段：

- `command_blocked`：许可 `REVOKED` 或驱动未就绪；
- `motion_stopped`：实测 TCP 速度 < `stopped_speed_mps`；速度无效时用
  “已松离合且 hold stream 在发零增量”兜底；
- `tcp_speed_mps`：10 Hz 采样两端 TCP 差分得到，dry-run 下用模拟 TCP；
- `state`：本节点状态机名字（见下表）。

## 4. 许可门行为

| 许可 | 新 target | 离合 / 增量 | 平移/姿态步长 |
|---|---|---|---|
| 启动/未收到许可 | 阻断 | `gripped=false`，持续零增量 | — |
| `REVOKED` | 阻断 | 强制松离合，hold stream 持续零增量；伺服保持使能 | — |
| `SLOW` | 跟随 | 保留当前跟踪 | 按 `slow_scale`（默认 0.3）缩放 |
| `GRANTED` | 跟随 | 保留当前跟踪 | 1.0 |

- `REVOKED` 不做 `servo_move_enable(false)`，臂保持使能并由零增量流托住。
- 从 `REVOKED` 恢复后**不会自动跟随**。即使操作者一直按着 Grip，也必须先松开、
  再重新按，节点才会重新捕获当前 TCP/手柄位姿并重新锚定（无跳变）。
- `SLOW → GRANTED`、`GRANTED → SLOW` 不打断跟踪，不需要重新按 Grip。

状态机名字：

| state | 含义 |
|---|---|
| `disconnected` | 真机模式 login/上电/使能失败，节点不发运动 |
| `blocked` | 许可 `REVOKED`，保持位置，等重新按 Grip |
| `arming` | 后台重锚定中（上电/使能/伺服恢复 + 捕获原点） |
| `tracking` | `GRANTED` 且正在跟随 |
| `tracking_slow` | `SLOW` 且正在跟随（步长已缩放） |
| `hold` | 已松离合/看门狗触发，零增量保持位置 |

dry-run 模式下所有状态加 `dry_run/` 前缀，例如 `dry_run/blocked`、
`dry_run/tracking`。

## 5. 本地看门狗

`gripped` 状态下超过 `target_watchdog_ms`（默认 200 ms）没有收到新 target：

1. 立即强制 `gripped=false`；
2. 打开 hold stream，持续发零增量；
3. 立即发布一次 `ArmState`；
4. 置“必须重新按 Grip”门闩：即使操作者还按着，也要先松开再按才能恢复跟踪。

看门狗在 125 Hz 的 hold/看门狗定时器里检查，不依赖 target 回调继续运行。

## 6. 保留的基线行为

- Grip 迟滞：按下 > `grip_press_threshold`（0.6），松开 <
  `grip_release_threshold`（0.4）；
- 5 mm/周期平移限速（`servo_p` 8 ms 周期，约 625 mm/s 软件上限）；
- 0.0015 rad/周期姿态限速；
- y 向单侧 ±110 mm 限位（左臂 y > -110，右臂 y < +110），以 Grip 按下时的
  绝对 TCP y 计算；
- 碰撞检测：`set_collision_level(3)`（保留检测，阈值 25N → 75N），
  `collision_recover()` + `clear_error()`；
- `servo_hold` 零增量流、黑匣子 CSV、PGI-140 夹爪锁定/模拟量控制；
- `SLOW/REVOKED` 不改变夹爪指令：不自动张开，也不缩放夹爪速度。

注意：y 限位沿用基线“预测手柄偏移”算法；如果手柄一次性跳变超过 5 mm/周期，
限速会丢失一部分跟随量，实际 TCP 不会瞬间追到边界。这是为了无跳变而保留的
基线语义，HIL 时应按平滑遥操作验收。

## 7. dry-run 与测试

### 7.1 纯逻辑单元测试（不需要 ROS/JAKA/硬件）

```bash
colcon test --packages-select vr_safe_arm
colcon test-result --verbose
```

测试源码在 `test/test_arm_core.cpp`，覆盖：许可阻断、强制松离合、重新按 Grip
门闩、Grip 迟滞、200 ms 看门狗、SLOW 0.3 缩放、5 mm/周期限速、y 限位、
停稳判据和状态名。

### 7.2 dry-run 端到端探针

终端 A：

```bash
ros2 launch vr_safe_arm rc_ctrl.launch.py dry_run:=true
```

终端 B：

```bash
python3 src/vr_safe_arm/test/dry_run_check.py --arm left
# 或：ros2 run 的情况下用安装后的脚本路径
```

探针发布 permit / Joy / target，逐项验证：

1. 启动默认 `REVOKED` / `command_blocked`；
2. `GRANTED` 后未按 Grip 不跟随；
3. 按 Grip 进入 `tracking`；
4. `REVOKED` 立即 `blocked`、松离合、`motion_stopped=true`；
5. 恢复许可但 Grip 仍按着时不自动跟随；
6. 松开再按后重新锚定；
7. 停发 target 200 ms 触发看门狗；
8. 看门狗后必须重新按 Grip；
9. 未按 Grip 时经历 `REVOKED`，恢复后第一次按下即可重新跟踪（不白吃一次按压）；
10. `SLOW` 保持 `tracking_slow`；
11. `ArmState` 字段/话题正常。

也可以用黑匣子验证限速/限位：把 `servo_blackbox_dir` 指到一个目录，dry-run 下
每次“本应发给 `servo_p` 的增量”都会写成 CSV（`kind=servo`，
`reason=dry_run_sent`）。本工单自测中实测：

- 正常阶段最大增量 5.0000 mm；
- SLOW 阶段最大增量 1.5000 mm（5 mm × 0.3）；
- 平滑向 -y 走 150 mm 时累计增量停在 -110.000 mm。

## 8. 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `arm_name` | `left` | `left` / `right`，决定默认话题和 Grip 轴 |
| `robot_ip` | left `.37` / right `.36` | JAKA 控制器 IP |
| `dry_run` | `false` | 跳过 JAKA/PGI 调用，保留完整安全逻辑 |
| `permit_topic` | `/vr_safe/permit` | 许可话题 |
| `slow_scale` | `0.3` | `SLOW` 平移/姿态步长缩放 |
| `target_watchdog_ms` | `200.0` | 本地 target 看门狗 |
| `state_publish_rate_hz` | `10.0` | `ArmState` 发布频率（上限 100） |
| `stopped_speed_mps` | `0.005` | `motion_stopped` 阈值 |
| `grip_press_threshold` | `0.6` | Grip 迟滞上沿 |
| `grip_release_threshold` | `0.4` | Grip 迟滞下沿 |
| `enable_y_limit` | `true` | ±110 mm y 限位开关 |
| `use_rpy_ctrl` | `true` | 6D 姿态跟随 |
| `rotation_scale/gain/...` | 基线值 | 姿态跟随参数 |
| `servo_blackbox_dir` | `log/teleop` | 黑匣子目录，空字符串关闭 |

## 9. 验证状态与 HIL 注意事项

已在本机（aarch64，ROS 2 Humble）完成：

- `cmake` 独立配置 + 编译到 `/tmp`：零 warning；
- `test_arm_core`：全部通过；
- 单节点 dry-run 端到端探针：11 项全过；
- `dual_arm.launch.py dry_run:=true`：左右两个进程都启动，右侧
  `/vr_safe/arm/right/state` 实测为 `dry_run/blocked`；
- 安装后用 `ldd` 确认 `libjakaAPI.so` 解析到包内安装目录（`$ORIGIN`）。

**未验证（需要现场 HIL）：**

- 真实 JAKA 的 login/power_on/enable/servo、碰撞恢复、PGI-140 夹爪；
- 10 Hz `get_tcp_position` 对 8 ms servo 流的延迟影响（若某控制器该调用较慢，
  需改用状态反馈或降低采样频率）；
- `tcp_speed_mps`/`motion_stopped` 阈值在真实振动和负载下的表现；
- p95“观察时间戳 → 臂零速”≤ 200 ms（需要 supervisor + 真相机/臂联调）；
- 姿态缩放、y 限位在真实手柄平滑输入下的边界手感；
- 双进程同时运行时的网络/会话稳定性。

HIL 建议：先 `dry_run:=true` 验证 launch/话题，再用真机低速、物理急停放在手边，
分别测试 `GRANTED` 跟踪、`SLOW` 降速、`REVOKED` 保持、重新按 Grip、停发 target
看门狗和单臂断网。

## 10. 许可与来源

- `rc_ctrl_node.cpp`、PGI-140 驱动/协议、`jaka_driver` 头文件来自
  `Vision_Teleop/src/vision_rc`（MIT，见上游包）。
- `JAKAZuRobot.h` 版权归 JAKA Robotics（Apache-2.0）；`libjakaAPI.so` 是
  JAKA 提供的二进制 SDK。
- `include/Eigen` 为 Eigen 头文件（MPL2）。
