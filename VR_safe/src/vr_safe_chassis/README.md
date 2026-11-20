# vr_safe_chassis

VR_safe 底盘执行端（工单 06）：AMR 遥控子集 + 许可门 + 零速 keepalive + 状态/里程计。
从 `demo/src/sr_amr_control` 复制后删除了 action、充电、定位、lidar reporter；第一版
不含 `MoveToStation` / `MoveFollowPath` / 充电 / 定位，也不依赖 `move_pause`。

## 运行

```bash
# 真机（配置见 VR_safe/config/chassis.yaml）
ros2 run vr_safe_chassis chassis_node --ros-args \
  -p config:=$HOME/Documents/dj_main/XRToolKit/VR_safe/config/chassis.yaml

# 无 AMR dry-run：不 import/连接 sros_sdk_py，用内存状态/里程计
ros2 run vr_safe_chassis chassis_node --ros-args \
  -p config:=$HOME/Documents/dj_main/XRToolKit/VR_safe/config/chassis.yaml \
  -p dry_run:=true
```

`chassis.yaml` 是纯 YAML（`chassis:` / `topics:` 两节），不是 ROS params 文件。
显式 ROS 参数（如 `dry_run`、`connect_ip`、`slow_scale`）优先于 YAML。未知字段、
类型错误、越界值都会让节点拒绝启动。非 dry-run 模式连不上 AMR 时返回非零退出码。

## 话题与服务

| 名称 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/vr_safe/permit` | `MotionPermit` | 订阅 | RELIABLE + TRANSIENT_LOCAL，depth 1 |
| `/vr_safe/teleop/button` | `sensor_msgs/Joy` | 订阅 | 主输入：左摇杆 + Y；teleop 需按固定频率发布 |
| `/vr_safe/chassis/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 可选外部 Twist，不依赖 teleop 发布 |
| `/vr_safe/chassis/state` | `ChassisState` | 发布 | 10 Hz；REVOKED → `state=hold` |
| `/vr_safe/chassis/system_state` | `SystemState` | 发布 | 从 SRP 复制 |
| `/vr_safe/chassis/battery_state` | `BatteryState` | 发布 | 从 SRP 复制 |
| `/vr_safe/chassis/odom` | `nav_msgs/Odometry` | 发布 | 含 `map→odom→base_link` TF |
| `/vr_safe/chassis/remote_control_enabled` | `std_srvs/SetBool` | 服务 | 使能/禁用遥控；使能即 deadman |
| `/vr_safe/chassis/remote_control_oba_enabled` | `std_srvs/SetBool` | 服务 | 遥控 OBA 开关 |
| `/vr_safe/chassis/emergency_stop` | `std_srvs/Trigger` | 服务 | 维修直连 AMR 急停 |
| `/vr_safe/chassis/release_emergency_stop` | `std_srvs/Trigger` | 服务 | 解除 AMR 急停 |

`ChassisState.detail_json` 带 `dry_run / permit / reason / armed / remote_control_enabled /
stick_centered / zero_failures / escalated / watchdog / last_error / slow_scale`。

## 安全语义

- **REVOKED**：阻断操作员 Twist，立即并每 `zero_keepalive_ms`（默认 100 ms）调用
  `set_remote_control_speed(0,0,0)`；**不退出 MANUAL**，不调用 `emergency_stop`
  （除非零速连续失败），不依赖 `move_pause`。
- **SLOW**：线速度/角速度均乘 `slow_scale`（默认 0.3）；**GRANTED**：正常速度。
- **零速升级**：连续 `max_zero_failures`（默认 5）次零速下发失败 → 调一次
  `emergency_stop()`，`ChassisState.state=estop` 并在 `last_error` 记录原因。
  任意一次零速成功会把连续计数清零。
- **本地看门狗**：`twist_watchdog_ms`（默认 200 ms）没有新 Twist → 自动零速并保持
  零速，直到出现新的 Twist。
- **恢复**：REVOKED 之后必须 (1) 重新使能遥控（Y 或 `remote_control_enabled` 服务，
  即 deadman）且 (2) 摇杆回中，才转发新速度；在此之前持续零速。停止期间偏过杆，
  就一定要再看到一次回中。
- **手柄**：左摇杆驱动底盘；X/A 归夹爪锁定，底盘 `estop_button=-1` 禁用，绝不会
  把 X 当 AMR 急停；Y 用于解除 AMR 急停并重新使能遥控。系统软件急停（LJ）由
  teleop 发 `ESTOP` 观察，经 supervisor 变 REVOKED 到达本节点。
- **未使能遥控时**：底盘不在 MANUAL，SRP 常以 70001 拒绝速度指令。零速仍然立即
  并按 100 ms 下发（满足 REVOKED 的零速语义），但这类预期错误**不计入**“连续
  5 次零速失败”，避免把“操作员还没按 Y”误升级成 AMR 急停；使能遥控后才开始
  计数并把真实下发失败升级为 `emergency_stop`。

## 配置

`VR_safe/config/chassis.yaml` 字段：

| 字段 | 默认 | 说明 |
|---|---|---|
| `dry_run` | false | 内存 SRP，不连 AMR |
| `connect_ip` / `connect_username` / `connect_passwd` | 192.168.71.50 / "" / "" | SRP 连接 |
| `max_linear_speed` / `max_angular_speed` | 0.5 / 0.8 | m/s、rad/s |
| `joy_deadzone` | 0.2 | 左摇杆死区 |
| `slow_scale` | 0.3 | SLOW 许可的缩放 |
| `zero_keepalive_ms` | 100 | 零速重发周期 |
| `twist_watchdog_ms` | 200 | 本地 Twist 看门狗 |
| `max_zero_failures` | 5 | 升级 emergency_stop 的阈值 |
| `speed_resend_ms` | 50 | 相同速度最多按此周期重发，避免 125 Hz Joy 变成 125 Hz SRP 请求 |
| `estop_button` | -1 | 本地急停键，-1 禁用 |
| `estop_release_button` | 1 | Y |
| `gate_tick_period_ms` | 20 | 许可门周期 |
| `state_publish_rate_hz` / `sync_state_period_s` | 10 / 0.3 | 状态发布与 SRP 同步 |

## 测试

```bash
cd VR_safe/src/vr_safe_chassis
python3 -m pytest test -q
```

覆盖许可门（REVOKED 阻断、零速周期、5 次失败升级、Twist 看门狗、SLOW 缩放、
deadman + 回中恢复）、Joy 映射/按键下标、配置校验、内存 SRP。测试只用假 SRP
和显式假时间，不需要 ROS、不需要 AMR。

## 已知限制 / HIL 未验证

- 真实 `SrpClient` 的连接、`set_remote_control_speed`、`emergency_stop`、
  `release_emergency_stop`、OBA 开关尚未接真机验证。
- 100 ms / 200 ms 的实际时序和 p95 延迟需要 HIL 测量；本包只保证逻辑。
- teleop 必须按固定频率（≥ 5 Hz，建议 10–20 Hz）发布 `/vr_safe/teleop/button`，
  否则 200 ms 看门狗会按设计清零。
- 本包不负责 AMR 原生 lidar/bumper/急停，也不宣称相机 360° 覆盖。
