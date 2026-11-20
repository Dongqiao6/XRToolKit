# VR_safe 设计

状态：2026-11-20 设计已对齐，等待开工确认；**尚未实现**。本文是实现和验收的共同依据，改动决策请同时更新 `docs/adr/`。

## 1. 目标

把 `demo` 的 Quest 遥操作（双臂 + PGI 夹爪 + AMR 遥控）和 `camera_zone` 的工位占用判定合进一个独立工程：

- `VR_safe` 有自己的 `.venv`、ROS 2 工作空间、配置、模型和启动脚本；
- 运行期不 import、不启动、不依赖 `demo/`、`camera_zone/`、`Vision_Teleop/` 的任何代码或产物；
- 底盘和双臂可以同时遥操作；
- 相机判定异常时，底盘和双臂一起停；
- 新增观察源、执行端、命令源时，不改既有安全协议。

## 2. 非目标

- 不做世界系协调（底盘移动时机械臂不补偿工具的世界位姿）；
- 不做 AMR 的 `MoveToStation` / `MoveFollowPath` / 充电 / 定位（第一版）；
- 不做 360° 底盘感知，AMR 自带 lidar/bumper/急停仍是底盘碰撞安全的主责；
- 不做视觉遥操作（`Vision_Teleop` 保持独立，只借 `vision_rc` 作为控制层复制基线）；
- 不做云端/远程遥操作，不把 DDS 开到局域网。

## 3. 总架构

```
Quest 3 ──► vr_safe_teleop ──► /vr_safe/teleop/{left,right}_target
                │                    /vr_safe/teleop/button
                │                    /vr_safe/teleop/heartbeat
                └──► ESTOP 观察（LJ）
相机 ──► vr_safe_perception ──► /vr_safe/observation
                                      │
                              vr_safe_supervisor
                          （许可、锁存、心跳、人工复位）
                                      │
                         /vr_safe/permit（RELIABLE + TRANSIENT_LOCAL）
                          ┌───────────┴───────────┐
                    vr_safe_arm ×2          vr_safe_chassis
                    （C++，各连一臂）        （Python，AMR 遥控子集）
```

## 4. 包与职责

| 包 | 语言 | 职责 |
|---|---|---|
| `vr_safe_interfaces` | rosidl | 安全契约：Observation、Permit、Heartbeat、ArmState、ConfirmResume |
| `vr_safe_chassis_interfaces` | rosidl | 从 `sr_amr_interfaces` 复制 `SystemState`、`BatteryState`（不含 action） |
| `vr_safe_perception` | Python | 相机 → 观察；复用 `zone_monitor` 的判定核心与 `camera_backend` |
| `vr_safe_teleop` | Python | Quest SDK → 目标/按键/心跳/ESTOP；基线 `demo/src/xrt_teleop` |
| `vr_safe_supervisor` | Python | 唯一许可、锁存、心跳看门狗、`confirm_resume`、审计事件 |
| `vr_safe_arm` | C++ ×2 | 基线 `Vision_Teleop/src/vision_rc`；加许可门、本地看门狗、重离合 |
| `vr_safe_chassis` | Python | 基线 `demo/src/sr_amr_control` 的遥控/状态子集；加零速 keepalive |
| `vr_safe_bringup` | launch/config | 组合启动、preflight、参数 |

## 5. 安全契约

`vr_safe_interfaces` 只带当前需要的字段：

- `SafetyObservation`：`header, source_id, zone_id, state(CLEAR/NEAR/OCCUPIED/UNKNOWN), confidence, evidence, detail_json`
- `MotionPermit`：`header, decision(GRANTED/SLOW/REVOKED), reason, source, latched, seq`
- `TeleopHeartbeat`：`header, source_id, seq`
- `ArmState`：`header, arm, command_blocked, motion_stopped, tcp_speed_mps, state`
- `ConfirmResume`：`confirmed_by, note` → `accepted, message`

QoS：

| 话题/服务 | QoS |
|---|---|
| `/vr_safe/permit` | RELIABLE + TRANSIENT_LOCAL，depth 1 |
| `/vr_safe/observation` | RELIABLE，depth 1 |
| `/vr_safe/teleop/heartbeat` | RELIABLE，depth 1 |
| `/vr_safe/teleop/{left,right}_target` | RELIABLE，depth 1 |
| `/vr_safe/teleop/button` | RELIABLE，depth 1 |
| `/vr_safe/arm/{left,right}/state` | RELIABLE，depth 1 |
| `/vr_safe/confirm_resume` | 服务 |

## 6. 安全策略

| 条件 | 观察 | 许可 | 执行 |
|---|---|---|---|
| 启动/还没收到观察 | UNKNOWN | REVOKED | HOLD |
| 相机 CLEAR | CLEAR | GRANTED | RUN |
| 相机 NEAR | NEAR | SLOW | SLOW（0.3×） |
| 有人进入区域 / 手伸入 / 头探入 | OCCUPIED | REVOKED | HOLD |
| 画面不可信 / 测不出人，且上次不是 OCCUPIED | UNKNOWN | GRANTED（上次 NEAR 则 SLOW） | RUN/SLOW |
| 人进过区后跟丢（上次 OCCUPIED，现 UNKNOWN） | UNKNOWN | REVOKED（锁存） | HOLD |
| 观察超过 800 ms 不新鲜（救急值；原 300 ms） | STALE | REVOKED | HOLD |
| 许可超过 300 ms 不新鲜（supervisor 失联） | PERMIT_TIMEOUT | REVOKED（本地看门狗） | HOLD |
| 遥操作心跳超过 300 ms | TELEOP_LOST | REVOKED | HOLD |
| 系统软件急停 `LJ` | ESTOP | REVOKED（锁存） | HOLD |
| 人工确认时 CLEAR | CLEAR | GRANTED | 仍需重新 Grip/遥控使能 |
| 人工确认时 NEAR | NEAR | SLOW | 仍需重新 Grip/遥控使能 |
| 人工确认时 OCCUPIED/跟丢 | 拒绝 | REVOKED | HOLD |

- 升档即时，降档沿用 `Grader` 的迟滞；`REVOKED` 只能人工确认解除。
- `SLOW` 不锁存：回到 CLEAR 自动恢复 `GRANTED`。
- `Y` 只解除 `LJ` 急停的按钮锁存；许可恢复仍要 `confirm_resume`。
- 夹爪在 `REVOKED`/`SLOW` 下保持最后指令/位置，不自动张开；`SLOW` 不缩放夹爪速度（**待确认的默认假设**）。

## 7. 控制层

### 双臂

- 每个 `vr_safe_arm` 进程连一台控制器（JAKA SDK 每进程一条连接），基线是 `Vision_Teleop/src/vision_rc`。
- 订阅 `/vr_safe/permit`。`REVOKED` 时：阻断新目标、强制 `gripped=false`、持续零增量、保持伺服使能。
- 本地看门狗：`gripped` 时 200 ms 没有新 target，按“跟踪丢失”处理，松开离合 + HOLD。
- 许可恢复后不自动跟随：操作者必须重新按 Grip，控制节点重新锚定当前位姿。
- 继续保留：Grip 迟滞、5 mm/周期限速、y 向 ±110 mm 限位、碰撞检测、`servo_hold`。

### 底盘

- `vr_safe_chassis` 只搬 `sr_amr_control` 的遥控、状态/里程计、急停子集；不带 action/充电/定位。
- 订阅 `/vr_safe/permit`。`REVOKED` 时：抑制操作员 `Twist`，立即并每 100 ms 调 `set_remote_control_speed(0,0,0)`；保持 MANUAL 模式。
- 连续 5 次零速失败：升级 AMR `emergency_stop`。
- 本地看门狗：200 ms 没有新 `Twist`，自动零速。
- 恢复：摇杆回中 + 重新使能遥控（deadman），才转发新的速度。
- 系统软件急停 `LJ`：发布 `ESTOP` 观察 → supervisor `REVOKED`；不走 AMR `emergency_stop`（除非零速连续失败）。

### 手柄

| 输入 | 功能 |
|---|---|
| 左/右 Grip | 对应臂离合 |
| 左/右 Trigger | 对应夹爪 |
| 左摇杆 | 底盘线速度/角速度 |
| `X` / `A` | 左/右夹爪锁定（不变） |
| `LJ`（左摇杆按下） | 系统软件急停 |
| `Y` | 解除 `LJ` 按钮锁存 |
| 右 `B` | 双臂协同模式（保留） |

## 8. 感知与标定

- 复用 `zone_monitor` 的核心：三维柱体、`BODY_PARTS=upper`、飞点治理、骨架反推、运动门、质量门、`Grader`。
- 五档映射四值：空闲→CLEAR、旁边有人→NEAR、看不清→UNKNOWN、手伸入/占用→OCCUPIED。
- 区域：`zone_mode=polygon|reach`，默认 `polygon`；`polygon` 读 `zone_ground.json`，`reach` 用 `jaka_model.reach_spheres`。
- 相机：Orbbec Gemini 335L 为主；RealSense 作为可选后端。
- 姿态后端：默认 MediaPipe CPU；`POSE_BACKEND=gpu` 可选（**默认待确认**）。
- 标定工具：`calibrate_zone.py`、`calibrate_extrinsic.py`、`preview_zone.py`、`record_clip.py`、`replay_clip.py`；不带 rerun/table_zones。
- 标定有效性：外参缺失、`ok:false`、地面差 >0.5°/2 cm，生产拒绝启动；开发 `--allow-uncalibrated`。

## 9. 配置

```
VR_safe/
  config/
    vr_safe.yaml        # 安全策略、超时、slow_scale、zone_mode、标定校验开关
    teleop.yaml         # 映射、频率、平滑、按键
    chassis.yaml        # IP、速度上限、零速周期、看门狗、急停键
    perception.yaml     # 相机、模型、阈值、BODY_PARTS
    calibration/        # zone_config.json, zone_ground.json, extrinsic_calib.json
  src/  build/  install/  log/  models/  scripts/  vendor/wheels/  docs/  .venv/
```

关键默认值：`observation_timeout_ms=800`（现场 CPU 感知约 5 Hz 的救急值，原 300 ms；优化到 ≥10 Hz 后收紧）、`teleop_heartbeat_timeout_ms=300`、`occupied_debounce_frames=2`、`slow_scale=0.3`、`zero_keepalive_ms=100`、`chassis_watchdog_ms=200`、`arm_target_watchdog_ms=200`、`confirm_requires=[CLEAR,NEAR]`、`require_valid_calibration=true`。

## 10. 测试与验收

- **dry-run 必做**：fake `CommandSource`、录像/合成 `Observation`、fake 臂/底盘 `SafetySink`；不接真机也能验证状态机、锁存、恢复、日志。
- **HIL**：真机低速、物理急停在旁；部署前必须跑。
- **指标**：观察时间戳 → 臂零速/底盘零速，p95 ≤ 200 ms。
- **矩阵**：OCCUPIED、观察超时、心跳丢失、`LJ` 急停、确认被拒、恢复不跳变、录像误报回归、supervisor 死亡、单臂故障。
- **回归**：同一段录像在感知算法改动前后必须给出一致的安全结论。

## 11. 部署与运维

- `scripts/bootstrap.sh`：建 `.venv`（不用 `--system-site-packages`）、装 `vendor/wheels`、复制模型、`colcon build`。
- `scripts/start.sh` / `stop.sh`：source ROS Humble、设 `ROS_DOMAIN_ID=42` + `ROS_LOCALHOST_ONLY=1`（可覆盖）、preflight（相机、机器人 IP、AMR IP、PC-service、标定）。
- 不默认 systemd 开机自启；验证后再加。
- 日志：`log/` JSONL 安全事件（许可、观察、确认、心跳丢失、配置哈希）+ 可选 rosbag；保留 30 天或 2 GB 滚动。

## 12. 风险与假设

1. `extrinsic_calib.json` 当前是 `ok:false`；生产启动会被拒绝，必须先重标或开发用 `--allow-uncalibrated`。
2. `xrobotoolkit_sdk 1.1.0` 目前只在 `demo/.venv`，没有现成 wheel；需要从 PyPI 下载或从上游打包，否则遥操作链路无法独立安装。
3. `sros_sdk_py 1.4.1` 要求 `protobuf>=6.33.2`，与 camera_zone 的 `protobuf 5.29.6` 可能冲突；VR_safe 的 venv 必须单独解一次依赖。
4. 底盘移动时工具在世界系随 base 平移，第一版不补偿；操作者要为此负责。
5. 顶置相机有后方/侧方盲区；底盘碰撞安全仍靠 AMR 控制器，不能因为相机而 downgrade。
6. 200 ms p95 尚未实测；MediaPipe CPU 单帧 34~48 ms，链路预算要上机量。
7. 夹爪在 HOLD 下保持最后位置、不自动张开；`SLOW` 不缩放夹爪。若需要“停即张爪”或“停即夹紧”，要改。
8. 启动失败策略：JAKA 臂连接失败中止启动；AMR 连接失败中止启动（`start_amr:=false` 可只跑臂）；相机失败允许启动但许可 `REVOKED`（不会动）。
9. JAKA SDK 每进程一条连接，双臂必须两个进程；单进程双连不做承诺。
10. `move_pause` 只对 action 有效，第一版遥控不用它；零速必须持续重发。
11. 模型/轮子约 1 GB，磁盘 40 GB 可用；`trt_cache` 不复制。
12. 第一版不含 AMR action/充电/定位；以后接入时它们作为额外 `CommandSource`/`SafetySink`，必须同样受许可门约束。

## 13. 实施顺序

1. 骨架：`vr_safe_interfaces`、`vr_safe_supervisor`、`vr_safe_bringup`、配置、fake sources/sinks、dry-run 验收脚本。
2. 感知：搬 `zone_monitor` 核心 + `camera_backend`，接录像回放；标定工具。
3. 遥操作：搬 `xrt_teleop`，改 `/vr_safe/...`，加心跳、`LJ` 急停、按键映射。
4. 双臂：搬 `Vision_Teleop/src/vision_rc`，加许可门、本地看门狗、重离合。
5. 底盘：搬 `sr_amr_control` 遥控子集，加零速 keepalive、看门狗、状态/里程计。
6. 集成 dry-run → HIL → 标定 → 验收 → 文档。
7. 可选：`reach` 区域、RealSense、GPU 姿态、AMR action。
