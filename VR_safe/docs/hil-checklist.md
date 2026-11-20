# HIL 验收清单

> 本清单需要在现场执行，不在会议室/CI 里跑。物理急停必须放在操作员手边；第一次测试空载、低速、区域清空。

## 0. 前置

- [ ] 现场外参已重标，`config/calibration/extrinsic_calib.json` 的 `ok` 为 `true`；否则生产 `preflight` 会拒绝。
- [ ] `config/calibration/zone_ground.json` 与实际贴的工位一致（`scripts/env.sh` + `preview_zone` 看过）。
- [ ] 左/右 JAKA 控制器上电、网线接好；`192.168.71.37/36` 可达。
- [ ] AMR `192.168.71.50` 可达；AMR 物理急停/遥控器可用。
- [ ] PC-service 在跑：`cd /opt/apps/roboticsservice && ./runService.sh`（保持终端）。
- [ ] Quest 3 / PC-service 已连接；`xrobotoolkit_sdk` 能 import。
- [ ] `./scripts/launch_quest.sh` 成功：`adb devices` 有 Quest、APK 进程在跑、头显里 Tracking/Send 已开。
- [ ] 物理急停在手边；第一次测试把底盘速度上限调到 ≤0.2 m/s，臂速降到最低。
- [ ] `./scripts/bootstrap.sh` 跑过，`source scripts/env.sh` 后 `python3 scripts/preflight.py --mode real` 通过。

## 1. 无硬件 dry-run

- [ ] `./scripts/test_dry_run.sh` → `PASS`。
- [ ] `./scripts/test_real_launch.sh` → 冒烟通过（节点都起来，不连硬件）。

## 2. 单臂/单底盘

- [ ] `ros2 launch vr_safe_bringup vr_safe.launch.py start_amr:=false start_perception:=false dry_run:=false`：两臂登录、使能、servo 正常；未按 Grip 不动。
- [ ] `ros2 launch ... start_arms:=false start_perception:=false dry_run:=false`：AMR 连接、MANUAL 状态、摇杆低速前进/后退/转向。
- [ ] 按 `Y` 使能遥控；松开摇杆后底盘应停（200 ms 看门狗）。
- [ ] 与 AMR 安全负责人确认“遥控时关闭 OBA”的既有行为；若对方认为 OBA 必须保留，把 `remote_control.py` 的关闭动作改为可配置。
- [ ] 按 `LJ`：supervisor 显示 `REVOKED/system_estop`，双臂 HOLD、底盘零速；`Y` 后仍 `REVOKED`，`confirm_resume` 后才恢复。

## 3. 相机 + 双臂 + 底盘

- [ ] 正常启动：`./scripts/start.sh --real`；默认前台，每秒一行状态；最多等 15 s 的**启用侧** target，收到才继续；超时会停止并退出。
- [ ] 另一个终端看原始日志：`tail -f log/latest.log`；日志按启动时间保留最近 10 次。
- [ ] 观察 `CLEAR` 时许可 `GRANTED`。
- [ ] 人靠近边界：`NEAR` → `SLOW`；臂/底盘速度约 0.3×；人离开自动回 `GRANTED`。
- [ ] 人进入区域：`OCCUPIED` → `REVOKED`；两臂 `command_blocked`、底盘 `hold_active`；同时停。
- [ ] 区域有人时 `confirm_resume` 被拒；人离开后确认成功；双臂必须重新按 Grip、底盘摇杆回中并重新使能。
- [ ] 恢复后第一帧不跳变：记录恢复前 TCP/手部位姿，恢复后 1 s 内位移应连续。
- [ ] 普通 `UNKNOWN`（画面里没检出人/关键点不足）：许可保持 `GRANTED`，不停止；画面灰字提示。
- [ ] 人先进区（`OCCUPIED`）再被挡住/跟丢：`UNKNOWN` 仍 `REVOKED`；重新 `CLEAR/NEAR` 后按 RJ 恢复。
- [ ] 拔掉相机/停感知进程：800 ms 内 `REVOKED`（reason 观察超时；救急值）。
- [ ] 停 PC-service/Quest 桥：300 ms 内 `REVOKED`（reason teleop 心跳丢失）。
- [ ] `LJ` 系统急停：全停；`Y` 只清按钮锁存；确认 + 重握后才恢复。
- [ ] 相机窗口：`permit=GRANTED` 绿框；人进入区域变红/填充；普通 `UNKNOWN` 不红框，显示“不停止”灰字。
- [ ] `REVOKED` 且 `CLEAR/NEAR` 时，画面提示“按 RJ 长按 1.5s 恢复”；按 RJ 1.5 s 后确认成功；区域有人/跟丢时按 RJ 被拒并在画面显示原因。
- [ ] 杀 supervisor：两臂本地看门狗 HOLD、底盘零速；重启后不自动恢复。
- [ ] 单臂故障（dry_run 断开一路/真机断网）：该臂 blocked，不影响另一臂的许可门；记录 `ArmState`。

## 4. 延迟

- [ ] 另开终端：`source scripts/env.sh && python3 scripts/latency_probe.py --events 10`
- [ ] 人反复进入区域，探针输出每次 ms 与 p95。
- [ ] 验收：p95 ≤ 200 ms（观察时间戳/发布 → 双臂 blocked + 底盘 hold）。
- [ ] 记录 CPU/GPU 频率、FPS、是否锁频；如果 p95 超限，先查相机帧龄、推理耗时、ROS 队列和底盘 SRP 同步调用。

## 5. 误报/漏报回归

- [ ] 用同一段录像（`record_clip` 采）跑 `replay_clip` 或 `observation_node --replay`，记录五档分布。
- [ ] 与 camera_zone 旧结果对照；差异必须能解释（不是站位差异）。
- [ ] 空场景 10 分钟：不得出现 `OCCUPIED`。
- [ ] 人站在区域边缘：不得在 `CLEAR/NEAR/OCCUPIED` 间高频抖动；`Grader` 迟滞生效。

## 6. 审计与退出

- [ ] `log/safety.jsonl` 有 supervisor_start、observation、permit_change、confirm_resume、estop、heartbeat 丢失记录。
- [ ] `log/teleop/*.csv` 有伺服黑匣子。
- [ ] `./scripts/stop.sh` 后相机进程正常退出（不要 kill -9），下次能开相机。
- [ ] 重启后 `LJ`/确认/恢复流程仍正确。

## 通过标准

- 第 3 节所有条目通过；
- 第 4 节 p95 ≤ 200 ms；
- 第 5 节无未解释的误报/漏报；
- 审计日志完整，退出后相机可复用。

## 已知限制（写进交付说明）

- 保护区在 JAKA base 系，随底盘移动；不做世界系补偿。
- 顶置相机有后方/侧方盲区；AMR 控制器自带 lidar/bumper/急停仍是底盘碰撞主责。
- 第一版 AMR 不含到站/路径/充电/定位。
- 外参当前 `ok:false`；未重标前生产 preflight 拒绝启动。
- 200 ms 是软件链路目标，不含机械臂/底盘机械响应和控制器内部延迟；现场以探针结果为准。
