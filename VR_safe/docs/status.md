# 实现状态（2026-11-20）

| 工单 | 状态 | 证据 | 未完成 |
|---|---|---|---|
| 01 项目壳与离线 bootstrap | 代码完成 | `colcon build` 9 包通过；`.venv` 独立；无兄弟目录 import；`vendor/wheels` + `models/` | 完整离线依赖需预置 wheel；当前 `VR_SAFE_OFFLINE=1` 缺件会明确报错 |
| 02 安全闭环 dry-run | 完成 | 13 个状态机测试；`scripts/test_dry_run.sh` PASS；许可变化全部写审计 | — |
| 03 相机观察源 | 代码 + 回放完成 | 58 个测试；MediaPipe 模型加载；无相机时 5 Hz `UNKNOWN`；polygon/reach、标定校验 | 真相机取流、真实录像精度、200 ms 延迟 |
| 04 Quest 遥操作源 | 代码 + dry-run 完成 | 41 个测试；`launch_quest.sh` + 假 adb 测试；15 s 等**启用侧** target（`wait_teleop.py`）；125 Hz Joy、20 Hz 心跳、LJ/Y、X/A、双臂 B | 真 Quest/PC-service HIL；真实位姿轴向/模拟量 |
| 05 双臂执行端 | 代码 + 构建完成 | CMake 构建；`ctest` 1/1；dry-run 探针 11 项；SLOW 1.5 mm、y 限位 110 mm | 真 JAKA login/servo/碰撞/PGI；p95 |
| 06 底盘执行端 | 代码 + dry-run 完成 | 31 个测试；许可门/许可超时看门狗/零速 keepalive/5 次升级/Twist 看门狗/deadman | 真 AMR SRP、MANUAL/ESTOP 状态迁移、p95 |
| 07 HIL 集成与验收 | 脚本 + 文档完成 | `vr_safe.launch.py` 真机装配；`test_real_launch.sh` 冒烟通过；`latency_probe.py`；`status.py`；`docs/hil-checklist.md` | 现场 HIL、p95 ≤ 200 ms、测试矩阵、外参重标 |
| 08 相机窗口 + RJ 恢复 | 代码 + overlay 测试完成 | `vr_safe_camera_view`；15 FPS 压缩图；红/黄/绿框与提示；`RJ` 长按 1.5 s → confirm；`--camera-view/--no-camera-view` | 真相机/Quest 的窗口与 RJ HIL |
| 09 UNKNOWN 策略 + 骨架 | 状态机/overlay 测试完成 | 普通 UNKNOWN 不停止；OCCUPIED 后跟丢仍 HOLD；观察超时/心跳丢失 HOLD；检测人员 33 点骨架；OCCUPIED 2 帧去抖；观察超时救急 0.8 s | 真人/真相机 HIL、误报漏报回归、感知 ≥10 Hz 后收紧 timeout |

## 运行体验

- 真机默认前台：阶段横幅 + 每秒 `permit/obs/L/R/chassis` 状态行；Ctrl-C 停全部。
- 原始日志按启动时间保存为 `log/run_*.log`，`log/latest.log` 指向最新，保留最近 10 次。
- `--background` 后台 + PID + `stop.sh`；`--no-quest` 只跳过 adb，仍等 target。

## 全量测试命令

```bash
cd VR_safe
./scripts/bootstrap.sh
source scripts/env.sh
./scripts/test_all.sh
```

结果：supervisor 20、perception 58、chassis 31、teleop 43、camera view 6、arm ctest 1/1、dry-run PASS、真机 launch 冒烟 PASS。

## 进生产前必须做

1. 重标外参，令 `config/calibration/extrinsic_calib.json` 的 `ok=true`；当前是 `ok=false`，生产 preflight 会拒绝。
2. 按 `docs/hil-checklist.md` 跑现场 HIL，尤其第 3/4 节的“一起停”和 p95。
3. 真 Quest、真 JAKA、真 AMR 的连接/状态/时序验证；当前只有 dry-run 和 build 证据。
4. 与 AMR 安全负责人确认“遥控时关闭 OBA”的既有行为。
