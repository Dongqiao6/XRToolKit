# 04: Quest 遥操作源 + 心跳 + 系统急停

**What to build:** 独立的 Quest 桥：把 XRoboToolkit 手柄位姿/按键翻译成 `/vr_safe/teleop/*` 目标、按键和心跳；保留 Grip 离合、双臂 B 模式、X/A 夹爪锁定；`LJ` 产生系统急停观察，`Y` 只解除按钮锁存。

**Blocked by:** 02

**Status:** implemented（代码/dry-run/测试）；Quest HIL 待现场

- [ ] 目标、按键、心跳都发布到 `/vr_safe/teleop/*`，不占用旧 `/rc_ctrl/*`。
- [ ] Grip/trigger/摇杆映射与 demo 一致；右 B 双臂模式保留。
- [ ] `LJ` 产生 `ESTOP` 观察；`Y` 只清按钮锁存，不直接恢复许可。
- [ ] 跟踪丢失时 grip 归零、target 停止发布，心跳继续（进程活着）。
- [ ] 合成输入 dry-run 能验证目标/按键/心跳语义；PC-service HIL 可复现。
- [ ] `scripts/launch_quest.sh`（代码已实现；USB/无线 HIL 待现场）
- [x] `start.sh --real` 在拉起节点后最多等 15 s 的 `/vr_safe/teleop/*_target`；超时停止并退出（`scripts/wait_teleop.py`，已用假 publisher 验证成功/超时路径）。 能通过 USB/无线 adb 安装并拉起 Quest APK；失败时 `start.sh --real` 拒绝启动。
- [ ] SDK 不可用时给出明确错误，不静默启动半套遥操作。
