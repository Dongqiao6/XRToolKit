# 05: 双臂执行端

**What to build:** 两个独立的 C++ 臂控制进程（各连一台 JAKA），从 `vision_rc` 基线复制并加许可门：`REVOKED` 时阻断新目标、松开离合、持续零增量、保持伺服；本地 target 看门狗兜住链路失联；许可恢复后必须重新按 Grip 并重新锚定。

**Blocked by:** 02, 04

**Status:** implemented（构建/ctest/dry-run）；JAKA HIL 待现场

- [ ] 两个臂进程可同时构建、启动，各自只连一台控制器。
- [ ] `REVOKED` 时不再发新 `servo_p` 目标，持续零增量，臂保持位置。
- [ ] 200 ms 没有新 target 且处于 gripped 时自动松开离合/HOLD。
- [ ] 许可恢复后不自动跟随；重新按 Grip 后从当前位置重新锚定，没有跳变。
- [ ] fake SDK dry-run 可验证许可门、重离合、看门狗。
- [ ] 真机 HIL 保留 Grip 迟滞、5 mm/周期限速、y 向限位、碰撞检测。
