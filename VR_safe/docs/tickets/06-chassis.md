# 06: 底盘执行端

**What to build:** 独立的 AMR 遥控子集：状态/里程计、手柄遥控、暂停/零速、系统急停；`REVOKED` 时抑制操作员速度并每 100 ms 发零速，连续 5 次失败升级 AMR `emergency_stop`；本地 200 ms 无新 Twist 自动零速；恢复要求摇杆回中并重新使能遥控。

**Blocked by:** 02, 04

**Status:** implemented（代码/测试/dry-run）；AMR HIL 待现场

- [ ] AMR 连接、状态、电池、odom 可用；连接失败时按约定中止启动。
- [ ] `REVOKED` 时新速度被阻断，零速 keepalive 持续发送；fake SRP 下可观测。
- [ ] 连续 5 次零速失败升级 `emergency_stop`，并记录原因。
- [ ] 200 ms 没有新 Twist 自动零速。
- [ ] 恢复要求摇杆回中 + 重新使能遥控，不因摇杆残留而窜出。
- [ ] 不带 action/充电/定位代码；`move_pause` 不用于遥控。
