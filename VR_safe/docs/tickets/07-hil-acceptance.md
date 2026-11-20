# 07: HIL 集成与验收

**What to build:** 把真相机、真双臂、真 AMR 放进同一个 launch，跑安全矩阵和延迟测试，完善 start/stop、预检、审计和最终文档；证明“人进入保护区 → 双臂和底盘一起停；恢复需人工确认且不跳变”。

**Blocked by:** 03, 05, 06

**Status:** scripts/docs ready；HIL 待现场硬件

- [ ] 一个 launch 起 teleop + perception + supervisor + 双臂 + 底盘；`start_amr:=false` 可只跑双臂。
- [ ] 观察时间戳到臂/底盘零速 p95 ≤ 200 ms（真机低速、物理急停在旁）。
- [ ] 测试矩阵全过：OCCUPIED、观察超时、心跳丢失、LJ、确认拒绝/接受、恢复不跳变、误报录像回归、supervisor 死亡、单臂故障。
- [ ] start/stop 脚本、preflight（相机/机器人 IP/AMR IP/PC-service/标定）和 JSONL 审计可用。
- [ ] README 写清启动、标定、按键、盲区、已知限制和 HIL 结果。
- [ ] 现场标定外参（当前 `ok:false` 不能进生产）。
