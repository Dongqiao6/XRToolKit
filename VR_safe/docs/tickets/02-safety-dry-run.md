# 02: 安全闭环 dry-run

**What to build:** 不接真相机、真机械臂、真底盘，也能完整演示“观察 → 许可 → 执行”的安全闭环：supervisor 拥有唯一许可与锁存，fake 观察源喂 CLEAR/NEAR/OCCUPIED/UNKNOWN/超时，fake 双臂/底盘 sink 显示 RUN/SLOW/HOLD，人工确认被拒绝或接受，审计事件落盘。

**Blocked by:** 01

**Status:** implemented & verified（dry-run PASS）

- [ ] `OCCUPIED`、`UNKNOWN`、观察超时、心跳丢失都使许可 `REVOKED`，fake sink 进入 HOLD。
- [ ] `NEAR` 使许可 `SLOW`，fake sink 显示 0.3×；回到 `CLEAR` 自动 `GRANTED`。
- [ ] `REVOKED` 锁存；区域非空/未知时确认被拒；`CLEAR` 或 `NEAR` 时确认成功，`NEAR` 只恢复到 `SLOW`。
- [ ] `LJ` 系统急停锁存；`Y` 只清按钮锁存，不恢复许可。
- [ ] 审计 JSONL 记录许可变化、观察切换、确认、心跳丢失和配置哈希。
- [ ] 一条 launch 能起 supervisor + fake 源 + fake sink，无需任何硬件。
