# 09: UNKNOWN 策略放宽 + 人员骨架

**What to build:** 只有当前 `OCCUPIED` 或“人进过区后跟丢”触发 HOLD；普通 `UNKNOWN` 不停止；观察超时/源消失/心跳丢失仍 HOLD。相机窗口画出所有检测到的人的简单骨架，并按状态着色。

**Blocked by:** 08

**Status:** implemented（状态机 16 测试 + overlay 5 测试 + 全量；真机 HIL 待现场）

- [x] 普通 `UNKNOWN`（上次不是 OCCUPIED）→ `GRANTED`；上次 `NEAR` → `SLOW`。
- [x] 上次 `OCCUPIED`、当前 `UNKNOWN` → `REVOKED` 锁存；等 `CLEAR/NEAR` 后确认。
- [x] 观察超时/源消失/心跳丢失仍 `REVOKED`。
- [x] 相机窗口画所有检出人员的 33 点骨架：区外绿、`NEAR` 黄、`OCCUPIED` 红。
- [x] 普通 `UNKNOWN` 不红框，灰字“看不清，不停止”。
- [x] `OCCUPIED` 连续 2 帧才 HOLD；单帧跳点不留下跟丢记忆。
- [x] 观察超时救急放宽到 0.8 s（待感知 ≥10 Hz 后收紧）。
- [ ] 真实相机/真人的骨架与跟丢场景 HIL。
