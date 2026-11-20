# 普通 UNKNOWN 不再单独停止

现场对齐（2026-11-20）：只有当前 `OCCUPIED`（人/手/头在保护区内）才停止。普通 `UNKNOWN`（画面里暂时没检出人、关键点不足）不再锁存，避免操作员被频繁的 `看不清` 卡住。为防止“人已经进区、随后模型跟丢就放行”，保留一条记忆规则：如果该观察源上一次稳定观察是 `OCCUPIED`，当前 `UNKNOWN` 仍 HOLD，直到重新看到 `CLEAR/NEAR` 并由人工确认。观察超时、观察源消失、心跳丢失、系统急停仍 HOLD——这些是感知/链路故障，不是“画面暂时看不清”。

## Considered Options

- 严格 fail-closed（原来）：任何 UNKNOWN 都 HOLD。安全但现场几乎无法连续遥操作。
- 完全放行 UNKNOWN（包括跟丢）：最顺手，但人进区后跟丢会直接放行，失效方向危险。
- 本决定：普通 UNKNOWN 放行；OCCUPIED 后的 UNKNOWN 保持 HOLD；感知/链路故障仍 HOLD。

## Consequences

- 普通 `UNKNOWN` 时机器人可以运动；操作员必须自己确认现场，系统不再代为兜底。
- 相机窗口用灰字提示“看不清，不停止”，红框只表示 `permit=REVOKED`。
- `confirm_resume` 仍要求当前 `CLEAR/NEAR`；跟丢状态不能确认。
