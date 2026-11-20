# 系统软件急停走 supervisor L2

Quest 左摇杆按下（`LJ`）被定义为系统软件急停：它向 supervisor 发一条 `ESTOP` 观察，supervisor 把许可置 `REVOKED` 并锁存；双臂 HOLD、底盘零速。它不调用 AMR 的 `emergency_stop`，也不等同于硬件急停。`Y` 只解除按钮锁存，恢复仍需 `confirm_resume` + 重新 Grip/遥控。

## Considered Options

- 只停底盘：操作员按一次急停，臂还在动，语义不完整。
- 直接调用 AMR `emergency_stop`：底盘硬停、恢复慢，而且臂不受影响。
- 把 `LJ` 当硬件急停：硬件急停必须独立于软件，不能用同一个按钮替代。

## Consequences

- supervisor 需要把 `ESTOP` 作为一种观察源；它比相机观察优先级更高，任何情况下都锁存。
- 需要区分“按钮锁存解除”和“许可恢复”：`Y` 只做前者，后者要 `confirm`。
- 底盘零速连续失败 5 次时，才升级为 AMR `emergency_stop`。
