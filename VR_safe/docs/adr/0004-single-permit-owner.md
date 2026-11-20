# 由独立的 safety_supervisor 持有唯一许可

双臂由两个 C++ 控制进程驱动（JAKA SDK 每进程一条连接），许可必须只有一份。我们决定用独立的 `safety_supervisor` 进程拥有许可、锁存、心跳看门狗和人工复位服务；两个 `arm_control` 订阅 `/vr_safe/permit`，各自保留本地看门狗，超时即 HOLD。这样两臂同时停、同时恢复，新增观察源或执行端只改 supervisor 的输入/输出，不改臂节点。

## Considered Options

- 每个 `arm_control` 内嵌 `SafetyGate`：少一个进程，但两份许可状态会漂移，复位只能放其中一个节点，职责不清。
- 感知节点直接发许可：少一跳，但把安全策略放进感知；感知故障即失去裁决，新增传感器要重复策略。

## Consequences

- supervisor 本身成为单点：它挂掉时，`arm_control` 必须靠本地看门狗 HOLD；supervisor 也要监控观察源心跳。
- 多一个 Python 进程和一条 `permit` 话题；同机 DDS，延迟可忽略。
