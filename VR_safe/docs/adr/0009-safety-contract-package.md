# 安全契约用独立的 vr_safe_interfaces

VR_safe 不复制 `workcell_interfaces`，也不把 JSON 塞进 `std_msgs/String`。安全契约定义在 `vr_safe_interfaces`：`SafetyObservation`、`MotionPermit`、`TeleopHeartbeat`、`ArmState`、`ConfirmResume`；话题统一 `/vr_safe/...`。这样旧工程的 `/workcell/*` 命名和未使用字段不会进入新系统，接入新传感器/机器人时只扩展这个包。

## Considered Options

- 复制 `workcell_interfaces`：能复用，但带着 `ArmExecutionState` 等当前不用的字段，且命名把新系统绑回旧工程。
- `std_msgs/String` + JSON：改动快，但失去类型、QoS 和 rosidl 的校验。
