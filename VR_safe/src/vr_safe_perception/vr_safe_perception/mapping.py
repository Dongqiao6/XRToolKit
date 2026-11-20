"""五档判定结果 → SafetyObservation 四值的映射与构造。

camera_zone 的 zone_monitor 输出五个中文档位；vr_safe_interfaces 只认
CLEAR/NEAR/OCCUPIED/UNKNOWN 四个安全状态。这个模块是两边唯一的翻译层，
也是测试里重点覆盖的 seam：不依赖相机、MediaPipe 或 ROS。
"""

import json
import math
from dataclasses import dataclass
from typing import Any, Dict, Optional

# camera_zone 的五个档位（值保持中文，和 zone_monitor.ORDER 完全一致）。
IDLE = "空闲"
BESIDE = "旁边有人"
UNCLEAR = "看不清"
HAND_IN = "手伸入"
OCCUPIED = "占用"
FIVE_STATES = (IDLE, BESIDE, UNCLEAR, HAND_IN, OCCUPIED)

# SafetyObservation 的四值（字符串形式；ROS 节点再翻译成 msg 常量）。
CLEAR = "CLEAR"
NEAR = "NEAR"
OCCUPIED_SAFE = "OCCUPIED"
UNKNOWN = "UNKNOWN"

# 五档 → 四值。手伸入和占用都按「区域被占」处理，这是安全侧的选择：
# 手可能已经在机械臂运动路径上，不能因为脚还在区外就放行。
FIVE_TO_FOUR = {
    IDLE: CLEAR,
    BESIDE: NEAR,
    UNCLEAR: UNKNOWN,
    HAND_IN: OCCUPIED_SAFE,
    OCCUPIED: OCCUPIED_SAFE,
}

# 与 SafetyObservation.evidence 的 EV_* 对齐。
EV_NONE = 0
EV_TORSO = 1
EV_LIMB = 2
EV_HEAD = 3
EV_LOST = 4


def map_five_to_four(state: str) -> str:
    """五档 → 四值。未知档位直接报错，不猜成 CLEAR。"""
    try:
        return FIVE_TO_FOUR[state]
    except KeyError as exc:
        raise ValueError(f"未知的五档状态：{state!r}") from exc


def evidence_for(state: str, why: str = "", info: Optional[Dict[str, Any]] = None) -> int:
    """按最终四值给一个 evidence 码。

    evidence 只说明「这条判断主要看到了什么」，不参与安全裁决：

        0 无       CLEAR / NEAR
        1 躯干     占用，且躯干点进了区域
        2 肢体     手/肘/膝伸入，或占用但不是躯干触发
        3 头       头探入
        4 丢失     看不清（画面不可信、人被挡、关键点全丢）
    """
    four = map_five_to_four(state) if state in FIVE_TO_FOUR else state
    if four == UNKNOWN:
        return EV_LOST
    if four == CLEAR or four == NEAR:
        return EV_NONE
    if state == HAND_IN:
        return EV_HEAD if "头" in (why or "") else EV_LIMB
    info = info or {}
    if "躯干" in (why or "") or int(info.get("torso_pts") or 0) > 0:
        return EV_TORSO
    return EV_LIMB


def confidence_for(state: str, info: Optional[Dict[str, Any]] = None) -> float:
    """给 0~1 的置信度。

    安全判断不靠这个数做门限（supervisor 只看状态），所以这里只表达
    「这次结论有多确定」：看不清时给低值，其余给高值。
    """
    four = FIVE_TO_FOUR.get(state, state)
    if four == UNKNOWN:
        return 0.3
    info = info or {}
    if int(info.get("n_persons") or 0) == 0:
        return 1.0
    if int(info.get("good_lms") or 0) >= 20:
        return 1.0
    return 0.9


def jsonable(value: Any) -> Any:
    """把 numpy / set / 非有限浮点数变成 json.dumps 能接受的东西。

    detail_json 是审计和现场排查唯一的原始记录，不能因为一个 ndarray 或
    NaN 就让整条观察发布失败——那会把一次真实判断吞掉。
    """
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, set):
        return sorted(jsonable(v) for v in value)
    # numpy 标量/数组、dataclass 之类：不直接 import numpy，按常用接口处理。
    if hasattr(value, "tolist"):
        return jsonable(value.tolist())
    if hasattr(value, "item"):
        return jsonable(value.item())
    return str(value)


@dataclass(frozen=True)
class Observation:
    """一条待发布的观察。字段与 SafetyObservation 一一对应。"""
    source_id: str
    zone_id: str
    state: str
    confidence: float
    evidence: int
    detail_json: str
    five_state: str
    reason: str


def build_observation(source_id: str, zone_id: str, five_state: str,
                      reason: str = "", info: Optional[Dict[str, Any]] = None) -> Observation:
    """由五档判定结果构造观察。detail 里保留全部中间量。"""
    four = map_five_to_four(five_state)
    evidence = evidence_for(five_state, reason, info)
    confidence = confidence_for(five_state, info)
    detail: Dict[str, Any] = dict(info or {})
    detail.update({
        "five_state": five_state,
        "safe_state": four,
        "reason": reason,
        "evidence": evidence,
    })
    return Observation(
        source_id=source_id,
        zone_id=zone_id,
        state=four,
        confidence=float(confidence),
        evidence=int(evidence),
        detail_json=json.dumps(jsonable(detail), ensure_ascii=False,
                               allow_nan=False, separators=(",", ":")),
        five_state=five_state,
        reason=reason,
    )
