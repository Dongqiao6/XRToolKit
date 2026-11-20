"""相机画面叠加层：许可/观察/区域/恢复提示。纯函数，便于离线测试。"""

from dataclasses import dataclass
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)
_FONT_CACHE = {}

BGR_RED = (0, 0, 255)
BGR_YELLOW = (0, 200, 255)
BGR_GREEN = (0, 200, 0)
BGR_GRAY = (160, 160, 160)
BGR_WHITE = (255, 255, 255)

# BlazePose 33 点的简单骨架；只用于画面提示。
BONES = [(11, 12), (11, 23), (12, 24), (23, 24),
         (11, 13), (13, 15), (12, 14), (14, 16),
         (23, 25), (25, 27), (24, 26), (26, 28),
         (27, 31), (28, 32), (0, 11), (0, 12)]


@dataclass
class OverlayState:
    permit_decision: str = "UNKNOWN"
    permit_latched: bool = False
    permit_reason: str = ""
    permit_source: str = ""
    obs_state: str = ""
    obs_reason: str = ""
    confirm_status: str = ""


def border_bgr(state: OverlayState):
    if state.permit_decision == "GRANTED":
        return BGR_GREEN
    if state.permit_decision == "SLOW":
        return BGR_YELLOW
    if state.permit_decision == "REVOKED":
        return BGR_RED
    return BGR_GRAY


def zone_bgr(state: OverlayState):
    if state.obs_state == "CLEAR":
        return BGR_GREEN
    if state.obs_state == "NEAR":
        return BGR_YELLOW
    if state.obs_state == "OCCUPIED":
        return BGR_RED
    return BGR_GRAY


def hint_for(state: OverlayState) -> str:
    if state.confirm_status:
        return state.confirm_status
    if state.obs_state == "UNKNOWN" and state.permit_decision != "REVOKED":
        return "看不清/测不出：按当前策略不停止，注意确认现场"
    if state.permit_decision == "REVOKED":
        if state.obs_state == "OCCUPIED":
            return "区域里还有人，不能恢复"
        if state.obs_state == "UNKNOWN":
            return "看不清/测不出，不能恢复"
        if state.permit_source == "system_estop":
            return "系统急停：先按 Y 解除按钮锁存，再按 RJ 长按 1.5s"
        if state.obs_state in ("CLEAR", "NEAR"):
            return "按 RJ 长按 1.5s 恢复许可"
        return "等待人工确认"
    if state.permit_decision == "SLOW":
        return "人在旁边，慢速运行"
    if state.permit_decision == "GRANTED":
        return "运行中：Grip 控制机械臂，左摇杆控制底盘"
    return "等待许可"


def _font(size):
    if size in _FONT_CACHE:
        return _FONT_CACHE[size]
    for path in _FONT_CANDIDATES:
        if os.path.isfile(path):
            try:
                f = ImageFont.truetype(path, size)
                _FONT_CACHE[size] = f
                return f
            except Exception:
                continue
    f = ImageFont.load_default()
    _FONT_CACHE[size] = f
    return f


def _fit(text, limit=46):
    text = str(text or "").replace("\n", " ")
    return text if len(text) <= limit else text[: limit - 1] + "…"


def smooth_persons(previous, current, alpha=0.35):
    """只平滑显示用骨架；不改安全判定。按 tracker id 优先匹配，缺失则按顺序。"""
    current = current or []
    previous = previous or []
    if not previous or not current:
        return current
    alpha = max(0.0, min(1.0, float(alpha)))
    by_id = {p.get("id"): p for p in previous
             if p.get("id") is not None}
    out = []
    for i, cur in enumerate(current):
        old = by_id.get(cur.get("id")) if cur.get("id") is not None else None
        if old is None and len(previous) == len(current):
            old = previous[i]
        cm = cur.get("landmarks") or []
        om = (old or {}).get("landmarks") or []
        if len(cm) != 33 or len(om) != 33:
            out.append(cur)
            continue
        marks = []
        for (x, y, v), (ox, oy, ov) in zip(cm, om):
            marks.append([alpha * float(x) + (1 - alpha) * float(ox),
                          alpha * float(y) + (1 - alpha) * float(oy),
                          float(v)])
        item = dict(cur)
        item["landmarks"] = marks
        out.append(item)
    return out


def _person_color(person):
    if person.get("state") == "OCCUPIED":
        return BGR_RED
    if person.get("state") == "NEAR":
        return BGR_YELLOW
    return BGR_GREEN


def _draw_skeletons(out, persons):
    h, w = out.shape[:2]
    for person in persons or []:
        marks = person.get("landmarks") or []
        if len(marks) < 33:
            continue
        color = _person_color(person)
        for a, b in BONES:
            if a >= len(marks) or b >= len(marks):
                continue
            ax, ay, av = marks[a]
            bx, by, bv = marks[b]
            if av < 0.5 or bv < 0.5:
                continue
            cv2.line(out, (int(ax * w), int(ay * h)),
                     (int(bx * w), int(by * h)), color, 2, cv2.LINE_AA)
        for x, y, v in marks:
            if v < 0.5:
                continue
            cv2.circle(out, (int(x * w), int(y * h)), 3, color, -1, cv2.LINE_AA)


def render(frame: np.ndarray, state: OverlayState, zone=None,
           persons=None) -> np.ndarray:
    """在 BGR 图上画边框、区域、骨架、状态条和恢复提示。"""
    out = frame.copy()
    h, w = out.shape[:2]

    if zone:
        color = zone_bgr(state)
        corners = zone.get("corners_px") or []
        if corners:
            pts = np.array(corners, np.int32).reshape(-1, 1, 2)
            if state.obs_state == "OCCUPIED":
                overlay = out.copy()
                cv2.fillPoly(overlay, [pts], color)
                out = cv2.addWeighted(overlay, 0.22, out, 0.78, 0)
            cv2.polylines(out, [pts], True, color, 2, cv2.LINE_AA)
        for sphere in zone.get("spheres") or []:
            try:
                x, y, r = sphere
                cv2.circle(out, (int(x), int(y)), max(2, int(r)), color, 2)
            except (TypeError, ValueError):
                continue

    _draw_skeletons(out, persons)

    top = 84
    bottom = 38
    img = Image.fromarray(cv2.cvtColor(out, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img)
    draw.rectangle([0, 0, w, top], fill=(15, 15, 15))
    draw.rectangle([0, h - bottom, w, h], fill=(15, 15, 15))

    title = f"许可 {state.permit_decision}"
    if state.permit_latched:
        title += "（已锁存）"
    if state.permit_reason:
        title += f"：{_fit(state.permit_reason, 36)}"
    draw.text((12, 6), title, font=_font(21), fill=border_bgr(state)[::-1])

    obs = f"观察 {state.obs_state or '—'}"
    if state.obs_reason:
        obs += f"：{_fit(state.obs_reason, 38)}"
    draw.text((12, 38), obs, font=_font(18), fill=zone_bgr(state)[::-1])

    hint = _fit(hint_for(state), 52)
    draw.text((12, h - bottom + 8), hint, font=_font(18), fill=BGR_WHITE[::-1])

    out = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    cv2.rectangle(out, (0, 0), (w - 1, h - 1), border_bgr(state), 10)
    return out
