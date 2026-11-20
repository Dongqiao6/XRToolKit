"""OpenCV overlay: skeleton, wrist markers, tool cross, state banner."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

LEFT_COLOUR = (255, 128, 0)  # BGR, blue-ish
RIGHT_COLOUR = (0, 128, 255)  # BGR, orange-ish
SKELETON_COLOUR = (170, 170, 170)
TOOL_COLOUR = (0, 255, 0)

POSE_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 7), (0, 4), (4, 5), (5, 6), (6, 8),
    (9, 10), (11, 12), (11, 13), (13, 15), (15, 17), (15, 19), (15, 21),
    (17, 19), (12, 14), (14, 16), (16, 18), (16, 20), (16, 22), (18, 20),
    (11, 23), (12, 24), (23, 24), (23, 25), (24, 26), (25, 27), (26, 28),
    (27, 29), (28, 30), (29, 31), (30, 32), (27, 31), (28, 32),
)

LEFT_WRIST = 15
RIGHT_WRIST = 16
LEFT_ELBOW = 13
RIGHT_ELBOW = 14
LEFT_INDEX = 19
RIGHT_INDEX = 20
LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_HIP = 23
RIGHT_HIP = 24


@dataclass
class Hud:
    prompt: str = "STANDBY"
    mode: str = "standby"
    left_engaged: bool = False
    right_engaged: bool = False
    left_gesture: str = "none"
    right_gesture: str = "none"
    left_ratio: float = float("nan")
    right_ratio: float = float("nan")
    left_depth: float = 0.0
    right_depth: float = 0.0
    left_pinch_m: float = float("nan")
    right_pinch_m: float = float("nan")
    left_curl: float = float("nan")
    right_curl: float = float("nan")
    fps: float = 0.0
    latency_ms: float = 0.0
    frozen: bool = False


def _banner_colour(hud: Hud) -> tuple[int, int, int]:
    if hud.frozen:
        return (0, 0, 220)  # red
    if hud.left_engaged or hud.right_engaged:
        return (0, 150, 0)  # green
    if hud.mode == "ready":
        return (0, 165, 255)  # orange
    return (110, 110, 110)  # grey: standby, waiting for the arming gesture


def draw_overlay(
    bgr: np.ndarray,
    pose_landmarks: np.ndarray | None,
    left_hand,
    right_hand,
    left_tool_px: np.ndarray | None,
    right_tool_px: np.ndarray | None,
    hud: Hud,
) -> np.ndarray:
    canvas = bgr.copy()
    height, width = canvas.shape[:2]

    if pose_landmarks is not None:
        points = [(int(x * width), int(y * height)) for x, y in pose_landmarks]
        for a, b in POSE_CONNECTIONS:
            cv2.line(canvas, points[a], points[b], SKELETON_COLOUR, 1, cv2.LINE_AA)
        for index, point in enumerate(points):
            colour = SKELETON_COLOUR
            if index == LEFT_WRIST:
                colour = LEFT_COLOUR
            elif index == RIGHT_WRIST:
                colour = RIGHT_COLOUR
            cv2.circle(canvas, point, 2, colour, -1, cv2.LINE_AA)
        _draw_wrist(
            canvas, points[LEFT_WRIST], "L", LEFT_COLOUR, hud.left_engaged, hud.left_gesture,
            _near_edge(points[LEFT_WRIST], width, height),
        )
        _draw_wrist(
            canvas, points[RIGHT_WRIST], "R", RIGHT_COLOUR, hud.right_engaged, hud.right_gesture,
            _near_edge(points[RIGHT_WRIST], width, height),
        )

    for hand, colour in ((left_hand, LEFT_COLOUR), (right_hand, RIGHT_COLOUR)):
        if hand is not None and hand.crop_box is not None:
            x, y, size, _ = hand.crop_box
            cv2.rectangle(canvas, (x, y), (x + size, y + size), colour, 1)
        if hand is not None and hand.landmarks_px is not None and hand.present:
            for point in hand.landmarks_px:
                cv2.circle(canvas, (int(point[0]), int(point[1])), 1, colour, -1, cv2.LINE_AA)

    for point, colour in ((left_tool_px, LEFT_COLOUR), (right_tool_px, RIGHT_COLOUR)):
        if point is None:
            continue
        x, y = int(point[0]), int(point[1])
        cv2.drawMarker(canvas, (x, y), TOOL_COLOUR, cv2.MARKER_CROSS, 16, 2)

    _draw_banner(canvas, hud)
    _draw_hands(canvas, hud)
    _draw_stats(canvas, hud)
    return canvas


def _near_edge(point, width: int, height: int, margin: float = 0.08) -> bool:
    """True when a landmark is close enough to the border to be clipped soon."""
    x, y = point
    return (
        x < width * margin
        or x > width * (1.0 - margin)
        or y < height * margin
        or y > height * (1.0 - margin)
    )


def _draw_wrist(canvas, point, label: str, colour, engaged: bool, gesture: str, near_edge: bool = False) -> None:
    if near_edge:
        colour = (0, 0, 255)  # red: hand about to leave the frame
    radius = 14 if engaged else 9
    thickness = 3 if engaged else 2
    cv2.circle(canvas, point, radius, colour, thickness, cv2.LINE_AA)
    if engaged:
        cv2.circle(canvas, point, radius + 6, colour, 1, cv2.LINE_AA)
    cv2.putText(
        canvas,
        f"{label}:{gesture}{' !' if near_edge else ''}",
        (point[0] + 16, point[1] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        colour,
        2,
        cv2.LINE_AA,
    )


def _draw_banner(canvas, hud: Hud) -> None:
    colour = _banner_colour(hud)
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 46), colour, -1)
    cv2.putText(
        canvas,
        hud.prompt,
        (14, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.85,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        "ESC = freeze / re-arm   q = quit",
        (canvas.shape[1] - 330, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


def _draw_hands(canvas, hud: Hud) -> None:
    lines = [
        f"L r={hud.left_ratio:.2f} d={hud.left_pinch_m*100:.1f}cm c={hud.left_curl:.2f} "
        f"{hud.left_gesture} {hud.left_depth:.2f}m",
        f"R r={hud.right_ratio:.2f} d={hud.right_pinch_m*100:.1f}cm c={hud.right_curl:.2f} "
        f"{hud.right_gesture} {hud.right_depth:.2f}m",
    ]
    y = canvas.shape[0] - 48
    for line in lines:
        cv2.putText(
            canvas, line, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA
        )
        y += 24


def _draw_stats(canvas, hud: Hud) -> None:
    text = f"{hud.fps:4.1f} fps   {hud.latency_ms:5.1f} ms"
    cv2.putText(
        canvas,
        text,
        (canvas.shape[1] - 210, canvas.shape[0] - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
