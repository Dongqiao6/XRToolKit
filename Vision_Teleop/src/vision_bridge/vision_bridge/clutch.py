"""Gesture-driven clutch and session state machine.

Two independent layers:

* session layer - the system arms only after both hands stay clenched for a
  configured time, and ESC always drops it back to standby;
* per-arm layer  - each arm follows only while its own hand is pinching.

A hand that disappears or whose gesture cannot be read releases its arm, so a
lost track can never keep a robot moving.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum

from .config import SafetyConfig


class Mode(str, Enum):
    STANDBY = "standby"
    READY = "ready"


class Gesture(str, Enum):
    NONE = "none"
    OPEN = "open"
    PINCH = "pinch"
    FIST = "fist"


@dataclass
class HandObservation:
    gesture: Gesture = Gesture.NONE
    pinch_ratio: float = float("nan")
    pinch_m: float = float("nan")
    curl: float = float("nan")
    present: bool = False

    @property
    def usable(self) -> bool:
        return self.present and self.gesture not in (Gesture.NONE,)


@dataclass
class ClutchOutput:
    mode: Mode
    left_engaged: bool
    right_engaged: bool
    left_gesture: Gesture
    right_gesture: Gesture
    prompt: str
    frozen: bool = False


class Clutch:
    def __init__(self, safety: SafetyConfig):
        self.safety = safety
        self.mode = Mode.STANDBY
        self.left_engaged = False
        self.right_engaged = False
        self._fist_accum = 0.0
        self._frozen_reason = ""
        self._emergency = False

    # -- session layer ---------------------------------------------------
    def toggle_emergency(self) -> None:
        """ESC: drop to standby and release both arms."""
        self.mode = Mode.STANDBY
        self.left_engaged = False
        self.right_engaged = False
        self._fist_accum = 0.0
        self._frozen_reason = "emergency stop"
        self._emergency = True

    def _update_session(self, dt: float, left: HandObservation, right: HandObservation) -> None:
        both_fist = left.gesture is Gesture.FIST and right.gesture is Gesture.FIST
        if self.mode is Mode.STANDBY:
            if both_fist:
                self._fist_accum += dt
                if self._fist_accum >= self.safety.fist_hold_s:
                    self.mode = Mode.READY
                    self._fist_accum = 0.0
                    self._frozen_reason = ""
                    self._emergency = False
            else:
                self._fist_accum = 0.0
        else:
            self._fist_accum = 0.0

    # -- per-arm layer ---------------------------------------------------
    @staticmethod
    def _arm_engaged(previous: bool, hand: HandObservation, safety: SafetyConfig) -> bool:
        if not hand.usable:
            return False
        if hand.gesture is Gesture.PINCH:
            # Re-entry needs a clearly closed pinch, exit happens on release,
            # but the gesture classifier already reports a stable label.
            return True if not previous else True
        return False

    def update(self, dt: float, left: HandObservation, right: HandObservation) -> ClutchOutput:
        self._update_session(dt, left, right)

        if self.mode is not Mode.READY:
            self.left_engaged = False
            self.right_engaged = False
            prompt = "STANDBY - clench both fists to arm"
            if self._frozen_reason:
                prompt = f"{prompt} ({self._frozen_reason})"
            return ClutchOutput(
                mode=self.mode,
                left_engaged=False,
                right_engaged=False,
                left_gesture=left.gesture,
                right_gesture=right.gesture,
                prompt=prompt,
                frozen=self._emergency,
            )

        self.left_engaged = self._arm_engaged(self.left_engaged, left, self.safety)
        self.right_engaged = self._arm_engaged(self.right_engaged, right, self.safety)

        if self.left_engaged and self.right_engaged:
            prompt = "DUAL ARM TRACKING"
        elif self.left_engaged:
            prompt = "LEFT ARM TRACKING"
        elif self.right_engaged:
            prompt = "RIGHT ARM TRACKING"
        else:
            prompt = "READY - pinch to follow"
        return ClutchOutput(
            mode=self.mode,
            left_engaged=self.left_engaged,
            right_engaged=self.right_engaged,
            left_gesture=left.gesture,
            right_gesture=right.gesture,
            prompt=prompt,
            frozen=self._emergency,
        )


def gestures_from_ratio(ratio: float, safety: SafetyConfig, previous: Gesture) -> Gesture:
    """Classify a normalised pinch distance with hysteresis."""
    if ratio != ratio:  # NaN: no reading
        return Gesture.NONE
    pinching = previous is Gesture.PINCH
    if pinching:
        return Gesture.PINCH if ratio <= safety.pinch_open else Gesture.OPEN
    return Gesture.PINCH if ratio <= safety.pinch_close else Gesture.OPEN


def is_fist(finger_curl: float, threshold: float = 1.15) -> bool:
    return finger_curl < threshold


FIST_CURL_ENTER = 1.15
FIST_CURL_EXIT = 1.35


class GestureFilter:
    """Pinch + fist classification with hysteresis, per hand.

    ``curl`` is the mean fingertip-to-wrist distance divided by hand size, so a
    closed fist sits well below an open hand and a pinch keeps the other three
    fingers extended.
    """

    def __init__(
        self,
        pinch_close: float,
        pinch_open: float,
        fist_curl_enter: float = FIST_CURL_ENTER,
        pinch_close_m: float = 0.02,
        pinch_open_m: float = 0.035,
        fist_curl_exit: float = FIST_CURL_EXIT,
        debounce_s: float = 0.0,
        pinch_min_curl: float = 1.2,
    ):
        self.pinch_close = pinch_close
        self.pinch_open = pinch_open
        self.pinch_close_m = float(pinch_close_m)
        self.pinch_open_m = float(pinch_open_m)
        self.fist_curl_enter = fist_curl_enter
        self.fist_curl_exit = fist_curl_exit
        self.debounce_s = max(0.0, float(debounce_s))
        self.pinch_min_curl = float(pinch_min_curl)
        self.previous = Gesture.NONE
        self._candidate = Gesture.NONE
        self._candidate_since = 0.0

    @staticmethod
    def _ratio_ok(ratio: float, limit: float) -> bool:
        return ratio != ratio or ratio <= limit  # NaN ratio is not a veto

    def update(
        self,
        present: bool,
        pinch_ratio: float,
        curl: float,
        now: float | None = None,
        pinch_m: float = float("nan"),
    ) -> Gesture:
        now = time.monotonic() if now is None else float(now)
        if not present:
            self.previous = Gesture.NONE
            self._candidate = Gesture.NONE
            self._candidate_since = now
            return Gesture.NONE
        if self.previous is Gesture.FIST:
            fist = curl <= self.fist_curl_exit
        else:
            fist = curl < self.fist_curl_enter
        if fist or curl < self.pinch_min_curl:
            # Too curled to be a pinch: a fist has the thumb and index tips
            # close together too, so the clutch must never see it as a pinch.
            gesture = Gesture.FIST
        elif pinch_m == pinch_m:  # absolute 3D distance is available
            # Require the metric distance AND the normalised ratio to agree: a
            # single bad landmark pair can make one of them look like a pinch,
            # but not both.
            if self.previous is Gesture.PINCH:
                close = pinch_m <= self.pinch_open_m and self._ratio_ok(pinch_ratio, 0.75)
            else:
                close = pinch_m <= self.pinch_close_m and self._ratio_ok(pinch_ratio, 0.65)
            gesture = Gesture.PINCH if close else Gesture.OPEN
        else:
            threshold = self.pinch_open if self.previous is Gesture.PINCH else self.pinch_close
            gesture = Gesture.PINCH if pinch_ratio <= threshold else Gesture.OPEN

        # A single noisy reading must not flip the clutch: require the new
        # label to persist for debounce_s seconds.
        if gesture is self.previous:
            self._candidate = gesture
            self._candidate_since = now
            return self.previous
        if gesture is not self._candidate:
            self._candidate = gesture
            self._candidate_since = now
        if self.debounce_s <= 0.0 or (now - self._candidate_since) >= self.debounce_s:
            self.previous = gesture
            self._candidate_since = now
        return self.previous
