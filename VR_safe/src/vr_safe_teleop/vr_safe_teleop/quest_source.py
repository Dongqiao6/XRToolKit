"""Quest 控制器输入源：真实 XRoboToolkit SDK 与 dry-run 合成输入。

``SdkQuestSource`` 只做 SDK 读取，不做安全判断；``DryRunQuestSource`` 从
JSON 帧重放合成输入。两者输出同一种原始 controller dict，交给
:class:`vr_safe_teleop.teleop_core.TeleopCore` 解释，因此按钮/心跳/急停语义
在没有 Quest 的环境里也能完整验证。
"""

import json
import os
import time
from typing import Any, Callable, Dict, Iterable, List, Optional

from .xrt_adapter import axis_pair, finite_float


class SdkQuestSource:
    """读取 XRoboToolkit SDK。SDK 缺失或初始化失败时抛 ``RuntimeError``。"""

    def __init__(self):
        try:
            import xrobotoolkit_sdk as xrt
        except ImportError as exc:
            raise RuntimeError(
                "xrobotoolkit_sdk 未安装：请先安装 XRoboToolkit PC-Service 的 "
                "Python 绑定；无 Quest 时请用 dry_run:=true。"
            ) from exc

        try:
            xrt.init()
        except Exception as exc:  # SDK 的异常类型不固定，统一包装成启动错误
            raise RuntimeError(
                "XRoboToolkit SDK 初始化失败：请启动 PC-service 并确认 Quest "
                "应用已连接；无 Quest 时请用 dry_run:=true。"
            ) from exc
        self._xrt = xrt
        self._closed = False

    def _safe(self, getter_name: str, default: Any) -> Any:
        try:
            return getattr(self._xrt, getter_name)()
        except Exception:
            # 单帧读取失败按“这一帧没有数据”处理；进程继续发心跳。
            return default

    def _read_side(self, prefix: str) -> Dict[str, Any]:
        state: Dict[str, Any] = {
            "pose": self._safe("get_%s_controller_pose" % prefix, ()),
            "grip": finite_float(self._safe("get_%s_grip" % prefix, 0.0), 0.0),
            "trigger": finite_float(
                self._safe("get_%s_trigger" % prefix, None), None
            ),
            "axis": axis_pair(
                self._safe("get_%s_axis" % prefix, (0.0, 0.0))
            ),
        }
        # X/A 分别只属于左/右手柄；其余按键各自读取。
        if prefix == "left":
            state["X"] = bool(self._safe("get_X_button", False))
            state["Y"] = bool(self._safe("get_Y_button", False))
            state["A"] = False
            state["B"] = False
        else:
            state["X"] = False
            state["Y"] = False
            state["A"] = bool(self._safe("get_A_button", False))
            state["B"] = bool(self._safe("get_B_button", False))
        state["menu"] = bool(
            self._safe("get_%s_menu_button" % prefix, False)
        )
        state["axis_click"] = bool(
            self._safe("get_%s_axis_click" % prefix, False)
        )
        return state

    def read(self) -> Dict[str, Dict[str, Any]]:
        return {
            "left": self._read_side("left"),
            "right": self._read_side("right"),
        }

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._xrt.close()
        except Exception:
            # 退出路径不因为 SDK 清理失败而掩盖真正的启动/运行错误。
            pass


class DryRunQuestSource:
    """重放合成控制器帧，不 import SDK。

    帧格式::

        {"t": 0.3, "left": {"pose": [...], "axis_click": true}, "right": {...}}

    ``t`` 是自源创建起的秒数；未写的键沿用上一帧。没有帧时输出中性输入
    （两侧均未跟踪），桥仍然发布心跳和中性 Joy，便于无 Quest 冒烟测试。
    """

    # 循环重放时给最后一帧留的展示时间，避免最后一帧一闪而过。
    LOOP_HOLD_S = 1.0

    def __init__(
        self,
        frames: Optional[Iterable[Dict[str, Any]]] = None,
        loop: bool = True,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._frames = _normalize_frames(frames or [])
        self._loop = bool(loop)
        self._clock = clock
        self._started_at = float(clock())

    @classmethod
    def from_file(
        cls,
        path: str,
        loop: bool = True,
        clock: Callable[[], float] = time.monotonic,
    ) -> "DryRunQuestSource":
        if not path:
            return cls([], loop=loop, clock=clock)
        if not os.path.isfile(path):
            raise ValueError("dry_run_input 文件不存在：%r" % path)
        try:
            with open(path, "r", encoding="utf-8") as handle:
                raw = json.load(handle)
        except json.JSONDecodeError as exc:
            raise ValueError(
                "dry_run_input JSON 解析失败（%s）：%s" % (path, exc)
            ) from exc
        except OSError as exc:
            raise ValueError("dry_run_input 读取失败（%s）：%s" % (path, exc)) from exc

        if isinstance(raw, dict):
            raw_frames = raw.get("frames", [])
        else:
            raw_frames = raw
        if not isinstance(raw_frames, list):
            raise ValueError("dry_run_input 的 frames 必须是数组")
        return cls(raw_frames, loop=loop, clock=clock)

    def _elapsed(self) -> float:
        return max(0.0, float(self._clock()) - self._started_at)

    def _cycle_elapsed(self, elapsed: float) -> float:
        if not self._loop or not self._frames:
            return elapsed
        last_t = self._frames[-1]["t"]
        cycle = last_t + self.LOOP_HOLD_S
        if cycle <= 0.0:
            return 0.0
        return elapsed % cycle

    def read(self) -> Dict[str, Dict[str, Any]]:
        elapsed = self._cycle_elapsed(self._elapsed())
        left: Dict[str, Any] = {}
        right: Dict[str, Any] = {}
        for frame in self._frames:
            if frame["t"] > elapsed:
                break
            left.update(frame["left"])
            right.update(frame["right"])
        return {"left": left, "right": right}

    def close(self):
        # 合成输入没有需要释放的资源。
        pass


def _normalize_frames(raw_frames: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    frames: List[Dict[str, Any]] = []
    previous_t = -1.0
    for index, raw in enumerate(raw_frames):
        if not isinstance(raw, dict):
            raise ValueError("dry_run_input 第 %d 帧必须是对象" % index)
        t = finite_float(raw.get("t", 0.0), None)
        if t is None or t < 0.0:
            raise ValueError("dry_run_input 第 %d 帧的 t 必须是非负有限数" % index)
        if t < previous_t:
            raise ValueError("dry_run_input 的帧必须按 t 非降序排列")
        previous_t = t
        left = raw.get("left", {})
        right = raw.get("right", {})
        if not isinstance(left, dict) or not isinstance(right, dict):
            raise ValueError("dry_run_input 第 %d 帧的 left/right 必须是对象" % index)
        frames.append({"t": float(t), "left": dict(left), "right": dict(right)})
    frames.sort(key=lambda frame: frame["t"])
    return frames


def create_source(
    dry_run: bool,
    dry_run_input: str = "",
    dry_run_loop: bool = True,
):
    """按配置创建输入源；dry_run 路径绝不 import SDK。"""
    if dry_run:
        return DryRunQuestSource.from_file(dry_run_input, loop=dry_run_loop)
    return SdkQuestSource()
