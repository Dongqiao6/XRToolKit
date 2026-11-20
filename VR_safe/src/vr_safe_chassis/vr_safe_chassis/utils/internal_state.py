"""第一版没有 action / 移动任务，保留 demo 的最小内部状态容器。

目前只有 ``movement_no``，用于将来接入 action 时生成任务号；底盘遥控路径不
读取它，也不依赖 ``move_pause``。
"""

from dataclasses import dataclass


@dataclass
class InternalState:
    movement_no: int = 0

    def generate_movement_no(self) -> int:
        self.movement_no += 1
        return self.movement_no
