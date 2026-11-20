#!/usr/bin/env python3
"""回放录像并发布 /vr_safe/observation。

等价于 observation_node --replay，只是把回放专用的参数暴露成一个短入口。
默认按原始帧间隔回放；--replay-fast 用最大速度跑，--replay-loop 循环。

    ros2 run vr_safe_perception replay_clip out/clips/legs --allow-uncalibrated
    ros2 run vr_safe_perception replay_clip out/clips/legs --replay-fast --no-hands
"""

import argparse
import sys

from ..paths import default_perception_config


def main(argv=None):
    ap = argparse.ArgumentParser(description="回放录像并发布 SafetyObservation")
    ap.add_argument("clip", help="录像片段目录")
    ap.add_argument("--config", default=default_perception_config())
    ap.add_argument("--allow-uncalibrated", action="store_true")
    ap.add_argument("--replay-fast", action="store_true")
    ap.add_argument("--replay-loop", action="store_true")
    ap.add_argument("--no-hands", action="store_true")
    ap.add_argument("--zone-mode", choices=["polygon", "reach"], default=None)
    ap.add_argument("--zone-file", default=None)
    ap.add_argument("--max-frames", type=int, default=0)
    args = ap.parse_args(argv)

    # rclpy / MediaPipe 只在真正回放时 import，--help 不需要部署环境齐全。
    from ..observation_node import main as node_main

    node_argv = ["--config", args.config, "--replay", args.clip]
    if args.allow_uncalibrated:
        node_argv.append("--allow-uncalibrated")
    if args.replay_fast:
        node_argv.append("--replay-fast")
    if args.replay_loop:
        node_argv.append("--replay-loop")
    if args.no_hands:
        node_argv.append("--no-hands")
    if args.zone_mode:
        node_argv += ["--zone-mode", args.zone_mode]
    if args.zone_file:
        node_argv += ["--zone-file", args.zone_file]
    if args.max_frames:
        node_argv += ["--max-frames", str(args.max_frames)]
    return int(node_main(node_argv) or 0)


if __name__ == "__main__":
    sys.exit(main())
