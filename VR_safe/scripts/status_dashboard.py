#!/usr/bin/env python3
"""常驻状态行：每秒打印一行，供 start.sh 前台使用。"""

import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException

from status import Status


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interval", type=float, default=1.0)
    args = ap.parse_args()
    rclpy.init()
    node = Status()
    try:
        while rclpy.ok():
            end = time.monotonic() + max(0.1, args.interval)
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            print(node.render_line(), flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
