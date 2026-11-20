#!/usr/bin/env python3
"""人工确认恢复许可。"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from vr_safe_interfaces.srv import ConfirmResume


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--by", default="operator", help="谁确认的")
    ap.add_argument("--note", default="", help="备注")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("vr_safe_confirm_resume")
    cli = node.create_client(ConfirmResume, "/vr_safe/confirm_resume")
    try:
        if not cli.wait_for_service(timeout_sec=args.timeout):
            print("✗ 找不到 /vr_safe/confirm_resume：supervisor 没在跑？", file=sys.stderr)
            return 1
        req = ConfirmResume.Request()
        req.confirmed_by = args.by
        req.note = args.note
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=args.timeout)
        if not fut.done():
            print("✗ 确认超时", file=sys.stderr)
            return 1
        res = fut.result()
        print(("✓ " if res.accepted else "✗ ") + res.message)
        return 0 if res.accepted else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
