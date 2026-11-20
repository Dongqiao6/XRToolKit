#!/usr/bin/env python3
"""外参标定入口：calib_extrinsic 的包装。

    ros2 run vr_safe_perception calibrate_extrinsic --help
    ros2 run vr_safe_perception calibrate_extrinsic --selftest
    ros2 run vr_safe_perception calibrate_extrinsic --capture out/cap --calibrate

标定结果写到 perception.yaml 的 calibration.extrinsic_file
（默认 VR_safe/config/calibration/extrinsic_calib.json）。
"""

import argparse
import sys

from ..config import load_config
from ..paths import default_perception_config
from ._common import load_and_env


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--config", default=default_perception_config(),
                     help="perception.yaml 路径")
    known, rest = pre.parse_known_args(argv)
    try:
        load_config(known.config)
    except Exception as exc:
        print(f"✗ perception 配置不合法：{exc}", file=sys.stderr)
        return 2

    cfg, paths = load_and_env(known.config)
    # calib_extrinsic 在 import 时读取这两个变量。
    import os
    os.environ["EXTRINSIC_CALIB_FILE"] = paths["extrinsic_file"]
    os.environ["ZONE_CFG"] = paths["zone_config"]

    from .. import calib_extrinsic

    sys.argv = [sys.argv[0]] + rest
    return int(calib_extrinsic.main() or 0)


if __name__ == "__main__":
    sys.exit(main())
