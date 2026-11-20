#!/usr/bin/env python3
"""区域标定入口：zone_tune 的 refit/save 包装。

和 zone_tune.py 的区别只有两点：路径来自 VR_safe/config/perception.yaml，
以及不依赖 camera_zone 的 out/ 目录。其余参数原样透传。

    ros2 run vr_safe_perception calibrate_zone --refit --save
    ros2 run vr_safe_perception calibrate_zone --size 1.2 1.2 --offset 0 0.2 --save
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

    load_and_env(known.config)

    # zone_tune 在 import 时读取 ZONE_CFG / ZONE_FILE，所以必须晚于上面。
    from .. import zone_tune

    sys.argv = [sys.argv[0]] + rest
    return int(zone_tune.main() or 0)


if __name__ == "__main__":
    sys.exit(main())
