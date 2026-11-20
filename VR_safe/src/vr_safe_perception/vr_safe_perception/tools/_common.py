"""标定/预览/录像工具共用的小函数：解析 --config、设置路径环境变量。

工具在 import zone_tune / calib_extrinsic 之前必须先设好这些环境变量，
因为这些老模块在 import 时就把默认路径算成了模块级常量。
"""

import os

from ..config import load_config, resolved_paths


def load_and_env(config_path):
    """读配置并把它解析出的路径写进环境变量，返回 (cfg, paths)。"""
    cfg = load_config(config_path)
    paths = resolved_paths(cfg, config_path)
    cal_dir = os.path.dirname(paths["zone_config"])
    os.makedirs(cal_dir, exist_ok=True)
    os.environ["ZONE_CFG"] = paths["zone_config"]
    os.environ["ZONE_FILE"] = paths["zone_file"]
    os.environ["EXTRINSIC_CALIB_FILE"] = paths["extrinsic_file"]
    os.environ["POSE_MODEL"] = paths["pose_model"]
    os.environ["HAND_MODEL"] = paths["hand_model"]
    os.environ["ZONE_H"] = str(cfg["zone"]["height_m"])
    os.environ["NEAR_M"] = str(cfg["zone"]["near_m"])
    os.environ["BODY_PARTS"] = str(cfg["pose"]["body_parts"])
    return cfg, paths
