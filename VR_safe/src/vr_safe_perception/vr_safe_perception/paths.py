"""VR_safe 目录约定：配置文件、标定产物、模型和仿真资源的默认位置。

这里只做路径解析，不读配置内容。优先级：

    VR_SAFE_ROOT 环境变量
    → 从本文件往上找带 config/ 和 docs/ 的目录（源码树里的 VR_safe/）
    → 当前工作目录往上找

安装之后配置不在 Python 包里，入口脚本必须显式传 --config 或设
VR_SAFE_PERCEPTION_CONFIG；这里的默认值只用于源码树里直接运行工具。
"""

import os
from pathlib import Path


def vr_safe_root():
    """返回 VR_safe 工程根目录；找不到返回 None。"""
    env = os.environ.get("VR_SAFE_ROOT", "").strip()
    if env:
        p = Path(env).expanduser().resolve()
        if p.is_dir():
            return p

    here = Path(__file__).resolve()
    for base in (here.parent, *here.parents):
        if (base / "config").is_dir() and (base / "docs").is_dir():
            return base

    cwd = Path.cwd().resolve()
    for base in (cwd, *cwd.parents):
        if (base / "config").is_dir() and (base / "docs").is_dir():
            return base
    return None


def _root_or_cwd():
    return vr_safe_root() or Path.cwd().resolve()


def default_perception_config():
    """默认的 perception.yaml。"""
    env = os.environ.get("VR_SAFE_PERCEPTION_CONFIG", "").strip()
    if env:
        return str(Path(env).expanduser())
    return str(_root_or_cwd() / "config" / "perception.yaml")


def default_calibration_dir():
    """标定产物目录：VR_safe/config/calibration/。"""
    env = os.environ.get("VR_SAFE_CALIBRATION_DIR", "").strip()
    if env:
        return str(Path(env).expanduser())
    return str(_root_or_cwd() / "config" / "calibration")


def default_models_dir():
    """媒体模型目录：VR_safe/models/。"""
    env = os.environ.get("VR_SAFE_MODELS_DIR", "").strip()
    if env:
        return str(Path(env).expanduser())
    return str(_root_or_cwd() / "models")


def default_output_dir():
    """运行时输出（预览图、录像、标定中间结果）：VR_safe/out/。"""
    return str(_root_or_cwd() / "out")
