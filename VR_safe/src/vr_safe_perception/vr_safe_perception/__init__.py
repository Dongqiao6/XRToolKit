"""VR_safe 相机观察源。

对外入口：
    observation_node   实时相机或录像 → /vr_safe/observation
    calibrate_zone     地面/区域标定
    calibrate_extrinsic 相机↔基座外参标定
    preview_zone       区域预览图
    record_clip        录彩色+深度片段
    replay_clip        回放片段并发布观察
"""

__version__ = "0.1.0"
