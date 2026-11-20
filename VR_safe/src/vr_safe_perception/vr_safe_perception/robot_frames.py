#!/usr/bin/env python3
"""从 JAKA S5 的模型文件里读相机外参，把相机系的点换算到机器人基座系。

为什么不现场标定：相机是固定装在机器人上的，这组位姿在设计图里就定好了。
⚠️ 但 sensors.json 自己标着 `extrinsics_calibrated: false`——那是**设计值**，
不是量出来的。仿真和粗略判断够用；真机上算「人离机械臂多远」之前，
还得实地量一次，装配公差可能是毫米也可能是厘米级。

坐标系有三层，别弄混：

    base_link            机器人基座。+X 前 / +Y 左 / +Z 上（ROS 惯例）
      └─ *_camera        相机外壳。约定同上
           └─ *_optical  光学中心。+X 右 / +Y 下 / +Z 前（OpenCV 惯例）

我们代码里一直用的是**光学系**，和 sensors.json 里 optical_frame_convention
写的完全一致，所以不用再转一道。

用法：

    from robot_frames import RobotFrames
    rf = RobotFrames.load()                  # 自动找 sensors.json
    p_base = rf.to_base(p_cam, "top")        # 相机光学系 → 基座系
    print(rf.describe())

命令行自查：

    python3 robot_frames.py
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

def _sim_asset(*rel):
    """找 JAKA 仿真包里的文件。按以下顺序：

    1. 包目录下的 jaka_s5_sim/（源码树和 --symlink-install 交付包）
    2. 上一级的 jaka_s5_sim/（开发时项目根下）
    3. 安装后的 share/vr_safe_perception/jaka_s5_sim/（setup.py 装的副本）

    都没有就返回第 2 种路径，让调用方报错时能看到一个具体位置。
    """
    for root in (os.path.join(HERE, "jaka_s5_sim"),
                 os.path.join(HERE, "..", "jaka_s5_sim")):
        p = os.path.join(root, "jaka_s5_mujoco_lab-0.3.0", "s5_sim", "assets", *rel)
        if os.path.exists(p):
            return os.path.normpath(p)
    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(get_package_share_directory("vr_safe_perception"),
                             "jaka_s5_sim")
        p = os.path.join(share, "jaka_s5_mujoco_lab-0.3.0", "s5_sim", "assets", *rel)
        if os.path.exists(p):
            return os.path.normpath(p)
    except Exception:
        pass
    return os.path.normpath(os.path.join(
        HERE, "..", "jaka_s5_sim", "jaka_s5_mujoco_lab-0.3.0", "s5_sim", "assets", *rel))


# 仿真包在同目录（交付包）或项目根（开发时）的 jaka_s5_sim/ 下
DEFAULT_SENSORS = _sim_asset("config", "sensors.json")


def rpy_to_R(r, p, y):
    """URDF 的 rpy：绕固定轴 X→Y→Z 依次转，等价于 R = Rz @ Ry @ Rx。

    顺序写反是这类代码最常见的错，而且小角度下看不出来——
    所以下面 self-check 里用一个大角度的例子对拍。
    """
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def make_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def align_to_floor(T_cam_base, origin, normal):
    """把「机器人在相机系的位姿」按实测地面摆正：立直，并落到地面上。

    为什么需要：sensors.json 的外参是**设计值**（它自己标着
    extrinsics_calibrated: false）。相机一旦被挪动——调角度、换位置——
    设计值就不再成立，机器人会歪着飘在半空（11-11 实测：悬空 0.13 m、
    歪 8.8°），可达空间跟着整体偏移，看着就不对。

    地面是现场测出来的，比设计值可信。所以拿地面校正两件事：
      ① 姿态：机器人的竖直轴对齐地面法向（绕两者叉积转过去）
      ② 高度：底盘原点落到地面上

    校正不了的是水平朝向——地面是旋转对称的，看不出机器人朝哪边。
    那个得靠别的办法（看机器人本体、或者标定）。设计值准时，这两项
    校正量都趋近 0，开着不会有坏处。

    返回 (校正后的变换, 抬了多少米, 转了多少度)。
    """
    n = np.asarray(normal, float)
    n = n / (np.linalg.norm(n) + 1e-12)
    o = np.asarray(origin, float)
    T = np.array(T_cam_base, float)
    up = T[:3, :3] @ np.array([0, 0, 1.0])     # 机器人的「上」在相机系

    if float(up @ n) < 0:                      # 法向朝下就翻过来
        n = -n
    axis = np.cross(up, n)
    sa = float(np.linalg.norm(axis))
    ca = float(np.clip(up @ n, -1.0, 1.0))
    deg = math.degrees(math.atan2(sa, ca))
    if sa > 1e-9:
        k = axis / sa
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R_fix = np.eye(3) + math.sin(math.radians(deg)) * K + \
            (1 - math.cos(math.radians(deg))) * (K @ K)   # 罗德里格斯
        T[:3, :3] = R_fix @ T[:3, :3]

    lift = -float((T[:3, 3] - o) @ n)          # 沿法向落到地面
    T[:3, 3] = T[:3, 3] + n * lift
    return T, lift, deg


class RobotFrames:
    """相机光学系 ↔ 机器人基座系的换算。"""

    def __init__(self, cams, meta):
        self.cams = cams          # id -> dict(T_base_optical, parent, model, ...)
        self.meta = meta

    @classmethod
    def load(cls, path=None):
        path = path or DEFAULT_SENSORS
        if not os.path.isfile(path):
            raise FileNotFoundError(
                f"找不到 {path}\n"
                f"把 jaka_s5_mujoco 的包解压到项目根的 jaka_s5_sim/ 下，"
                f"或者用 RobotFrames.load('/路径/sensors.json')")
        with open(path) as f:
            d = json.load(f)

        cams = {}
        for c in d["cameras"]:
            if not c.get("enabled", True):
                continue
            # 两段变换：父连杆 → 相机外壳，再 → 光学中心
            T_parent_cam = make_T(c["xyz"], c["rpy"])
            T_cam_opt = make_T(c.get("optical_xyz", [0, 0, 0]),
                               c.get("optical_rpy", [0, 0, 0]))
            cams[c["id"]] = {
                "parent": c["parent"],
                "model": c.get("model_id"),
                "fovy_deg": c.get("fovy_deg"),
                "optical_frame": c.get("optical_frame"),
                "T_parent_optical": T_parent_cam @ T_cam_opt,
                # 只有挂在 base_link 上的相机才能直接得到到基座的变换。
                # 腕部相机挂在 Link_06 上，要知道当前关节角才能算——
                # 那得接运动学，不在这个文件的职责里。
                "fixed_to_base": c["parent"] == "base_link",
            }
        return cls(cams, {
            "intrinsics_calibrated": d.get("intrinsics_calibrated"),
            "extrinsics_calibrated": d.get("extrinsics_calibrated"),
            "optical_convention": d.get("optical_frame_convention"),
            "source": path,
        })

    def T_base_optical(self, cam_id="top"):
        c = self.cams[cam_id]
        if not c["fixed_to_base"]:
            raise ValueError(
                f"{cam_id} 挂在 {c['parent']} 上，不是 base_link。"
                f"要算它到基座的变换得先知道当前关节角，这里给不了。")
        return c["T_parent_optical"]

    def to_base(self, p_cam, cam_id="top"):
        """相机光学系下的点（米）→ 机器人基座系。支持 (3,) 或 (N,3)。"""
        P = np.atleast_2d(np.asarray(p_cam, float))
        T = self.T_base_optical(cam_id)
        out = (T[:3, :3] @ P.T).T + T[:3, 3]
        return out[0] if np.asarray(p_cam).ndim == 1 else out

    def describe(self):
        L = [f"来源 {self.meta['source']}",
             f"光学系约定 {self.meta['optical_convention']}",
             f"内参已标定 {self.meta['intrinsics_calibrated']}   "
             f"外参已标定 {self.meta['extrinsics_calibrated']}"]
        if not self.meta.get("extrinsics_calibrated"):
            L.append("⚠ 外参是设计值不是实测值，真机算距离前要实地量一次")
        for cid, c in self.cams.items():
            T = c["T_parent_optical"]
            tag = "→base" if c["fixed_to_base"] else f"挂在 {c['parent']}（要关节角）"
            L.append(f"  {cid:12} {c['model']:6} fovy {c['fovy_deg']}°  {tag}")
            if c["fixed_to_base"]:
                o = T[:3, 3]
                L.append(f"               光心在基座系 "
                         f"({o[0]:+.3f}, {o[1]:+.3f}, {o[2]:+.3f}) m")
        return "\n".join(L)


def _self_check(rf):
    """拿几何上能自己验证的量对拍，别光看数字好不好看。"""
    ok = True
    T = rf.T_base_optical("top")
    R, t = T[:3, :3], T[:3, 3]

    # 1) 旋转矩阵得是正交的，行列式为 +1（不是 -1，那是镜像）
    e1 = float(np.abs(R @ R.T - np.eye(3)).max())
    e2 = float(np.linalg.det(R))
    print(f"[1] 正交性误差 {e1:.2e}（应 ~0）   行列式 {e2:+.6f}（应 +1）",
          "✓" if e1 < 1e-9 and abs(e2 - 1) < 1e-9 else "✗")
    ok &= e1 < 1e-9 and abs(e2 - 1) < 1e-9

    # 2) 光心高度应该约等于 sensors.json 里写的 1.666 m（光学偏移只有 1.35 cm）
    print(f"[2] 光心高度 {t[2]:.3f} m（设计值 1.666，光学偏移 1.35 cm 以内）",
          "✓" if abs(t[2] - 1.666) < 0.05 else "✗")
    ok &= abs(t[2] - 1.666) < 0.05

    # 3) 相机光轴（光学系 +Z）在基座系里应该朝前下方：
    #    rpy 的 pitch 是 +0.3142 rad = +18°，URDF 里绕 Y 正转是低头，
    #    所以光轴的 X 分量为正（朝前）、Z 分量为负（朝下）。
    axis = R @ np.array([0, 0, 1.0])
    down = math.degrees(math.asin(-axis[2]))
    print(f"[3] 光轴在基座系 ({axis[0]:+.3f}, {axis[1]:+.3f}, {axis[2]:+.3f})"
          f"  → 俯角 {down:+.1f}°（设计值 18°）",
          "✓" if axis[0] > 0.9 and 15 < down < 21 else "✗")
    ok &= axis[0] > 0.9 and 15 < down < 21

    # 4) 相机正前方 2 m 处的点，换到基座系应该在机器人前方、比光心低
    p = rf.to_base([0.0, 0.0, 2.0], "top")
    drop = t[2] - p[2]
    print(f"[4] 相机正前 2 m 的点 → 基座系 "
          f"({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m，比光心低 {drop:.3f} m"
          f"（2·sin18° = {2*math.sin(math.radians(18)):.3f}）",
          "✓" if abs(drop - 2*math.sin(math.radians(18))) < 0.02 else "✗")
    ok &= abs(drop - 2*math.sin(math.radians(18))) < 0.02

    # 5) 换算不能改变两点间的距离（刚体变换）
    a, b = np.array([0.1, -0.2, 1.5]), np.array([-0.3, 0.4, 2.2])
    d1 = float(np.linalg.norm(a - b))
    d2 = float(np.linalg.norm(rf.to_base(a) - rf.to_base(b)))
    print(f"[5] 距离保持 {d1:.6f} → {d2:.6f} m",
          "✓" if abs(d1 - d2) < 1e-9 else "✗")
    ok &= abs(d1 - d2) < 1e-9
    return ok


if __name__ == "__main__":
    rf = RobotFrames.load(sys.argv[1] if len(sys.argv) > 1 else None)
    print(rf.describe())
    print("\n自查：")
    sys.exit(0 if _self_check(rf) else 1)
