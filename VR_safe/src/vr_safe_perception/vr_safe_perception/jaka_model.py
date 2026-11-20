#!/usr/bin/env python3
"""JAKA S5 双臂机器人：读 URDF、算正运动学、给出机械臂包络。

为什么不用 mujoco：我们要的只是「给定关节角，各连杆在基座系的位姿」，
URDF 里本来就有，自己算几十行就够，还不用往环境里塞 mujoco + scipy。
rerun 能直接加载 STL，所以可视化也不需要仿真器。
（s5_sim 那个包是运动学**检查器**——check_path 算碰撞间隙，用途不一样。）

三件事：

    forward()        关节角 → 每个连杆在 base_link 里的 4×4 位姿
    arm_capsules()   机械臂的胶囊包络，用来算人离它多远
    distance_to()    一堆点到机械臂的最短距离

坐标系：URDF 的 base_link 是 +X 前 / +Y 左 / +Z 上（ROS 惯例）。
相机光学系是 +X 右 / +Y 下 / +Z 前。两者的换算在 robot_frames.py 里，
**别在这个文件里再写一遍**。

自查：

    python3 jaka_model.py
"""
import os
import sys
import xml.etree.ElementTree as ET

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


DEFAULT_URDF = _sim_asset("urdf", "jaka_s5_dual.urdf")

ARM_JOINTS = {
    "left": [f"left_joint_{i}" for i in range(1, 7)],
    "right": [f"right_joint_{i}" for i in range(1, 7)],
}
ARM_LINKS = {
    "left": [f"left_Link_{i:02d}" for i in range(7)],
    "right": [f"right_Link_{i:02d}" for i in range(7)],
}
# 各连杆的包络半径（米）。URDF 里有网格但没有现成的包络，这组数是按
# JAKA S5 的管径估的：根部粗、末端细。宁可估大一点——包络偏大只会让
# 机器人早一点避让，偏小才会撞人。真机前应该用网格实测半径替掉。
LINK_RADIUS = [0.090, 0.080, 0.070, 0.062, 0.055, 0.050, 0.050]

# 按部位上色。URDF 里 69 个连杆一个 material 都没有，不上色的话 rerun
# 全渲成同一种默认灰，整机糊成一团看不出结构。
# 配色按「看演示的人该先看哪里」排：相机和夹爪最亮（方案的主角），
# 臂身次之，底盘和塔架压暗当背景。
PART_COLORS = [
    # (匹配规则, RGB, 说明)
    # 连杆名是 *_camera_link；*_camera_face 是网格文件名，别拿来匹配连杆
    (lambda n: "camera" in n,                   (70, 205, 225), "相机"),
    (lambda n: "finger" in n or "gripper" in n, (245, 150, 55), "夹爪"),
    (lambda n: "tool_mount" in n,               (200, 120, 50), "工具法兰"),
    (lambda n: n.endswith(("Link_02", "Link_04")), (232, 232, 228), "臂管"),
    (lambda n: n.endswith(("Link_01", "Link_03", "Link_05")), (118, 124, 132), "关节壳"),
    (lambda n: n.endswith(("Link_00", "Link_06")), (68, 72, 80), "法兰端盖"),
    (lambda n: "base_plate" in n,               (88, 94, 104), "臂基座板"),
    (lambda n: "tower" in n,                    (96, 102, 112), "塔架"),
    (lambda n: "cabinet" in n,                  (62, 66, 74), "控制柜"),
    (lambda n: "wheel" in n or "caster" in n,   (40, 42, 48), "轮子"),
    (lambda n: "fan" in n or "grille" in n,     (52, 56, 64), "风扇格栅"),
    (lambda n: "deck" in n,                     (108, 114, 124), "载物台"),
    (lambda n: n.startswith("amr") or "bumper" in n or "shell" in n,
                                                (58, 68, 84), "底盘"),
]
DEFAULT_COLOR = (90, 95, 104)


def link_color(name):
    """连杆名 → RGB。规则按顺序匹配，先中先得。"""
    for test, rgb, _ in PART_COLORS:
        if test(name):
            return rgb
    return DEFAULT_COLOR


def _xyz(e, attr, default=(0.0, 0.0, 0.0)):
    if e is None or e.get(attr) is None:
        return np.array(default, float)
    return np.array([float(v) for v in e.get(attr).split()], float)


def rpy_to_R(r, p, y):
    """URDF 的 rpy：绕固定轴 X→Y→Z，等价于 R = Rz @ Ry @ Rx。"""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def axis_angle_R(axis, th):
    """绕任意轴转 th 弧度（罗德里格斯公式）。"""
    a = np.asarray(axis, float)
    a = a / (np.linalg.norm(a) + 1e-12)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


class JakaModel:
    def __init__(self, urdf_path=None):
        path = urdf_path or DEFAULT_URDF
        if not os.path.isfile(path):
            raise FileNotFoundError(
                f"找不到 {path}\n把 jaka_s5_mujoco 包解压到项目根的 jaka_s5_sim/ 下")
        self.path = path
        root = ET.parse(path).getroot()
        self.root_link = None
        self.joints = {}          # 子连杆名 -> 关节信息
        children = set()
        for j in root.findall("joint"):
            child = j.find("child").get("link")
            o = j.find("origin")
            self.joints[child] = {
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "xyz": _xyz(o, "xyz"),
                "rpy": _xyz(o, "rpy"),
                "axis": _xyz(j.find("axis"), "xyz", (0, 0, 1)),
            }
            children.add(child)
        links = [l.get("name") for l in root.findall("link")]
        for l in links:
            if l not in children:
                self.root_link = l
                break
        self.links = links
        # 连杆的可视网格，给 rerun 加载用。
        # ⚠️ 一个 link 可以有**多个** visual：这个模型里臂管和关节壳是分开的
        # 两个 STL（Link_02_visual_housing + Link_02_visual_silver），底盘某些
        # 件甚至有 21 个。只取第一个的话，臂管全丢，渲染出来是一截截散开的
        # 关节壳——11-11 就是这么踩的。所以这里存的是**列表**。
        self.meshes = {}          # 连杆名 -> [零件, …]
        base = os.path.dirname(path)
        for l in root.findall("link"):
            parts = []
            for v in l.findall("visual"):
                m = v.find("geometry/mesh")
                if m is None:
                    continue      # box/cylinder 这类基本体暂不画
                f = m.get("filename", "")
                if f.startswith("package://"):
                    f = f[len("package://"):].split("/", 1)[-1]
                fp = f if os.path.isabs(f) else os.path.normpath(
                    os.path.join(base, "..", f))
                o = v.find("origin")
                parts.append({
                    "file": fp, "exists": os.path.isfile(fp),
                    "xyz": _xyz(o, "xyz"), "rpy": _xyz(o, "rpy"),
                    "scale": _xyz(m, "scale", (1.0, 1.0, 1.0)),
                    "name": os.path.basename(fp),
                })
            if parts:
                self.meshes[l.get("name")] = parts

    def joint_names(self):
        return [j["name"] for j in self.joints.values() if j["type"] != "fixed"]

    def forward(self, q=None):
        """关节角 → 每个连杆在 base_link 里的 4×4 位姿。

        q 是 {关节名: 弧度}，没给的关节按 0 算。
        """
        q = q or {}
        by_name = {v["name"]: k for k, v in self.joints.items()}
        T = {self.root_link: np.eye(4)}

        def resolve(link):
            if link in T:
                return T[link]
            j = self.joints.get(link)
            if j is None:                 # 不在树上，当成固定在根上
                T[link] = np.eye(4)
                return T[link]
            Tp = resolve(j["parent"])
            L = np.eye(4)
            L[:3, :3] = rpy_to_R(*j["rpy"])
            L[:3, 3] = j["xyz"]
            if j["type"] in ("revolute", "continuous"):
                th = float(q.get(j["name"], 0.0))
                R = np.eye(4)
                R[:3, :3] = axis_angle_R(j["axis"], th)
                L = L @ R
            elif j["type"] == "prismatic":
                d = float(q.get(j["name"], 0.0))
                D = np.eye(4)
                a = np.asarray(j["axis"], float)
                D[:3, 3] = a / (np.linalg.norm(a) + 1e-12) * d
                L = L @ D
            T[link] = Tp @ L
            return T[link]

        for l in self.links:
            resolve(l)
        return T

    def arm_capsules(self, q=None, arms=("left", "right")):
        """机械臂的胶囊包络：[(起点, 终点, 半径), …]，base_link 系。

        相邻两个连杆原点连成一段，套一个半径。算「人离机械臂多远」时，
        点到线段的距离减去半径就是到臂表面的距离——比用网格快几个数量级，
        而且偏保守（包络比实物粗）。
        """
        T = self.forward(q)
        caps = []
        for arm in arms:
            names = ARM_LINKS[arm]
            pts = [T[n][:3, 3] for n in names if n in T]
            for i in range(len(pts) - 1):
                caps.append((pts[i], pts[i + 1],
                             LINK_RADIUS[min(i, len(LINK_RADIUS) - 1)]))
        return caps

    def reach_spheres(self, margin=0.0, arms=("left", "right"), samples=4000):
        """机械臂的**可达空间**，当保护范围用：[(球心, 半径), …]，base_link 系。

        和 arm_capsules 的区别很重要：

            arm_capsules   机械臂**此刻**占的位置。要实时关节角，范围小
            reach_spheres  机械臂**可能**到达的位置。装好就固定，范围大

        做保护范围该用后者：人走进可达空间就有危险，不管臂现在在哪——
        下一秒它就可能摆过去。而且不需要实时关节角，少一条依赖，
        执行侧挂了也不影响感知这边判断。

        半径取两个的大者：
          解析上界  各连杆首尾拉直的长度。6R 臂通常能接近伸直，这是个
                    偏保守（偏大）的估计——保护范围宁大勿小
          随机采样  实际采到的最远距离，用来验证上界没算错

        球心取第一个转轴（Link_01 的原点），那是臂的肩部。
        """
        T0 = self.forward()
        out = []
        for arm in arms:
            names = [n for n in ARM_LINKS[arm] if n in T0]
            if len(names) < 2:
                continue
            o = T0[names[1]][:3, 3]          # Link_01 = 肩
            pts = [T0[n][:3, 3] for n in names[1:]]
            analytic = sum(float(np.linalg.norm(pts[i+1] - pts[i]))
                           for i in range(len(pts) - 1))
            # 采样验证：随机关节角下末端离肩最远多少
            rng = np.random.default_rng(0)
            jn = ARM_JOINTS[arm]
            best = 0.0
            for _ in range(max(0, samples)):
                q = {n: float(v) for n, v in zip(jn, rng.uniform(-np.pi, np.pi, len(jn)))}
                T = self.forward(q)
                best = max(best, float(np.linalg.norm(T[names[-1]][:3, 3] - o)))
            r = max(analytic, best) + float(margin)
            out.append((o, r, {"解析上界": round(analytic, 4),
                               "采样最远": round(best, 4),
                               "余量": float(margin), "臂": arm}))
        return out

    def joint_limits(self, arm):
        """读关节限位，给随机动作用。没写 limit 的按 ±π 算。"""
        by_name = {v["name"]: v for v in self.joints.values()}
        out = {}
        root = ET.parse(self.path).getroot()
        lim = {j.get("name"): j.find("limit") for j in root.findall("joint")}
        for n in ARM_JOINTS[arm]:
            L = lim.get(n)
            if L is not None and L.get("lower") and L.get("upper"):
                out[n] = (float(L.get("lower")), float(L.get("upper")))
            else:
                out[n] = (-np.pi, np.pi)
        return out

    def wander(self, t, arms=("left", "right"), speed=0.25, span=0.6, seed=0):
        """给演示用的「随手动动」关节角：几个不同频率的正弦叠起来。

        用正弦不用随机游走：随机游走会越飘越远、还会抖；正弦连续可导，
        看起来像在匀速作业，而且 t 一样结果就一样，录屏能复现。

        span 控制摆幅（占关节行程的比例），speed 控制快慢。
        每个关节的频率和相位都不同，整条臂就不会像节拍器一样齐动。
        """
        q = {}
        k = 0
        for arm in arms:
            lims = self.joint_limits(arm)
            for n in ARM_JOINTS[arm]:
                lo, hi = lims[n]
                # 限位有的是 ±2π（关节能转好几圈），全幅摆太夸张，
                # 压到 ±90° 以内看着才像在干活
                lo, hi = max(lo, -np.pi/2), min(hi, np.pi/2)
                mid, half = (lo + hi) / 2, (hi - lo) / 2 * span
                f = 0.13 + 0.047 * ((k * 7 + seed) % 9)      # 各关节频率错开
                ph = 0.7 * ((k * 5 + seed) % 11)
                q[n] = mid + half * np.sin(2 * np.pi * f * speed * t + ph)
                k += 1
        return q

    @staticmethod
    def point_in_spheres(points, spheres):
        """点在不在可达空间里，以及到球面的距离（球内为正）。"""
        P = np.atleast_2d(np.asarray(points, float))
        best = np.full(len(P), -np.inf)
        for sp in spheres:
            o, r = sp[0], sp[1]
            best = np.maximum(best, r - np.linalg.norm(P - o, axis=1))
        return best

    @staticmethod
    def distance_to(points, capsules):
        """一堆点到机械臂表面的最短距离（米）。负数表示已经插进包络里了。"""
        P = np.atleast_2d(np.asarray(points, float))
        if not len(P) or not capsules:
            return None, None
        best = np.full(len(P), np.inf)
        for a, b, r in capsules:
            ab = b - a
            L2 = float(ab @ ab) + 1e-12
            t = np.clip((P - a) @ ab / L2, 0.0, 1.0)[:, None]
            d = np.linalg.norm(P - (a + t * ab), axis=1) - r
            best = np.minimum(best, d)
        k = int(np.argmin(best))
        return float(best[k]), k


def _self_check():
    m = JakaModel()
    print(f"URDF {m.path}")
    print(f"根连杆 {m.root_link}   连杆 {len(m.links)}   "
          f"可动关节 {len(m.joint_names())}")
    n_part = sum(len(v) for v in m.meshes.values())
    have = sum(1 for v in m.meshes.values() for p in v if p["exists"])
    multi = sum(1 for v in m.meshes.values() if len(v) > 1)
    print(f"可视零件 {have}/{n_part} 个找得到文件，分布在 {len(m.meshes)} 个连杆上"
          f"（其中 {multi} 个连杆有多个零件）")

    ok = True
    T0 = m.forward()
    # 1) 两臂的**安装点**应该左右对称。注意末端不对称是正常的：
    #    两臂同轴反向安装，安装 rpy 一个 -π/2 一个 +π/2，
    #    所以同样的关节角会指向不同方向。查安装点才查得准。
    l0, r0 = T0["left_Link_00"][:3, 3], T0["right_Link_00"][:3, 3]
    sym = (abs(l0[1] + r0[1]) < 1e-6 and abs(l0[0] - r0[0]) < 1e-6
           and abs(l0[2] - r0[2]) < 1e-6)
    l6, r6 = T0["left_Link_06"][:3, 3], T0["right_Link_06"][:3, 3]
    print(f"[1] 两臂安装点对称  左{np.round(l0,4)}  右{np.round(r0,4)}",
          "✓" if sym else "✗")
    print(f"    （零位末端 左{np.round(l6,3)} 右{np.round(r6,3)}——"
          f"同轴反向安装，末端本来就不镜像）")
    ok &= sym

    # 2) 转一个关节，末端必须动；不动说明关节没接上树
    T1 = m.forward({"left_joint_1": 0.5})
    moved = float(np.linalg.norm(T1["left_Link_06"][:3, 3] - l6))
    print(f"[2] left_joint_1 转 0.5 rad，末端移动 {moved*1000:.0f} mm",
          "✓" if moved > 0.01 else "✗")
    ok &= moved > 0.01

    # 3) 绕自身轴转末端关节，末端原点不该移动（只转姿态）
    T2 = m.forward({"left_joint_6": 1.0})
    moved6 = float(np.linalg.norm(T2["left_Link_06"][:3, 3] - l6))
    print(f"[3] left_joint_6 转 1.0 rad，末端原点移动 {moved6*1000:.2f} mm（应 ~0）",
          "✓" if moved6 < 1e-6 else "✗")
    ok &= moved6 < 1e-6

    # 4) 旋转矩阵保持正交
    e = max(float(np.abs(T[:3, :3] @ T[:3, :3].T - np.eye(3)).max())
            for T in T0.values())
    print(f"[4] 所有连杆旋转矩阵正交，最大误差 {e:.2e}", "✓" if e < 1e-9 else "✗")
    ok &= e < 1e-9

    # 5) 距离计算：取左臂末端自己当测试点，距离应约等于 -半径（在包络里面）
    caps = m.arm_capsules()
    d, _ = m.distance_to([l6], caps)
    print(f"[5] 末端点到臂表面 {d*1000:+.0f} mm（在包络内应为负，约 -50）",
          "✓" if -0.12 < d < 0 else "✗")
    ok &= -0.12 < d < 0

    # 6) 远处的点距离应为正，且量级合理
    far = l6 + np.array([0.0, 0.0, 1.0])
    d2, _ = m.distance_to([far], caps)
    print(f"[6] 末端正上方 1 m 处的点 {d2*1000:+.0f} mm（应为正、小于 1000）",
          "✓" if 0 < d2 < 1.0 else "✗")
    ok &= 0 < d2 < 1.0
    print(f"\n包络段数 {len(caps)}")

    # 7) 可达空间：采样最远不该超过解析上界（能超说明上界算错了）
    sp = m.reach_spheres(margin=0.0, samples=800)
    ok7 = True
    for o, r, meta in sp:
        over = meta["采样最远"] > meta["解析上界"] + 1e-6
        print(f"[7] {meta['臂']:5} 可达半径 {r:.3f} m"
              f"（解析上界 {meta['解析上界']:.3f}，采样最远 {meta['采样最远']:.3f}）"
              f"  球心 {np.round(o, 3)}", "✗ 采样超出上界" if over else "✓")
        ok7 &= not over
    ok &= ok7
    return ok


if __name__ == "__main__":
    sys.exit(0 if _self_check() else 1)
