#!/usr/bin/env python3
"""不动机械臂，标定「相机 ↔ 机器人基座」的相对位置：模型对点云 + 地面约束。

思路（11-16 用户定）：

    1. 从控制器 10000 端口**只读**当前关节角（不发任何指令，臂不会动）
    2. 把 JAKA 三维模型摆成同样姿态，在网格表面均匀采样成点云（带法向）
    3. 相机取 N 帧深度做时域中值，反投影成点云
    4. 两份点云配准，求相机相对机器人基座的位姿

地面约束：机器人底座和相机支架立在同一块地面上。实测地面（zone_config.json 的
plane）已经定了机器人的竖直方向和高度（align_to_floor），剩下只需求水平面内
3 个量——绕地面法向的转角 θ、地面内平移 tx、ty。6 自由度降成 3 自由度，
少了倾斜和高度这几个最容易被噪声带偏的方向，解更稳。
另外不加地面约束做一次 6 自由度配准作对照：两者差得多，说明地面或模型有问题。

坐标约定：
    T_cam_base  基座系 → 相机光学系（和 zone_rerun.py 里同名变量一致）
    base0       「设计值 + align_to_floor」得到的初值基座系；地面就是它的 z=0 面
    θ/tx/ty     机器人在 base0 里实际的位置：绕 base0 的 z 轴转 θ，再平移 (tx, ty)
                T_cam_base(标定) = T_cam_base(初值) @ [Rz(θ) | (tx, ty, 0)]

现场实跑（先停检测程序让出相机，跑完再起）：

    cd ~/camera_zone && ./stop.sh keep-view
    ./.venv/bin/python calib_extrinsic.py              # 标定，结果写 extrinsic_calib.json
    POSE_BACKEND=gpu ./start.sh                        # 起回检测，自动用上标定值

其他用法：

    ./.venv/bin/python calib_extrinsic.py --dry-run     # 只算不写主结果文件
    ./.venv/bin/python calib_extrinsic.py --from out/calib/<时间戳>   # 用存下的采集重算
    ./.venv/bin/python calib_extrinsic.py --selftest    # 合成数据自测，不碰相机和控制器

只依赖 numpy + opencv（JAKA 的 .venv 里没有 scipy/open3d/trimesh；有 scipy 时用它
的 KD 树，否则用 opencv 自带的 FLANN）。
"""
import argparse
import json
import math
import os
import shutil
import struct
import sys
import time

import numpy as np

from .paths import default_calibration_dir, vr_safe_root

HERE = os.path.dirname(os.path.abspath(__file__))
# 标定产物落到 VR_safe/config/calibration/，中间结果落到 VR_safe/out/calib/。
_CALIB_DIR = default_calibration_dir()
CALIB_FILE = os.environ.get(
    "EXTRINSIC_CALIB_FILE", os.path.join(_CALIB_DIR, "extrinsic_calib.json"))
ZONE_CFG = os.environ.get(
    "ZONE_CFG", os.path.join(_CALIB_DIR, "zone_config.json"))
OUT_DIR = os.environ.get(
    "CALIB_OUT_DIR", os.path.join(str(vr_safe_root() or HERE), "out", "calib"))

# 相机现在装在独立支架上，不在机器人塔架上——模型里那个顶部相机外壳实物上没有
DEFAULT_EXCLUDE = ("top_camera_link",)
# --static-only：只留不随关节动的部分（底盘、控制柜、塔架、风扇罩、两臂安装座 Link_00）
STATIC_EXCLUDE = tuple(f"_Link_0{i}" for i in range(1, 7)) + ("gripper", "finger",
                                                              "wrist_camera")

# 由粗到细的裁剪半径（米）：只用离模型表面这么近的点。
# 第一档要盖住初值误差（±10°、±10 cm 时臂端最多偏 ~25 cm）
RADII = (0.30, 0.20, 0.12, 0.08, 0.05, 0.03, 0.02)
FLOOR_CLEAR = 0.05          # 只用高于地面 5 cm 以上的点（去掉地面）
INLIER_M = 0.02             # 质量指标里「内点」的门槛


# ═════════════════════════════════ 小工具 ═════════════════════════════════

def Rz(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def planar_T(th, tx, ty):
    T = np.eye(4)
    T[:3, :3] = Rz(th)
    T[:3, 3] = [tx, ty, 0.0]
    return T


def rotvec_R(w):
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    k = np.asarray(w, float) / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def rot_angle_deg(R):
    return math.degrees(math.acos(float(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def xform(T, P):
    return P @ T[:3, :3].T + T[:3, 3]


def load_plane(path=ZONE_CFG):
    """每次现读 zone_config.json 的地面，不缓存（相机挪过、地面重标过都能跟上）。"""
    with open(path) as f:
        pl = (json.load(f).get("plane") or {})
    if not (pl.get("origin") and pl.get("normal")):
        raise RuntimeError(f"{path} 里没有 plane（地面还没标）。先跑 zone_tune.py 标地面")
    return {"origin": [float(v) for v in pl["origin"]],
            "normal": [float(v) for v in pl["normal"]],
            "file_mtime": time.strftime("%Y-%m-%d %H:%M:%S",
                                        time.localtime(os.path.getmtime(path)))}


def plane_diff(pa, pb):
    """两份地面差多少：(法向夹角 度, 原点沿法向的距离差 米)。"""
    na = np.asarray(pa["normal"], float); na /= np.linalg.norm(na)
    nb = np.asarray(pb["normal"], float); nb /= np.linalg.norm(nb)
    if na @ nb < 0:
        nb = -nb
    ang = math.degrees(math.acos(float(np.clip(na @ nb, -1, 1))))
    # 相机光心到两个平面的距离差，比比原点更有意义（原点是平面上任取的一点）
    da = float(np.asarray(pa["origin"]) @ na)
    db = float(np.asarray(pb["origin"]) @ nb)
    return ang, abs(da - db)


def initial_T_cam_base(plane, camera_id="top"):
    """现行做法：设计值取逆 + 按实测地面摆正。标定的初值，也是 3 自由度的参考系。"""
    from .robot_frames import RobotFrames, align_to_floor
    T = np.linalg.inv(RobotFrames.load().T_base_optical(camera_id))
    T, lift, deg = align_to_floor(T, plane["origin"], plane["normal"])
    return T, lift, deg


# ═══════════════════════ 接线用：启动时选哪套外参 ═══════════════════════

def choose_T_cam_base(T_design_floor, plane=None, path=CALIB_FILE, log=print,
                      fallback_name="设计值 + 地面摆正"):
    """zone_rerun.py / camera_zone.py 启动时调。

    有 extrinsic_calib.json、没设 EXTRINSIC=design、且标定时用的地面和现在的
    一致，就用标定值；否则用传进来的「设计值 + 地面摆正」。返回 (T_cam_base, 说明)。

    为什么要核地面：标定结果只对那一次的相机位置有效。相机一挪，地面会重标，
    这时旧标定值肯定是错的，而「设计值 + 地面摆正」至少竖直方向和高度是对的。
    """
    if os.environ.get("EXTRINSIC", "").lower() == "design":
        msg = f"外参：{fallback_name}（EXTRINSIC=design 指定）"
        log(msg)
        return T_design_floor, msg
    if not os.path.isfile(path):
        msg = f"外参：{fallback_name}（还没有 extrinsic_calib.json）"
        log(msg)
        return T_design_floor, msg
    try:
        with open(path) as f:
            d = json.load(f)
        T = np.asarray(d["T_cam_base"], float).reshape(4, 4)
        if plane is None:
            plane = load_plane()
        ang, dist = plane_diff(plane, d["plane"])
        if ang > 0.5 or dist > 0.02:
            msg = (f"外参：{fallback_name}（⚠ extrinsic_calib.json 已过期：标定时的地面"
                   f"和现在差 {ang:.2f}°、{dist*100:.1f} cm，相机可能挪过，要重新标定）")
            log(msg)
            return T_design_floor, msg
        c = d.get("correction", {})
        msg = (f"外参：标定值 extrinsic_calib.json（{d.get('time', '?')}，"
               f"在初值上转 {c.get('theta_deg', 0):+.2f}°、平移 "
               f"({c.get('tx_m', 0)*100:+.1f}, {c.get('ty_m', 0)*100:+.1f}) cm）")
        log(msg)
        return T, msg
    except Exception as e:
        msg = f"外参：{fallback_name}（extrinsic_calib.json 读不了：{e}）"
        log(msg)
        return T_design_floor, msg


# ══════════════════════════════ 模型点云 ══════════════════════════════

def load_mesh(path):
    """读 STL（二进制/文本）或 OBJ，返回 (顶点 (V,3), 三角形 (F,3))。"""
    if path.lower().endswith(".obj"):
        V, F = [], []
        with open(path, errors="ignore") as f:
            for line in f:
                if line.startswith("v "):
                    V.append([float(x) for x in line.split()[1:4]])
                elif line.startswith("f "):
                    idx = []
                    for tok in line.split()[1:]:
                        i = int(tok.split("/")[0])
                        idx.append(i - 1 if i > 0 else len(V) + i)
                    for k in range(1, len(idx) - 1):        # 多边形按扇形拆三角
                        F.append([idx[0], idx[k], idx[k + 1]])
        return np.asarray(V, float), np.asarray(F, int)
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0] if len(b) >= 84 else 0
    if len(b) == 84 + 50 * n:                                  # 二进制 STL
        a = np.frombuffer(b[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)),
                                                  ("a", "<u2")]))
        V = a["v"].reshape(-1, 3).astype(float)
        return V, np.arange(len(V)).reshape(-1, 3)
    V = [[float(x) for x in ln.split()[1:4]]                   # 文本 STL
         for ln in b.decode("utf-8", "ignore").splitlines()
         if ln.strip().startswith("vertex")]
    V = np.asarray(V, float)
    return V, np.arange(len(V)).reshape(-1, 3)


def sample_surface(V, F, spacing, rng):
    """按面积在三角面上均匀撒点，约每 spacing² 一个。返回 (点, 单位法向)。"""
    A, B, C = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    cr = np.cross(B - A, C - A)
    area = 0.5 * np.linalg.norm(cr, axis=1)
    ok = area > 1e-12
    A, B, C, cr, area = A[ok], B[ok], C[ok], cr[ok], area[ok]
    if not len(area):
        return np.zeros((0, 3)), np.zeros((0, 3))
    n = max(1, int(round(area.sum() / spacing ** 2)))
    fi = rng.choice(len(area), size=n, p=area / area.sum())
    u, v = rng.random(n), rng.random(n)
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    P = A[fi] + u[:, None] * (B[fi] - A[fi]) + v[:, None] * (C[fi] - A[fi])
    N = cr[fi] / (2 * area[fi])[:, None]
    return P, N


_MESH_CACHE = {}


def model_cloud(robot, q, spacing=0.006, exclude=DEFAULT_EXCLUDE, seed=0, only=()):
    """按关节角摆好整机，在网格表面采样。返回 (点, 法向, 连杆名数组)，基座系。"""
    from .jaka_model import rpy_to_R
    rng = np.random.default_rng(seed)
    T_links = robot.forward(q)
    Ps, Ns, Ls = [], [], []
    for link, parts in robot.meshes.items():
        if link not in T_links or any(e and e in link for e in exclude):
            continue
        if only and not any(o in link for o in only):
            continue
        for mi in parts:
            if not mi["exists"]:
                continue
            if mi["file"] not in _MESH_CACHE:
                _MESH_CACHE[mi["file"]] = load_mesh(mi["file"])
            V, F = _MESH_CACHE[mi["file"]]
            M = np.eye(4)
            M[:3, :3] = rpy_to_R(*mi["rpy"])
            M[:3, 3] = mi["xyz"]
            T = T_links[link] @ M
            Vs = V * np.asarray(mi["scale"], float)
            P, N = sample_surface(Vs, F, spacing, rng)
            Ps.append(xform(T, P))
            Ns.append(N @ T[:3, :3].T)
            Ls += [link] * len(P)
    return np.vstack(Ps), np.vstack(Ns), np.asarray(Ls)


def visible_mask(P_base, N_base, T_cam_base, intr, cell=4, tol=0.015, graze=0.15):
    """模型点里，从相机这个位姿看得见的那些：在画面里、没被模型自己挡住、不是掠射面。

    遮挡用粗 z 缓冲（cell×cell 像素一格，比格内最近点远 tol 以上的算被挡）。
    法向朝向不可靠（网格绕序不一定一致），所以不做背面剔除，只靠 z 缓冲。
    """
    Pc = xform(T_cam_base, P_base)
    Nc = N_base @ T_cam_base[:3, :3].T
    z = Pc[:, 2]
    ok = z > 0.15
    u = np.full(len(z), -1.0)
    v = np.full(len(z), -1.0)
    u[ok] = Pc[ok, 0] / z[ok] * intr.fx + intr.ppx
    v[ok] = Pc[ok, 1] / z[ok] * intr.fy + intr.ppy
    ok &= (u >= 0) & (u < intr.width) & (v >= 0) & (v < intr.height)
    view = -Pc / (np.linalg.norm(Pc, axis=1, keepdims=True) + 1e-12)
    ok &= np.abs((Nc * view).sum(1)) > graze
    gw, gh = intr.width // cell + 1, intr.height // cell + 1
    key = np.zeros(len(z), int)
    key[ok] = (v[ok] // cell).astype(int) * gw + (u[ok] // cell).astype(int)
    zbuf = np.full(gw * gh, np.inf)
    np.minimum.at(zbuf, key[ok], z[ok])
    ok[ok] = z[ok] <= zbuf[key[ok]] + tol
    return ok


# ══════════════════════════════ 最近邻 ══════════════════════════════

class NN:
    """最近邻查询。有 scipy 用 KD 树，否则 opencv FLANN，再不行分块暴力。"""

    def __init__(self, P):
        self.P = np.ascontiguousarray(P, np.float32).reshape(-1, 3)
        self.kind = None
        if len(self.P) < 2:     # opencv FLANN 吃到空数组会直接段错误（11-16 踩过）
            raise RuntimeError(f"最近邻索引只有 {len(self.P)} 个点，没法配准")
        try:
            from scipy.spatial import cKDTree
            self.tree, self.kind = cKDTree(self.P), "scipy"
            return
        except Exception:
            pass
        try:
            import cv2
            self.idx = cv2.flann_Index(self.P, dict(algorithm=1, trees=4))
            self.kind = "flann"
            return
        except Exception:
            self.kind = "brute"

    def query(self, Q):
        Q = np.ascontiguousarray(Q, np.float32)
        if not len(Q):
            return np.zeros(0), np.zeros(0, int)
        if self.kind == "scipy":
            d, i = self.tree.query(Q, k=1)
            return d.astype(float), i.astype(int)
        if self.kind == "flann":
            i, d2 = self.idx.knnSearch(Q, 1, params=dict(checks=64))
            return np.sqrt(np.maximum(d2[:, 0], 0)).astype(float), i[:, 0].astype(int)
        d_all, i_all = np.empty(len(Q)), np.empty(len(Q), int)
        p2 = (self.P.astype(float) ** 2).sum(1)
        for s in range(0, len(Q), 2048):
            q = Q[s:s + 2048].astype(float)
            D = (q ** 2).sum(1)[:, None] + p2[None] - 2 * q @ self.P.T.astype(float)
            k = np.argmin(D, axis=1)
            i_all[s:s + 2048] = k
            d_all[s:s + 2048] = np.sqrt(np.maximum(D[np.arange(len(k)), k], 0))
        return d_all, i_all


# ══════════════════════════════ 配准 ══════════════════════════════

class Model:
    """采样好的模型点云 + 按当前位姿挑可见点并建最近邻索引。"""

    def __init__(self, P, N, links, intr):
        self.P, self.N, self.links, self.intr = P, N, links, intr

    def visible(self, T_cam_base):
        m = visible_mask(self.P, self.N, T_cam_base, self.intr)
        return self.P[m], self.N[m], m


def icp(S, model, T0, dof, G_init=None, radii=RADII, iters=25, log=None, trunc_from=0.08):
    """把场景点 S（base0 系）配到模型上。

    求 G：G @ s ≈ 模型点。dof=3 时 G = [Rz(φ) | (a, b, 0)]，高度和倾斜锁死；
    dof=6 时 G 是一般刚体变换。点到面残差 + Huber 鲁棒核，裁剪半径由粗到细。
    相机看模型的位姿 = T0 @ inv(G)，每换一档半径按它重算一次可见点。
    返回 (G, 每档记录)。
    """
    G = np.eye(4) if G_init is None else G_init.copy()
    hist = []
    for r in radii:
        Pm, Nm, _ = model.visible(T0 @ np.linalg.inv(G))
        if len(Pm) < 100:
            raise RuntimeError(f"模型在画面里几乎看不见（{len(Pm)} 个可见点），初值太离谱")
        nn = NN(Pm)
        delta = max(0.006, r / 4)
        for it in range(iters):
            X = xform(G, S)
            d, idx = nn.query(X)
            sel = d < r
            if sel.sum() < 50:
                raise RuntimeError(f"半径 {r:.2f} m 内只剩 {sel.sum()} 个点，配不上")
            p, q, n = X[sel], Pm[idx[sel]], Nm[idx[sel]]
            res = ((p - q) * n).sum(1)
            a = np.abs(res)
            w = np.where(a <= delta, 1.0, delta / np.maximum(a, 1e-12))
            if r <= trunc_from:          # 细档用截断核：残差超过 2.5δ 的直接不算（管线、线缆等）
                w[a > 2.5 * delta] = 0.0
            if dof == 1:                 # 只沿基座 x（机器人前后）平移
                J = n[:, :1]
            elif dof == 3:
                J = np.stack([-p[:, 1] * n[:, 0] + p[:, 0] * n[:, 1],
                              n[:, 0], n[:, 1]], axis=1)
            else:
                J = np.hstack([np.cross(p, n), n])
            H = (J * w[:, None]).T @ J
            g = (J * w[:, None]).T @ res
            H += 1e-6 * np.trace(H) / len(H) * np.eye(len(H))
            x = -np.linalg.solve(H, g)
            D = np.eye(4)
            if dof == 1:
                D[0, 3] = x[0]
                step_r, step_t = 0.0, abs(x[0])
            elif dof == 3:
                D[:3, :3] = Rz(x[0])
                D[:3, 3] = [x[1], x[2], 0.0]
                step_r, step_t = abs(x[0]), math.hypot(x[1], x[2])
            else:
                D[:3, :3] = rotvec_R(x[:3])
                D[:3, 3] = x[3:]
                step_r, step_t = float(np.linalg.norm(x[:3])), float(np.linalg.norm(x[3:]))
            G = D @ G
            if step_r < 1e-5 and step_t < 1e-5:
                break
        hist.append({"radius_m": r, "iters": it + 1, "n_used": int(sel.sum()),
                     "huber_mm": round(delta * 1000, 1),
                     "med_abs_res_mm": round(float(np.median(a)) * 1000, 2)})
        if log:
            log(f"    半径 {r*100:4.0f} cm：{it+1:2d} 轮，用了 {sel.sum():6d} 点，"
                f"残差中位 {np.median(a)*1000:5.1f} mm")
    return G, hist


def coarse_search(S, model, th_deg=40, th_step=4, tx=0.8, ty=0.5, t_step=0.08,
                  n_sub=3000, top=5, log=None):
    """水平面内粗搜初值：θ×tx×ty 网格上数「4 cm 内有模型点的场景点数」，取前几个互不相近的。

    为什么要：设计值可能离实际很远（11-16 实测差 40 多厘米），ICP 只在初值附近
    十几厘米内收敛。粗搜用全部模型点（不挑可见），场景点抽到 n_sub 个，几十秒。
    返回 [(得分, G), …]，G 是 planar_T 形式（场景 → 模型）。
    """
    rng = np.random.default_rng(3)
    Ss = S[rng.choice(len(S), min(n_sub, len(S)), replace=False)]
    nn = NN(model.P)
    cand = []
    for th in np.arange(-th_deg, th_deg + 1e-9, th_step):
        R = Rz(math.radians(th))
        X0 = Ss @ R.T
        for a_ in np.arange(-tx, tx + 1e-9, t_step):
            for b_ in np.arange(-ty, ty + 1e-9, t_step):
                d, _ = nn.query(X0 + [a_, b_, 0.0])
                cand.append((int((d < 0.04).sum()), th, a_, b_))
    cand.sort(key=lambda c: -c[0])
    picked = []
    for c in cand:
        if all(abs(c[1] - p[1]) > 8 or math.hypot(c[2] - p[2], c[3] - p[3]) > 0.15
               for p in picked):
            picked.append(c)
        if len(picked) >= top:
            break
    if log:
        log("  粗搜前几名（得分=4 cm 内场景点数 / " + str(len(Ss)) + "）：" + "；".join(
            f"{c[0]} @ θ{c[1]:+.0f}° ({c[2]*100:+.0f},{c[3]*100:+.0f})cm" for c in picked))
    return [(c[0], planar_T(math.radians(c[1]), c[2], c[3])) for c in picked]


def planar_params(G):
    """G 把场景配到模型上；机器人在 base0 里的实际位置是 inv(G)。返回 (θ 弧度, tx, ty)。"""
    D = np.linalg.inv(G)
    return math.atan2(D[1, 0], D[0, 0]), float(D[0, 3]), float(D[1, 3])


def evaluate(S, model, T0, G):
    """在位姿 G 下的质量指标。

    内点比例   离模型 10 cm 以内的场景点里，有多少在 2 cm 以内
    残差       内点到模型表面的点到面距离，中位/90 分位（mm）
    覆盖率     模型可见点里，2 cm 内有场景点的比例（看模型有多少部分被实物「认领」）
    """
    Pm, Nm, _ = model.visible(T0 @ np.linalg.inv(G))
    X = xform(G, S)
    d, idx = NN(Pm).query(X)
    band = d < 0.10
    inl = d < INLIER_M
    res = np.abs(((X[inl] - Pm[idx[inl]]) * Nm[idx[inl]]).sum(1))
    dm, _ = NN(X).query(Pm)
    return {
        "n_scene_band": int(band.sum()),
        "n_inlier": int(inl.sum()),
        "inlier_ratio": round(float(inl.sum() / max(1, band.sum())), 4),
        "res_med_mm": round(float(np.median(res)) * 1000, 2) if len(res) else None,
        "res_p90_mm": round(float(np.percentile(res, 90)) * 1000, 2) if len(res) else None,
        "n_model_visible": int(len(Pm)),
        "model_coverage": round(float((dm < INLIER_M).mean()), 4),
    }


def link_group(name):
    """连杆名 → 部位（回执里按部位汇报用）。"""
    n = str(name)
    if "tower" in n or "fan" in n:
        return "塔架"
    if any(k in n for k in ("amr", "bumper", "shell", "deck", "cabinet", "wheel", "caster")):
        return "底盘/控制柜"
    if "gripper" in n or "finger" in n:
        return "夹爪"
    if "camera" in n:
        return "腕部相机"
    for side, cn in (("left", "左臂"), ("right", "右臂")):
        if n.startswith(side + "_Link_"):
            return f"{cn} Link_{n[-2:]}"
    return "其他"


def link_stats(S, model, T0, G):
    """按部位统计：模型可见点数、被实物点「认领」的模型点数、贡献的内点数。

    认领 = 某个模型可见点是某个内点（2 cm 内）的最近点。贡献内点 = 内点按最近
    模型点归到部位。主要靠哪几段，看「贡献内点」那一列。
    """
    Pm, Nm, m = model.visible(T0 @ np.linalg.inv(G))
    Lv = model.links[m]
    X = xform(G, S)
    d, idx = NN(Pm).query(X)
    inl = d < INLIER_M
    groups = np.array([link_group(l) for l in Lv])
    claimed = np.zeros(len(Pm), bool)
    claimed[idx[inl]] = True
    out = {}
    tot_inl = max(1, int(inl.sum()))
    for g in sorted(set(groups)):
        k = groups == g
        n_in = int(np.isin(idx[inl], np.where(k)[0]).sum())
        out[g] = {"visible": int(k.sum()), "claimed": int(claimed[k].sum()),
                  "claimed_ratio": round(float(claimed[k].mean()), 3) if k.any() else 0.0,
                  "inliers": n_in, "inlier_share": round(n_in / tot_inl, 3)}
    return dict(sorted(out.items(), key=lambda kv: -kv[1]["inliers"]))


# ══════════════════════════════ 场景点云 ══════════════════════════════

def depth_median(frames):
    """N 帧深度逐像素取中值，只算测到的值；一半以上帧测不到的像素当空洞。"""
    D = np.stack(frames).astype(np.float32)
    D[D <= 0] = np.nan
    valid = np.isfinite(D).sum(0)
    with np.errstate(all="ignore"):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            med = np.nanmedian(D, axis=0)
    med[(valid < max(1, len(frames) // 2)) | ~np.isfinite(med)] = 0
    return med.astype(np.float32)


def dark_mask(color, thresh):
    """很暗的像素（黑色波纹管、线缆）。thresh=0 不去。膨胀 2 像素把边缘也带上。"""
    if color is None or thresh <= 0:
        return None
    import cv2
    v = color.max(axis=2)                        # HSV 的 V
    m = (v < thresh).astype(np.uint8)
    return cv2.dilate(m, np.ones((5, 5), np.uint8)) > 0


def scene_cloud(depth, intr, step=2, zmin=0.25, zmax=4.5, drop=None):
    ys, xs = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
    z = depth[ys, xs].reshape(-1)
    ok = (z > zmin) & (z < zmax)
    if drop is not None:
        ok &= ~drop[ys, xs].reshape(-1)
    px = np.stack([xs.reshape(-1)[ok], ys.reshape(-1)[ok]], 1).astype(float)
    return intr.deproject_many(px, z[ok])


def crop_scene(P_cam, T0, model_P):
    """相机点云 → base0 系，去地面（高于地面 5 cm），只留模型外接框外扩 0.5 m 内的点。"""
    S = xform(np.linalg.inv(T0), P_cam)
    lo, hi = model_P.min(0) - 0.5, model_P.max(0) + 0.5
    keep = (S[:, 2] > FLOOR_CLEAR) & np.all((S >= lo) & (S <= hi), axis=1)
    return S[keep]


# ══════════════════════════════ 画图 ══════════════════════════════

def silhouette(model, T_cam_base, shape):
    import cv2
    Pm, _, _ = model.visible(T_cam_base)
    Pc = xform(T_cam_base, Pm)
    z = Pc[:, 2]
    u = np.round(Pc[:, 0] / z * model.intr.fx + model.intr.ppx).astype(int)
    v = np.round(Pc[:, 1] / z * model.intr.fy + model.intr.ppy).astype(int)
    ok = (u >= 0) & (u < shape[1]) & (v >= 0) & (v < shape[0])
    m = np.zeros(shape[:2], np.uint8)
    m[v[ok], u[ok]] = 255
    m = cv2.morphologyEx(cv2.dilate(m, np.ones((3, 3), np.uint8)), cv2.MORPH_CLOSE,
                         np.ones((7, 7), np.uint8))
    return m


def draw_compare(color, model, T_before, T_after, out_dir, text_after=""):
    """配准前（红）/后（绿）模型轮廓投到彩色图上，另出一张左右对照图。"""
    import cv2
    os.makedirs(out_dir, exist_ok=True)
    paths = []
    imgs = []
    for tag, T, col in (("before", T_before, (40, 40, 240)), ("after", T_after, (60, 220, 60))):
        img = color.copy()
        m = silhouette(model, T, img.shape)
        over = img.copy()
        over[m > 0] = (0.55 * over[m > 0] + 0.45 * np.array(col)).astype(np.uint8)
        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(over, cs, -1, col, 2)
        label = "before: design + floor" if tag == "before" else f"after: calibrated {text_after}"
        cv2.putText(over, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 3)
        cv2.putText(over, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 1)
        p = os.path.join(out_dir, f"{tag}.jpg")
        cv2.imwrite(p, over)
        paths.append(p)
        imgs.append(over)
    both = color.copy()
    for T, col in ((T_before, (40, 40, 240)), (T_after, (60, 220, 60))):
        cs, _ = cv2.findContours(silhouette(model, T, both.shape), cv2.RETR_EXTERNAL,
                                 cv2.CHAIN_APPROX_NONE)
        cv2.drawContours(both, cs, -1, col, 2)
    cv2.putText(both, "red=before  green=after", (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 2)
    p = os.path.join(out_dir, "compare_overlay.jpg")
    cv2.imwrite(p, both)
    paths.append(p)
    p = os.path.join(out_dir, "compare_side_by_side.jpg")
    cv2.imwrite(p, np.hstack(imgs))
    paths.append(p)
    return paths


def draw_residual(color, depth, intr, model, T_cam_base, out_dir):
    """每个深度像素到模型表面的距离上色：绿 <1 cm，黄 1~3 cm，红 3~10 cm；更远不画。"""
    import cv2
    P = scene_cloud(depth, intr, step=1)
    Pm, _, _ = model.visible(T_cam_base)
    d, _ = NN(xform(T_cam_base, Pm)).query(P)
    img = (color * 0.5).astype(np.uint8)
    u = np.round(P[:, 0] / P[:, 2] * intr.fx + intr.ppx).astype(int)
    v = np.round(P[:, 1] / P[:, 2] * intr.fy + intr.ppy).astype(int)
    ok = (u >= 0) & (u < img.shape[1]) & (v >= 0) & (v < img.shape[0]) & (d < 0.10)
    col = np.zeros((ok.sum(), 3), np.uint8)
    dd = d[ok]
    col[dd < 0.01] = (60, 220, 60)
    col[(dd >= 0.01) & (dd < 0.03)] = (40, 220, 240)
    col[dd >= 0.03] = (40, 40, 240)
    img[v[ok], u[ok]] = col
    cv2.putText(img, "depth-to-model: green<1cm yellow<3cm red<10cm", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    p = os.path.join(out_dir, "residual_after.jpg")
    cv2.imwrite(p, img)
    return p


# ══════════════════════════════ 采集 ══════════════════════════════

STILL_DEG = 0.1        # 关节角变化小于它算静止
STILL_S = 6.0          # 至少静止这么久（控制器每 2 秒推一条，≈3 条）


def read_joints(ips, wait_s=20.0, still_wait_s=120.0):
    """只读两臂关节角，并等臂静止。

    要求：两臂都有新鲜数据（绝不用全零兜底）；不是全 0 占位值；且最近至少
    STILL_S 秒、≥3 条推送内每个关节变化 < STILL_DEG。返回 (live, q)。
    """
    from .jaka_live import JakaLive
    from .jaka_model import ARM_JOINTS
    live = JakaLive({"left": ips[0], "right": ips[1]}).start()
    need = set(ARM_JOINTS["left"] + ARM_JOINTS["right"])
    t0 = time.time()
    while time.time() - t0 < wait_s:
        q = live.q()
        if need <= set(q):
            break
        time.sleep(0.2)
    else:
        st = live.status()
        live.stop()
        raise SystemExit(f"✗ {wait_s:.0f} 秒内没拿到两臂新鲜关节角，不标定（不拿全零凑数）："
                         f"{st}。控制柜开机、网线接好后再跑")
    for arm in ("left", "right"):
        if all(abs(q[k]) < math.radians(0.01) for k in ARM_JOINTS[arm]):
            live.stop()
            raise SystemExit(
                f"✗ {arm} 臂 6 个关节角全是 0.000°——控制柜在推占位值（11-16 实测"
                f"本体没上电时 powered_on=0、关节角全 0），不能当真实姿态用。"
                f"本体上电后再跑；或用 --static-only 只拿不随关节动的部分标定")
    # ── 等静止：记录每条推送，最近 STILL_S 秒内（每臂 ≥3 条）都和最新一条差 < STILL_DEG ──
    hist = {arm: [] for arm in live.readers}
    seen = {arm: -1 for arm in live.readers}
    t1 = time.time()
    print(f"等机械臂静止（连续 ≥{STILL_S:.0f} 秒、≥3 条推送内每个关节变化 <{STILL_DEG}°）…")
    while time.time() - t1 < still_wait_s:
        for arm, r in live.readers.items():
            if r.n != seen[arm] and r.deg is not None:
                seen[arm] = r.n
                hist[arm].append((r.t, list(r.deg)))
        ok = True
        for arm, h in hist.items():
            if len(h) < 3 or h[-1][0] - h[0][0] < STILL_S:
                ok = False
                continue
            last = np.asarray(h[-1][1])
            # 从最新往回找，最长的一段「和最新差 < 门槛」的推送
            k = len(h) - 1
            while k > 0 and np.abs(np.asarray(h[k - 1][1]) - last).max() < STILL_DEG:
                k -= 1
            if len(h) - k < 3 or h[-1][0] - h[k][0] < STILL_S:
                ok = False
        if ok and need <= set(live.q()):
            print("  臂已静止：" + live.status())
            return live, live.q()
        time.sleep(0.2)
    st = live.status()
    live.stop()
    raise SystemExit(f"✗ {still_wait_s:.0f} 秒内机械臂没静止下来，不标定：{st}")


def joints_changed(live, q_ref, need):
    """采集中核关节角：断了或任一关节变化 ≥ STILL_DEG 就返回原因，否则 None。"""
    q = live.q()
    if not set(need) <= set(q):
        return "关节角断了（数据不新鲜）"
    d = max(abs(math.degrees(q[k] - q_ref[k])) for k in need)
    return f"关节角变了 {d:.2f}°（臂在动）" if d >= STILL_DEG else None


def capture(a, out_dir):
    """开相机取 N 帧 + 读关节角（前后各读一次，臂在动就报错）。存盘以便离线重算。"""
    import cv2
    from .camera_backend import open_camera
    ips = [s.strip() for s in (a.ips.split(",") + [""])[:2]]
    if a.static_only:
        live, q_before = None, {}
        print("--static-only：不读关节角，只用底盘/塔架/控制柜/臂安装座这些不随关节动的部分")
    else:
        live, q_before = read_joints(ips, a.joint_wait)
        print("关节角（度）：" + live.status())
    from .jaka_model import ARM_JOINTS
    need = ARM_JOINTS["left"] + ARM_JOINTS["right"]
    cam = open_camera(None, a.width, a.height, 30)
    try:
        for attempt in range(1, 4):
            frames, color, why = [], None, None
            for i, fr in enumerate(cam.frames()):
                if i < a.warmup:
                    continue                   # 刚开流的几帧曝光/深度不稳
                if live is not None:           # 每帧都核：推送一到就会反映在 q() 里
                    why = joints_changed(live, q_before, need)
                    if why:
                        break
                frames.append(fr.depth.copy())
                color = fr.color.copy()
                if len(frames) >= a.frames:
                    break
            if live is not None and not why:
                # 采完再等一条新推送核一次，确认采集结束时臂也没动
                n0 = {arm: r.n for arm, r in live.readers.items()}
                t_w = time.time()
                while time.time() - t_w < 5 and any(r.n == n0[arm] for arm, r in live.readers.items()):
                    time.sleep(0.1)
                why = joints_changed(live, q_before, need)
            if not why:
                break
            print(f"  第 {attempt} 次采集作废：{why}，等臂再次静止后重采")
            live.stop()
            live, q_before = read_joints(ips, a.joint_wait)
        else:
            raise SystemExit("✗ 连续 3 次采集期间臂都在动，不标定")
        intr = cam.intr
    finally:
        cam.close()
    if live is None:
        q, dq = {}, 0.0
    else:
        q_after = live.q()
        live.stop()
        dq = max(abs(q_after[k] - q_before[k]) for k in need)
        q = {k: float(q_before[k]) for k in need}
    os.makedirs(out_dir, exist_ok=True)
    np.savez_compressed(os.path.join(out_dir, "frames_mm.npz"),
                        depth=np.clip(np.stack(frames) * 1000, 0, 65535).astype(np.uint16))
    cv2.imwrite(os.path.join(out_dir, "color.jpg"), color, [cv2.IMWRITE_JPEG_QUALITY, 95])
    with open(os.path.join(out_dir, "capture.json"), "w") as f:
        json.dump({"time": time.strftime("%Y-%m-%d %H:%M:%S"),
                   "intr": {"fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy,
                            "width": intr.width, "height": intr.height,
                            "coeffs": list(intr.coeffs)},
                   "q_rad": q, "q_deg": {k: round(math.degrees(v), 3) for k, v in q.items()},
                   "joint_change_during_capture_deg": round(math.degrees(dq), 4),
                   "static_only": bool(a.static_only),
                   "n_frames": len(frames)}, f, ensure_ascii=False, indent=1)
    print(f"采了 {len(frames)} 帧，存到 {out_dir}")
    return load_capture(out_dir)


def load_capture(d):
    import cv2
    from .camera_backend import Intrinsics
    meta = json.load(open(os.path.join(d, "capture.json")))
    frames = np.load(os.path.join(d, "frames_mm.npz"))["depth"].astype(np.float32) / 1000.0
    color = cv2.imread(os.path.join(d, "color.jpg"))
    i = meta["intr"]
    intr = Intrinsics(i["fx"], i["fy"], i["ppx"], i["ppy"], int(i["width"]),
                      int(i["height"]), tuple(i.get("coeffs", (0,) * 5)))
    return {"frames": list(frames), "color": color, "intr": intr, "q": meta["q_rad"],
            "meta": meta}


# ══════════════════════════════ 主流程 ══════════════════════════════

def calibrate(cap, plane, T0, a, out_dir, log=print, dark=0, perturb=None):
    from .jaka_model import JakaModel
    intr, q = cap["intr"], cap["q"]
    robot = JakaModel()
    exclude = tuple(x for x in a.exclude.split(",") if x)
    if a.static_only:
        exclude += STATIC_EXCLUDE
    only = tuple(x for x in a.only.split(",") if x)
    P, N, L = model_cloud(robot, q, spacing=a.spacing, exclude=exclude, only=only)
    if only:
        log(f"只用这些连杆配准：{only}")
    model = Model(P, N, L, intr)
    vis0 = model.visible(T0)[2]
    if a.static_only and vis0.sum() < 300:      # 有粗搜，初值下看不见也能找；静态模式例外
        raise SystemExit(f"✗ 按初值，模型在画面里只看得见 {vis0.sum()} 个点（参与的部件："
                         f"{', '.join(sorted(set(L)))[:200]}），没法配准。"
                         + ("相机装在机器人上、底盘塔架在镜头后面时，--static-only 不可用，"
                            "得等机械臂本体上电拿到真实关节角" if a.static_only else ""))
    log(f"模型采样 {len(P)} 点（间距 {a.spacing*1000:.0f} mm，去掉 {exclude}），"
        f"初值下可见 {vis0.sum()} 点；可见部件："
        + ", ".join(sorted(set(L[vis0]) - {""}))[:400])

    depth = depth_median(cap["frames"])
    drop = dark_mask(cap["color"], dark)
    S = crop_scene(scene_cloud(depth, intr, step=a.step, drop=drop), T0, P)
    log(f"场景点 {len(S)}（中值深度、抽稀 {a.step}、去地面、模型框外扩 0.5 m"
        + (f"、去掉很暗像素 V<{dark}（占画面 {drop.mean()*100:.1f}%）" if drop is not None else "")
        + "）")

    before = evaluate(S, model, T0, np.eye(4))
    log(f"配准前：内点比例 {before['inlier_ratio']:.2f}，残差中位 {before['res_med_mm']} mm，"
        f"覆盖率 {before['model_coverage']:.2f}")

    if a.init_planar:
        th_i, tx_i, ty_i = [float(v) for v in a.init_planar.split(",")]
        log(f"初值：给定 θ{th_i:+.2f}°、({tx_i*100:+.1f}, {ty_i*100:+.1f}) cm（不粗搜）")
        starts = [(0, np.linalg.inv(planar_T(math.radians(th_i), tx_i, ty_i)))]
    else:
        log("水平面内粗搜初值（θ ±40°、tx ±0.8 m、ty ±0.5 m）：")
        starts = coarse_search(S, model, log=log)
    cands = []
    for score, Gs in starts:          # 每个候选都细配一遍，按内点数挑
        try:
            Gc, hc = icp(S, model, T0, 3, G_init=Gs)
            ev = evaluate(S, model, T0, Gc)
            cands.append((ev["n_inlier"], Gc, hc, Gs, ev, score))
        except Exception as e:
            log(f"    候选失败：{e}")
    if not cands:
        raise SystemExit("✗ 粗搜的候选一个都配不上")
    cands.sort(key=lambda c: -c[0])
    cand_summary = []
    for n_in, Gc, _, _, ev, score in cands:
        t_, x_, y_ = planar_params(Gc)
        cand_summary.append({"coarse_score": score, "n_inlier": n_in,
                             "inlier_ratio": ev["inlier_ratio"], "res_med_mm": ev["res_med_mm"],
                             "model_coverage": ev["model_coverage"],
                             "theta_deg": round(math.degrees(t_), 3),
                             "tx_m": round(x_, 4), "ty_m": round(y_, 4)})
    log("  各候选细配后：" + "；".join(
        f"θ{c['theta_deg']:+.1f}° ({c['tx_m']*100:+.1f},{c['ty_m']*100:+.1f})cm 内点{c['n_inlier']}"
        f" 覆盖{c['model_coverage']:.2f}" for c in cand_summary))
    G_start = cands[0][3]
    log("3 自由度配准（地面约束，从最好的候选起）：")
    G3, hist3 = icp(S, model, T0, 3, G_init=G_start, log=log)
    dof_main = 3
    three_dof = None
    G3_free = None
    if a.dof1:
        th3_, tx3_, ty3_ = planar_params(G3)
        ev3 = evaluate(S, model, T0, G3)
        three_dof = {"theta_deg": round(math.degrees(th3_), 4), "tx_m": round(tx3_, 5),
                     "ty_m": round(ty3_, 5), "metrics": ev3,
                     "T_cam_base": (T0 @ np.linalg.inv(G3)).round(6).tolist()}
        log(f"  3 自由度解（对照）：θ {math.degrees(th3_):+.2f}°、({tx3_*100:+.1f}, "
            f"{ty3_*100:+.1f}) cm，残差中位 {ev3['res_med_mm']} mm、内点 {ev3['n_inlier']}")
        G3_free = G3                          # 6 自由度对照从 3 自由度解出发
        G_start = planar_T(0.0, float(G_start[0, 3]), 0.0)
        log("1 自由度配准（θ=0、左右=0，只沿机器人前后平移）：")
        G3, hist3 = icp(S, model, T0, 1, G_init=G_start, log=log)
        dof_main = 1
    th, tx, ty = planar_params(G3)
    T3 = T0 @ np.linalg.inv(G3)
    after = evaluate(S, model, T0, G3)
    per_link = link_stats(S, model, T0, G3)
    log("  按部位（可见点 / 被认领 / 贡献内点占比）：" + "；".join(
        f"{g} {v['visible']}/{v['claimed']}/{v['inlier_share']*100:.0f}%"
        for g, v in per_link.items()))
    log(f"  → 转 {math.degrees(th):+.2f}°，平移 ({tx*100:+.1f}, {ty*100:+.1f}) cm；"
        f"内点比例 {after['inlier_ratio']:.2f}，残差中位 {after['res_med_mm']} mm、"
        f"90 分位 {after['res_p90_mm']} mm，覆盖率 {after['model_coverage']:.2f}")

    # ── 6 自由度对照：同一初值、不加地面约束 ──
    log("6 自由度对照（从 3 自由度解出发，放开倾斜和高度）：")
    try:
        G_ref = G3 if G3_free is None else G3_free
        G6, hist6 = icp(S, model, T0, 6, G_init=G_ref, radii=(0.05, 0.03, 0.02), log=log)
        T6 = T0 @ np.linalg.inv(G6)
        D = np.linalg.inv(T0 @ np.linalg.inv(G_ref)) @ T6   # 和 3 自由度解差多少
        up6 = np.linalg.inv(G6)[:3, :3] @ [0, 0, 1.0]       # 6 自由度解的「上」在 base0 里
        tilt = math.degrees(math.acos(float(np.clip(up6[2], -1, 1))))
        D6 = np.linalg.inv(G6)
        cmp6 = {
            "rot_diff_deg": round(rot_angle_deg(D[:3, :3]), 3),
            "trans_diff_cm": round(float(np.linalg.norm(D[:3, 3])) * 100, 2),
            "trans_diff_xyz_cm": [round(float(v) * 100, 2) for v in D[:3, 3]],
            "tilt_vs_floor_deg": round(tilt, 3),
            "height_vs_floor_cm": round(float(D6[2, 3]) * 100, 2),
            "theta_deg": round(math.degrees(math.atan2(D6[1, 0], D6[0, 0])), 3),
            "metrics": evaluate(S, model, T0, G6),
            "T_cam_base": T6.round(6).tolist(),
        }
        log(f"  → 与 3 自由度差 {cmp6['rot_diff_deg']:.2f}°、{cmp6['trans_diff_cm']:.1f} cm；"
            f"6 自由度解相对地面倾斜 {tilt:.2f}°、离地 {cmp6['height_vs_floor_cm']:+.1f} cm")
    except Exception as e:
        cmp6 = {"error": str(e)}
        log(f"  → 6 自由度没配上：{e}")

    # ── 收敛性：随机扰动初值 ──
    rng = np.random.default_rng(1)
    runs = []
    n_pert = a.perturb if perturb is None else perturb
    log(f"在粗搜起点上随机扰动 {n_pert} 次（±10°、±10 cm）：")
    for k in range(n_pert):
        d_th = math.radians(rng.uniform(-10, 10))
        d_t = rng.uniform(-0.10, 0.10, 2)
        Gi = (planar_T(0, d_t[0], 0) if dof_main == 1 else planar_T(d_th, *d_t)) @ G_start
        try:
            Gk, _ = icp(S, model, T0, dof_main, G_init=Gi)
            thk, txk, tyk = planar_params(Gk)
            runs.append([math.degrees(thk), txk, tyk])
        except Exception as e:
            runs.append([np.nan, np.nan, np.nan])
            log(f"    第 {k+1} 次失败：{e}")
    R = np.asarray(runs, float).reshape(-1, 3)
    ref = np.array([math.degrees(th), tx, ty])
    good = np.isfinite(R[:, 0]) & (np.abs(R[:, 0] - ref[0]) < 0.5) & \
        (np.hypot(R[:, 1] - ref[1], R[:, 2] - ref[2]) < 0.01)
    conv = {
        "n": int(len(R)), "n_same_solution": int(good.sum()),
        "rule": "与主解差 <0.5° 且 <1 cm 算收敛到同一解",
        "theta_spread_deg": round(float(np.nanmax(np.abs(R[good, 0] - ref[0]))), 4) if good.any() else None,
        "trans_spread_cm": round(float(np.nanmax(np.hypot(R[good, 1] - ref[1],
                                                         R[good, 2] - ref[2]))) * 100, 3)
        if good.any() else None,
        "runs_deg_m": np.round(R, 5).tolist(),
    }
    log(f"  → {conv['n_same_solution']}/{conv['n']} 次收敛到同一解，"
        f"离散度 θ {conv['theta_spread_deg']}°、平移 {conv['trans_spread_cm']} cm")

    # ── 多帧一致性：帧分 3 组各自中值、各自配准 ──
    groups = []
    fr = cap["frames"]
    if len(fr) >= 9:
        k = len(fr) // 3
        for g in range(3):
            Sg = crop_scene(scene_cloud(depth_median(fr[g * k:(g + 1) * k]), intr,
                                        step=a.step, drop=drop), T0, P)
            try:
                Gg, _ = icp(S=Sg, model=model, T0=T0, dof=dof_main, G_init=G_start)
                thg, txg, tyg = planar_params(Gg)
                groups.append([math.degrees(thg), txg, tyg])
            except Exception as e:
                groups.append([np.nan, np.nan, np.nan])
        Gr = np.asarray(groups, float)
        frames_spread = {
            "groups_deg_m": np.round(Gr, 5).tolist(),
            "theta_range_deg": round(float(np.nanmax(Gr[:, 0]) - np.nanmin(Gr[:, 0])), 4),
            "trans_range_cm": round(float(max(np.nanmax(Gr[:, 1]) - np.nanmin(Gr[:, 1]),
                                               np.nanmax(Gr[:, 2]) - np.nanmin(Gr[:, 2]))) * 100, 3),
        }
        log(f"  分 3 组帧各自配准：θ 极差 {frames_spread['theta_range_deg']}°，"
            f"平移极差 {frames_spread['trans_range_cm']} cm")
    else:
        frames_spread = {"note": "帧太少，没做分组"}

    # ── 可信度门槛 ──
    why = []
    if after["n_inlier"] < 2000:
        why.append(f"内点太少（{after['n_inlier']}）")
    if after["res_med_mm"] is None or after["res_med_mm"] > 12:
        why.append(f"残差中位太大（{after['res_med_mm']} mm）")
    if after["inlier_ratio"] < before["inlier_ratio"]:
        why.append("配准后内点比例反而下降")
    if conv["n"] and conv["n_same_solution"] < 0.6 * conv["n"]:
        why.append(f"扰动初值只有 {conv['n_same_solution']}/{conv['n']} 次收敛到同一解")
    if abs(math.degrees(th)) > 40 or math.hypot(tx, ty) > 0.8:
        why.append("校正量超出搜索范围（>40° 或 >80 cm），多半配错了地方")
    for c in cand_summary[1:]:
        far = abs(c["theta_deg"] - math.degrees(th)) > 3 or \
            math.hypot(c["tx_m"] - tx, c["ty_m"] - ty) > 0.05
        if far and c["n_inlier"] > 0.85 * after["n_inlier"]:
            why.append(f"有另一个差不多好的解（θ{c['theta_deg']:+.1f}°、"
                       f"({c['tx_m']*100:+.0f},{c['ty_m']*100:+.0f}) cm，内点 {c['n_inlier']} vs "
                       f"{after['n_inlier']}），解不唯一")
            break
    if isinstance(cmp6, dict) and cmp6.get("tilt_vs_floor_deg") is not None and \
            (cmp6["tilt_vs_floor_deg"] > 3 or abs(cmp6["height_vs_floor_cm"]) > 5):
        why.append(f"放开地面约束后倾斜 {cmp6['tilt_vs_floor_deg']:.1f}°、离地 "
                   f"{cmp6['height_vs_floor_cm']:+.1f} cm，地面约束和点云不太一致")

    T_before = T0
    paths = []
    try:
        paths = draw_compare(cap["color"], model, T_before, T3, out_dir,
                             f"{math.degrees(th):+.1f}deg ({tx*100:+.1f},{ty*100:+.1f})cm")
        paths.append(draw_residual(cap["color"], depth, intr, model, T3, out_dir))
    except Exception as e:
        log(f"（画图失败：{e}）")

    return {
        "schema": "jaka_extrinsic_calib.v1",
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "ok": not why, "not_ok_reasons": why,
        "method": "模型对点云配准 + 地面约束（"
                  + ("1 自由度：只求前后平移 tx，θ=0、ty=0" if dof_main == 1 else "3 自由度：θ、tx、ty")
                  + "），点到面 ICP + Huber"
                  f"（细档截断），裁剪半径 {list(RADII)} m"
                  + ("；只用不随关节动的部分（--static-only）" if a.static_only else ""),
        "static_only": bool(a.static_only),
        "convention": "T_cam_base: 基座系→相机光学系；T_cam_base = T_init @ [Rz(theta)|(tx,ty,0)]，"
                      "theta/tx/ty 在初值基座系（地面为 z=0）里",
        "T_cam_base": T3.round(6).tolist(),
        "T_base_cam": np.linalg.inv(T3).round(6).tolist(),
        "correction": {"theta_deg": round(math.degrees(th), 4), "tx_m": round(tx, 5),
                       "ty_m": round(ty, 5)},
        "T_init": T0.round(6).tolist(),
        "init_note": "设计值（sensors.json）取逆 + align_to_floor",
        "plane": plane,
        "camera_id": a.camera_id,
        "q_rad": cap["q"],
        "q_deg": {k: round(math.degrees(v), 3) for k, v in cap["q"].items()},
        "model": {"n_points": int(len(P)), "spacing_m": a.spacing, "exclude": list(exclude),
                  "only": list(only)},
        "n_scene_points": int(len(S)),
        "metrics_before": before, "metrics_after": after, "per_link": per_link,
        "dark_filter_V": dark,
        "icp_stages": hist3, "coarse_candidates": cand_summary,
        "dof": dof_main, "three_dof_for_comparison": three_dof,
        "convergence": conv, "frame_groups": frames_spread,
        "six_dof": cmp6,
        "figures": paths,
    }


def run_variants(cap, plane, T0, a, out_dir):
    """--dark>0 时：主跑去掉很暗像素（黑色波纹管），再跑一遍不去的作对照。"""
    if a.dark <= 0 or a.no_compare:
        return calibrate(cap, plane, T0, a, out_dir, dark=max(0, a.dark))
    print(f"\n════ 主跑：去掉很暗像素（V<{a.dark}，黑色波纹管/线缆）════")
    res = calibrate(cap, plane, T0, a, os.path.join(out_dir, "dark_removed"), dark=a.dark)
    print("\n════ 对照：不去暗像素 ════")
    alt = calibrate(cap, plane, T0, a, os.path.join(out_dir, "all_pixels"), dark=0,
                    perturb=min(a.perturb, 6))
    ca, cb = res["correction"], alt["correction"]
    res["variant_all_pixels"] = {
        "correction": cb, "metrics_after": alt["metrics_after"], "ok": alt["ok"],
        "not_ok_reasons": alt["not_ok_reasons"], "convergence": alt["convergence"],
        "per_link": alt["per_link"], "six_dof": {k: v for k, v in alt["six_dof"].items()
                                                if k != "T_cam_base"},
        "diff_theta_deg": round(cb["theta_deg"] - ca["theta_deg"], 4),
        "diff_trans_cm": round(math.hypot(cb["tx_m"] - ca["tx_m"],
                                          cb["ty_m"] - ca["ty_m"]) * 100, 3)}
    print(f"\n去不去暗像素：θ 差 {res['variant_all_pixels']['diff_theta_deg']:+.2f}°，"
          f"平移差 {res['variant_all_pixels']['diff_trans_cm']:.1f} cm")
    return res


def write_result(res, out_dir, a):
    os.makedirs(out_dir, exist_ok=True)
    raw = os.path.join(out_dir, "result.json")
    with open(raw, "w") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    print(f"完整结果：{raw}")
    if a.dry_run:
        print("--dry-run：不写 extrinsic_calib.json")
        return
    if not res["ok"] and not a.force:
        print("✗ 没过可信度门槛，不写 extrinsic_calib.json（检测程序继续用设计值+地面摆正）："
              + "；".join(res["not_ok_reasons"]) + "。确认要用加 --force")
        return
    if os.path.isfile(CALIB_FILE):                 # 上一次的标定先挪走，不直接覆盖
        bak = CALIB_FILE.replace(".json", time.strftime("_%Y%m%d_%H%M%S.bak.json"))
        shutil.copy2(CALIB_FILE, bak)
        print(f"旧标定备份到 {bak}")
    with open(CALIB_FILE, "w") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    print(f"✓ 写好 {CALIB_FILE}。重启检测程序后生效（EXTRINSIC=design 可强制回到设计值）")


# ══════════════════════════════ 合成数据自测 ══════════════════════════════

def look_at(eye, target, up=(0, 0, 1.0)):
    """相机光学系（+X 右 +Y 下 +Z 前）在基座系里的位姿 T_base_cam。"""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.stack([x, y, z], 1)
    T[:3, 3] = eye
    return T


def render_depth(P_cam, intr, splat=1):
    z = P_cam[:, 2]
    ok = z > 0.1
    u = np.round(P_cam[ok, 0] / z[ok] * intr.fx + intr.ppx).astype(int)
    v = np.round(P_cam[ok, 1] / z[ok] * intr.fy + intr.ppy).astype(int)
    z = z[ok]
    D = np.full((intr.height, intr.width), np.inf)
    for du in range(-splat, splat + 1):
        for dv in range(-splat, splat + 1):
            uu, vv = u + du, v + dv
            k = (uu >= 0) & (uu < intr.width) & (vv >= 0) & (vv < intr.height)
            np.minimum.at(D, (vv[k], uu[k]), z[k])
    return D


def synth_scene(robot, q, T_cam_base_true, intr, n_frames, rng, mismatch=True):
    """合成：模型渲染深度 + 地面 + 一个人 + 一张桌子 + 深度噪声和空洞。"""
    P, N, L = model_cloud(robot, q, spacing=0.002, exclude=DEFAULT_EXCLUDE, seed=7)
    if mismatch:     # 实物和模型不完全一样：夹爪换了形状、多一根线缆
        keep = np.array(["gripper" not in l for l in L])
        P = P[keep]
        cable = np.array([-0.15, 0.0, 1.3]) + np.outer(np.linspace(0, 1, 3000),
                                                     [0.25, 0.2, -0.6])
        cable += rng.normal(0, 0.01, cable.shape)
        P = np.vstack([P, cable])
    # 人：站在机器人前方 1 m 的圆柱；桌子：机器人旁边 0.7 m 高
    th = rng.uniform(0, 2 * np.pi, 40000)
    hz = rng.uniform(0, 1.7, 40000)
    person = np.stack([0.9 + 0.18 * np.cos(th), -0.6 + 0.18 * np.sin(th), hz], 1)
    tx, ty = rng.uniform(0.4, 1.0, 30000), rng.uniform(0.3, 0.9, 30000)
    table = np.stack([tx, ty, np.full_like(tx, 0.72)], 1)
    Pall = np.vstack([P, person, table])
    D = render_depth(xform(T_cam_base_true, Pall), intr)
    # 黑色波纹管：沿左臂 Link_02→Link_05 外侧 7 cm 绕一根 3.5 cm 粗的管子（模型里没有）
    Tl = robot.forward(q)
    ctr = np.vstack([np.linspace(Tl[f"left_Link_0{i}"][:3, 3], Tl[f"left_Link_0{i+1}"][:3, 3], 80)
                     for i in (2, 3, 4)]) + np.array([0.0, 0.07, 0.05])
    ang = rng.uniform(0, 2 * np.pi, (len(ctr), 60))
    hose = (ctr[:, None, :] + 0.035 * np.stack([np.cos(ang), np.sin(ang) * 0.7,
                                               np.sin(ang) * 0.7], -1)).reshape(-1, 3)
    hose = np.vstack([hose + rng.normal(0, 0.004, hose.shape) for _ in range(6)])
    Dh = render_depth(xform(T_cam_base_true, hose), intr)
    hose_px = Dh < D
    D = np.minimum(D, Dh)
    # 地面：每个像素的射线和 z=0 平面求交
    ys, xs = np.mgrid[0:intr.height, 0:intr.width]
    rays = np.stack([(xs - intr.ppx) / intr.fx, (ys - intr.ppy) / intr.fy,
                     np.ones_like(xs, float)], -1)
    Tb = np.linalg.inv(T_cam_base_true)
    rb = rays @ Tb[:3, :3].T
    s = -Tb[2, 3] / np.where(np.abs(rb[..., 2]) > 1e-9, rb[..., 2], 1e-9)
    floor_z = np.where(s > 0, s, np.inf)          # 射线参数 = 相机系 z
    D = np.minimum(D, floor_z)
    D[~np.isfinite(D)] = 0
    frames = []
    for _ in range(n_frames):
        sig = 0.0015 * (D / 1.5) ** 2 + 0.0005
        F = D + rng.normal(0, 1, D.shape) * sig
        F[rng.random(D.shape) < 0.05] = 0
        frames.append(F.astype(np.float32))
    import importlib.util
    if importlib.util.find_spec("cv2"):
        img = np.clip(255 - D / 4.0 * 200, 0, 255).astype(np.uint8)
        img = np.maximum(img, 70)
        img[hose_px] = 20                       # 管子是黑的
        color = np.repeat(img[..., None], 3, axis=2)
    else:
        color = None
    return frames, color


def selftest(a):
    from .camera_backend import Intrinsics
    from .jaka_model import JakaModel
    from .robot_frames import RobotFrames
    rng = np.random.default_rng(42)
    intr = Intrinsics(425.0, 425.0, 424.0, 240.0, 848, 480, (0, 0, 0, 0, 0))
    robot = JakaModel()
    q = {"left_joint_1": 0.3, "left_joint_2": 0.7, "left_joint_3": -1.2, "left_joint_4": 0.4,
         "left_joint_5": 1.0, "left_joint_6": 0.2,
         "right_joint_1": -0.4, "right_joint_2": -0.6, "right_joint_3": 1.1,
         "right_joint_4": -0.3, "right_joint_5": -0.9, "right_joint_6": 0.1}
    # 真值：相机在独立支架上，离地 1.68 m，在机器人斜前方往下看
    T_base_cam_true = look_at([1.6, -0.6, 1.68], [0.0, 0.0, 1.2])
    T_true = np.linalg.inv(T_base_cam_true)
    # 实测地面（相机系），故意加 0.3° 倾斜误差，看 6 自由度对照能不能看出来
    n_true = T_true[:3, :3] @ [0, 0, 1.0]
    tilt = rotvec_R(np.array([math.radians(0.3), 0, 0]))
    plane = {"origin": (T_true[:3, 3]).tolist(),
             "normal": (tilt @ n_true).tolist(), "file_mtime": "selftest"}
    # 初值：真值上叠一个水平误差（转 7°、平移 (9, -6) cm），再按（略歪的）地面摆正
    from .robot_frames import align_to_floor
    D_err = planar_T(math.radians(7.0), 0.09, -0.06)
    T0, _, _ = align_to_floor(T_true @ np.linalg.inv(D_err), plane["origin"], plane["normal"])
    frames, color = synth_scene(robot, q, T_true, intr, a.frames, rng)
    cap = {"frames": frames, "color": color, "intr": intr, "q": q}
    out_dir = a.out or os.path.join(OUT_DIR, "selftest")
    t = time.time()
    res = run_variants(cap, plane, T0, a, out_dir)
    G_true = np.linalg.inv(np.linalg.inv(T0) @ T_true)          # 真值对应的 G
    th_t, tx_t, ty_t = planar_params(G_true)
    c = res["correction"]
    e_th = abs(c["theta_deg"] - math.degrees(th_t))
    e_t = math.hypot(c["tx_m"] - tx_t, c["ty_m"] - ty_t)
    T3 = np.asarray(res["T_cam_base"])
    # 用一个和标定无关的量核：机器人上几个关键点在相机系里的位置误差
    Tl = robot.forward(q)
    keys = ["left_Link_06", "right_Link_06", "main_tower", "left_Link_03", "right_Link_03"]
    errs = [np.linalg.norm((T3 @ Tl[k])[:3, 3] - (T_true @ Tl[k])[:3, 3]) for k in keys]
    print("\n══ 自测结果 ══")
    print(f"真值校正 θ {math.degrees(th_t):+.3f}°、t ({tx_t*100:+.2f}, {ty_t*100:+.2f}) cm")
    print(f"标定校正 θ {c['theta_deg']:+.3f}°、t ({c['tx_m']*100:+.2f}, {c['ty_m']*100:+.2f}) cm")
    print(f"误差 θ {e_th:.3f}°、平移 {e_t*1000:.1f} mm；关键点最大误差 {max(errs)*1000:.1f} mm；"
          f"可信度门槛 {'过' if res['ok'] else '没过：' + '；'.join(res['not_ok_reasons'])}；"
          f"耗时 {time.time()-t:.0f} s")
    res["selftest"] = {"truth_theta_deg": math.degrees(th_t), "truth_tx_m": tx_t,
                       "truth_ty_m": ty_t, "err_theta_deg": e_th, "err_trans_mm": e_t * 1000,
                       "keypoint_err_mm": [round(x * 1000, 2) for x in errs],
                       "plane_tilt_injected_deg": 0.3, "nn_backend": NN(np.random.rand(10, 3)).kind}
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "result.json"), "w") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    ok = e_th < 0.3 and e_t < 0.008 and res["ok"]
    print("✓ 自测通过" if ok else "✗ 自测没过")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--frames", type=int, default=30, help="取几帧做深度时域中值")
    ap.add_argument("--warmup", type=int, default=15, help="开流后先丢几帧")
    ap.add_argument("--width", type=int, default=848)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--ips", default="192.168.71.37,192.168.71.36", help="左臂IP,右臂IP（只读 10000 端口）")
    ap.add_argument("--joint-wait", type=float, default=20.0, help="等两臂新鲜关节角最多几秒")
    ap.add_argument("--camera-id", default="top", help="sensors.json 里的相机 id（初值用）")
    ap.add_argument("--exclude", default=",".join(DEFAULT_EXCLUDE),
                    help="模型里不参与配准的连杆（名字含这些字串就去掉），逗号分隔")
    ap.add_argument("--spacing", type=float, default=0.006, help="模型表面采样间距（米）")
    ap.add_argument("--step", type=int, default=2, help="深度图抽稀步长")
    ap.add_argument("--perturb", type=int, default=12, help="随机扰动初值试几次")
    ap.add_argument("--dark", type=int, default=50,
                    help="彩色图亮度（HSV 的 V）低于它的像素不参与配准（黑色波纹管）；"
                         "0=不去。>0 时另跑一遍不去的作对照")
    ap.add_argument("--from", dest="src", default="", help="用存下的采集目录离线重算")
    ap.add_argument("--out", default="", help="输出目录（默认 out/calib/<时间戳>）")
    ap.add_argument("--no-compare", action="store_true", help="不跑「不去暗像素」的对照")
    ap.add_argument("--dof1", action="store_true",
                    help="只求沿机器人前后方向的平移（θ、左右固定 0）；3 自由度解并列报出作对照")
    ap.add_argument("--only", default="",
                    help="只用名字含这些字串的连杆配准，逗号分隔。例：只用两臂圆柱段 "
                         "_Link_01,_Link_02,_Link_03,_Link_04,_Link_05")
    ap.add_argument("--init-planar", default="", metavar="θ度,tx米,ty米",
                    help="直接给初值（机器人在初值基座系里的 θ/tx/ty），不粗搜")
    ap.add_argument("--dry-run", action="store_true", help="只算，不写 extrinsic_calib.json")
    ap.add_argument("--force", action="store_true", help="没过可信度门槛也写")
    ap.add_argument("--selftest", action="store_true", help="合成数据自测（不碰相机和控制器）")
    ap.add_argument("--static-only", action="store_true",
                    help="不读关节角，只用底盘/塔架/控制柜/臂安装座配准（臂没上电、关节角不可用时）")
    ap.add_argument("--wait-still", action="store_true",
                    help="只等机械臂静止（不开相机）就退出：停检测程序前先跑它")
    ap.add_argument("--capture-only", action="store_true",
                    help="只采集存盘就退出（尽快把相机还给检测程序），之后用 --from 离线算")
    a = ap.parse_args()
    sys.stdout.reconfigure(line_buffering=True)

    if a.selftest:
        sys.exit(0 if selftest(a) else 1)
    if a.wait_still:          # 只等臂静止就退出，不碰相机（停检测前先用它确认）
        ips = [x.strip() for x in (a.ips.split(",") + [""])[:2]]
        live, _ = read_joints(ips, a.joint_wait)
        live.stop()
        return

    plane = load_plane()                     # 现读，不缓存
    T0, lift, deg = initial_T_cam_base(plane, a.camera_id)
    print(f"地面（{ZONE_CFG}，改于 {plane['file_mtime']}）；初值 = 设计值 + 地面摆正"
          f"（落了 {lift:+.2f} m、转了 {deg:.1f}°）")
    out_dir = a.out or os.path.join(OUT_DIR, time.strftime("%Y%m%d_%H%M%S"))
    if a.src:
        cap = load_capture(a.src)
        print(f"用存下的采集 {a.src}（{cap['meta']['time']}，{len(cap['frames'])} 帧）")
    else:
        cap = capture(a, out_dir)
        if a.capture_only:
            print(f"--capture-only：已存盘。离线算：calib_extrinsic.py --from {out_dir}"
                  + (" --static-only" if a.static_only else ""))
            return
    if a.src and cap["meta"].get("static_only") and not a.static_only:
        raise SystemExit("这份采集没有关节角（--static-only 采的），重算也要加 --static-only")
    if not a.static_only and not cap["q"]:
        raise SystemExit("采集里没有关节角，不能按臂姿态配准；加 --static-only")
    res = run_variants(cap, plane, T0, a, out_dir)
    res["capture_dir"] = a.src or out_dir
    write_result(res, out_dir, a)
    c = res["correction"]
    print(f"\n结论：在「设计值+地面摆正」上绕竖直轴转 {c['theta_deg']:+.2f}°、"
          f"平移 ({c['tx_m']*100:+.1f}, {c['ty_m']*100:+.1f}) cm；"
          f"残差中位 {res['metrics_after']['res_med_mm']} mm；"
          f"{'可信' if res['ok'] else '不可信：' + '；'.join(res['not_ok_reasons'])}")


if __name__ == "__main__":
    main()
