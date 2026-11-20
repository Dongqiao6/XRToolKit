#!/usr/bin/env python3
"""桌面分区 + 手部试探 —— 看看占用判断能精细到什么程度。

不用人工标定：开机先从深度图里把**桌面那个平面**拟合出来，然后在平面上划网格。
相机换个角度、桌子挪一下，重跑一次就行。

手的高度分两级，这是「精细度」最直观的体现：
    接触   指尖离桌面 < 8 cm    —— 手真的压在这块区域上
    悬停   8 ~ 35 cm            —— 手在上方掠过，还没落下
    不算   > 35 cm              —— 只是从上面飞过，不该触发

判据用**每根指尖单独判**，不是整只手取平均：手是摊开的，用平均值会在两格
交界处来回跳，而且伸一根手指去点的时候平均值还在手掌那边。

区域有两种来法：
    自己框（推荐）  鼠标在画面上点出多边形，想框哪块框哪块，形状不限
    均分网格        整张桌面切成 rows×cols，快速试手用

实际工位上只有机械臂作业的那一块需要管，均分网格会把无关的桌面也算进去，
所以正式用一律自己框。

跑法：
    python3 table_zones.py --pick                 框区域（鼠标点，存成 zones.json）
    python3 table_zones.py --show                 实时窗口（有 zones.json 就用它）
    python3 table_zones.py --show --grid 3x3      强制用均分网格
    python3 table_zones.py --show --rezone        重新拟合桌面

窗口里的键：
    r   开始 / 停止录制（每按一次开始都新建一个目录，不会覆盖上一段）
    q   退出
录的是叠加之后的画面，和屏幕上看到的一模一样，可以直接拿去放。
录制期间左上角有红点和已录时长；**停止一定要按 r 或 q**，直接关窗口
文件尾部写不进去，视频会坏。

产出 out/table_zones/ 下的平面参数，下次直接复用。
"""
import argparse, json, os, time
from collections import deque

import cv2, mediapipe as mp, numpy as np
from camera_backend import open_camera
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import (HandLandmarker, HandLandmarkerOptions,
                                           RunningMode)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "table_zones")
PLANE_FILE = os.path.join(OUT, "plane.json")
W, H, FPS = 848, 480, 30
TOUCH_H, HOVER_H = 0.08, 0.35          # 接触 / 悬停 的高度门槛，米
TIPS = {4: "拇指", 8: "食指", 12: "中指", 16: "无名指", 20: "小指"}
HOLD_N = 3        # 连着几帧才认，压掉单帧跳变


def fit_plane_ransac(pts, iters=120, tol=0.015):
    """从一堆三维点里找出最大的那个平面（桌面）。

    不能直接对全部点做最小二乘：画面里还有墙、人、桌子腿，一起拟合会得到
    一个谁都不贴的斜面。RANSAC 的做法是反复随机取三点定面、数有多少点贴着它，
    贴得最多的那个就是桌面。
    """
    best_n, best_c, best_cnt = None, None, 0
    rng = np.random.default_rng(0)
    for _ in range(iters):
        idx = rng.choice(len(pts), 3, replace=False)
        a, b, c = pts[idx]
        n = np.cross(b - a, c - a)
        ln = np.linalg.norm(n)
        if ln < 1e-9:
            continue
        n /= ln
        cnt = int(np.sum(np.abs((pts - a) @ n) < tol))
        if cnt > best_cnt:
            best_n, best_c, best_cnt = n, a, cnt
    if best_n is None:
        return None, None, 0
    # 用内点重新做一次最小二乘，比三点定面稳得多
    inl = pts[np.abs((pts - best_c) @ best_n) < tol]
    c = inl.mean(axis=0)
    _, _, vt = np.linalg.svd(inl - c)
    n = vt[-1] / (np.linalg.norm(vt[-1]) + 1e-12)
    if n @ c > 0:        # 法向统一指向相机这边，高度才会是正的
        n = -n
    return c, n, len(inl)


class Table:
    """桌面平面 + 上面划的网格。"""

    def __init__(self, origin, normal, u, v, umin, umax, vmin, vmax, rows, cols):
        self.origin = np.asarray(origin, float)
        self.normal = np.asarray(normal, float)
        self.u, self.v = np.asarray(u, float), np.asarray(v, float)
        self.umin, self.umax, self.vmin, self.vmax = umin, umax, vmin, vmax
        self.rows, self.cols = rows, cols

    def height(self, p):
        """点离桌面多高，米。桌面之上为正。"""
        return float((np.asarray(p, float) - self.origin) @ self.normal)

    def uv(self, p):
        w = np.asarray(p, float) - self.origin
        return float(w @ self.u), float(w @ self.v)

    def cell(self, p):
        """投影到桌面落在哪个格子。返回 (行, 列) 或 None。"""
        uu, vv = self.uv(p)
        if not (self.umin <= uu <= self.umax and self.vmin <= vv <= self.vmax):
            return None
        c = int((uu - self.umin) / (self.umax - self.umin) * self.cols)
        r = int((vv - self.vmin) / (self.vmax - self.vmin) * self.rows)
        return (min(r, self.rows - 1), min(c, self.cols - 1))

    @staticmethod
    def _pt_in_poly(q, poly):
        """射线法：从 q 往 +u 打一条射线，数穿过几条边。奇数次就在里面。"""
        n, inside = len(poly), False
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            if (y1 > q[1]) != (y2 > q[1]):
                xx = x1 + (q[1] - y1) * (x2 - x1) / (y2 - y1 + 1e-12)
                if xx > q[0]:
                    inside = not inside
        return inside

    def zone_of(self, p, polys):
        """落在哪个区域里。polys 是 {名字: uv多边形}，先命中先算。

        均分网格也会被转成同样形状的 uv 多边形喂进来，所以下游只有一套代码，
        不用为两种区域各写一遍判定和绘制。
        """
        q = self.uv(p)
        for name, poly in polys.items():
            if self._pt_in_poly(q, poly):
                return name
        return None

    def grid_polys(self):
        """把均分网格也表达成 {名字: uv多边形}。"""
        du = (self.umax - self.umin) / self.cols
        dv = (self.vmax - self.vmin) / self.rows
        out = {}
        for r in range(self.rows):
            for c in range(self.cols):
                out[f"{r+1},{c+1}"] = [
                    (self.umin + (c+ddu)*du, self.vmin + (r+ddv)*dv)
                    for ddu, ddv in ((0,0), (1,0), (1,1), (0,1))]
        return out

    def uv_to_px(self, uu, vv, intr):
        """桌面上的一个二维点 → 图像像素。画区域边框用。"""
        p3 = self.origin + uu * self.u + vv * self.v
        if p3[2] <= 0.05:
            return None
        xy = intr.project(p3)
        return (int(xy[0]), int(xy[1])) if xy else None

    def cell_corners3d(self, r, c):
        du = (self.umax - self.umin) / self.cols
        dv = (self.vmax - self.vmin) / self.rows
        out = []
        for ddu, ddv in ((0,0), (1,0), (1,1), (0,1)):
            uu = self.umin + (c + ddu) * du
            vv = self.vmin + (r + ddv) * dv
            out.append(self.origin + uu * self.u + vv * self.v)
        return out

    def to_dict(self):
        return {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                for k, v in self.__dict__.items()}

    @classmethod
    def from_dict(cls, d):
        return cls(d["origin"], d["normal"], d["u"], d["v"],
                   d["umin"], d["umax"], d["vmin"], d["vmax"],
                   d["rows"], d["cols"])


def build_table(cam, rows, cols, n_frames=20):
    """采几帧深度，拟合桌面，在上面框出网格范围。"""
    print("正在找桌面……别把手放在桌上。", flush=True)
    intr = cam.intr
    acc = []
    for i, fr in enumerate(cam.frames()):
        acc.append(fr.depth)
        if len(acc) >= n_frames:
            break
    dm = np.median(np.stack(acc), axis=0)

    # 抽稀成点云。全分辨率 40 万点，RANSAC 跑不动也没必要。
    h_, w_ = dm.shape
    ys, xs = np.mgrid[0:h_:4, 0:w_:4]
    ds = dm[::4, ::4]
    m = (ds > 0.2) & (ds < 2.5)         # 桌面就在手边，远处的墙不要
    if m.sum() < 500:
        return None, "深度点太少，相机可能没对着桌子，或者离得太近"
    pts = intr.deproject_many(
        np.stack([xs[m].astype(float), ys[m].astype(float)], axis=1), ds[m])

    c, n, n_in = fit_plane_ransac(pts)
    if c is None or n_in < 300:
        return None, f"没找到够大的平面（内点只有 {n_in}）"

    # 在平面上建二维轴
    a = np.array([0.0, 1.0, 0.0])
    if abs(a @ n) > 0.9:
        a = np.array([1.0, 0.0, 0.0])
    u = np.cross(n, a); u /= np.linalg.norm(u)
    v = np.cross(n, u)

    inl = pts[np.abs((pts - c) @ n) < 0.02]
    uu = (inl - c) @ u
    vv = (inl - c) @ v
    # 取 5~95 分位当边界，掐掉边缘的飞点
    umin, umax = np.percentile(uu, [5, 95])
    vmin, vmax = np.percentile(vv, [5, 95])
    t = Table(c, n, u, v, float(umin), float(umax), float(vmin), float(vmax), rows, cols)
    os.makedirs(OUT, exist_ok=True)
    with open(PLANE_FILE, "w") as f:
        json.dump(t.to_dict(), f, ensure_ascii=False, indent=2)
    print(f"✓ 桌面找到了：内点 {n_in}，范围 "
          f"{umax-umin:.2f} × {vmax-vmin:.2f} m，划成 {rows}×{cols} 格", flush=True)
    return t, None


ZONES_FILE = os.path.join(OUT, "zones.json")


def ray_to_plane(px, py, intr, table):
    """从相机原点经过这个像素打一条射线，求它和桌面平面的交点。

    为什么不直接读这个像素的深度：用户点的是「桌面上的那个位置」，但那个
    像素上可能正好摆着东西，或者是深度空洞。读深度会得到物体表面的点，
    算出来的区域就歪了。射线求交永远落在桌面上，和桌上摆什么无关。
    """
    d = intr.deproject(float(px), float(py), 1.0)
    nd = float(d @ table.normal)
    if abs(nd) < 1e-6:           # 射线和桌面平行，打不到
        return None
    t = float(table.origin @ table.normal) / nd
    if t <= 0:                   # 交点在相机背后
        return None
    return d * t


def do_pick(cam, table):
    """鼠标在画面上点出若干个区域，存成 zones.json。"""
    state = {"cur": [], "zones": [], "frame": None}

    def on_mouse(ev, x, y, flags, _):
        if ev != cv2.EVENT_LBUTTONDOWN:
            return
        p3 = ray_to_plane(x, y, cam.intr, table)
        if p3 is None:
            print("  ✗ 这个位置打不到桌面，换一处点"); return
        uu, vv = table.uv(p3)
        state["cur"].append({"px": (x, y), "uv": (uu, vv)})
        print(f"  + 第 {len(state['cur'])} 个点  uv=({uu:+.3f}, {vv:+.3f}) m", flush=True)

    intr = cam.intr
    cv2.namedWindow("pick")
    cv2.setMouseCallback("pick", on_mouse)
    print("""
框区域：
  左键   点一个顶点（至少 3 个，形状随意，不用是矩形）
  回车   当前区域收口，开始下一个
  u      撤掉上一个点      c  清空当前区域
  s      存盘退出          q  不存退出
""", flush=True)
    try:
        for fr in cam.frames():
            vis = fr.color.copy()

            # 已经定好的区域
            for i, z in enumerate(state["zones"]):
                pts = [table.uv_to_px(u, v, intr) for u, v in z["poly_uv"]]
                pts = [q for q in pts if q]
                if len(pts) >= 3:
                    poly = np.array(pts, np.int32)
                    cv2.polylines(vis, [poly], True, (90, 220, 90), 2)
                    ctr = poly.mean(axis=0).astype(int)
                    cv2.putText(vis, z["name"], (ctr[0]-28, ctr[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (90, 220, 90), 2)
            # 正在画的
            cur = [q["px"] for q in state["cur"]]
            for q in cur:
                cv2.circle(vis, q, 5, (60, 160, 255), -1)
            if len(cur) >= 2:
                cv2.polylines(vis, [np.array(cur, np.int32)], False, (60, 160, 255), 2)
            cv2.rectangle(vis, (0, 0), (W, 28), (35, 35, 35), -1)
            cv2.putText(vis, f"zones={len(state['zones'])}  points={len(cur)}   "
                        f"ENTER=close  u=undo  c=clear  s=save  q=quit",
                        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
            cv2.imshow("pick", vis)

            k = cv2.waitKey(1) & 0xFF
            if k in (13, 10):                       # 回车
                if len(state["cur"]) < 3:
                    print("  ✗ 至少要 3 个点"); continue
                name = f"区域{len(state['zones'])+1}"
                state["zones"].append(
                    {"name": name, "poly_uv": [q["uv"] for q in state["cur"]]})
                print(f"  ✓ {name} 收口，{len(state['cur'])} 个顶点", flush=True)
                state["cur"] = []
            elif k == ord("u") and state["cur"]:
                state["cur"].pop(); print("  - 撤销")
            elif k == ord("c"):
                state["cur"] = []; print("  清空当前")
            elif k == ord("s"):
                if state["cur"]:
                    print("  ✗ 当前这个还没收口，先按回车或 c")
                    continue
                if not state["zones"]:
                    print("  ✗ 一个区域都没有"); continue
                os.makedirs(OUT, exist_ok=True)
                with open(ZONES_FILE, "w") as f:
                    json.dump({"zones": state["zones"],
                               "说明": "poly_uv 是桌面平面内的二维坐标（米），"
                                       "配合 plane.json 用。桌面重新拟合后要重框。",
                               "存于": time.strftime("%Y-%m-%d %H:%M:%S")},
                              f, ensure_ascii=False, indent=2)
                print(f"\n✓ {len(state['zones'])} 个区域 → {ZONES_FILE}")
                return 0
            elif k == ord("q"):
                print("没存，退出。"); return 1
    finally:
        cv2.destroyAllWindows()


def depth_at(dm, px, py, patch=3):
    h, w = dm.shape
    if not (0 <= px < w and 0 <= py < h):
        return None
    p = dm[max(0,py-patch):py+patch+1, max(0,px-patch):px+patch+1]
    g = p[p > 0]
    return float(np.median(g)) if g.size >= 3 else None


def main():
    # W/H 是模块级的（很多地方直接用），开相机后要按实际分辨率改写
    global W, H
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", default="2x3", help="行x列，默认 2x3")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--rezone", action="store_true", help="忽略已存的平面，重新拟合")
    ap.add_argument("--pick", action="store_true", help="鼠标框区域，存成 zones.json")
    ap.add_argument("--force-grid", action="store_true", help="有 zones.json 也用均分网格")
    ap.add_argument("--backend", default=None, choices=["orbbec", "realsense"],
                    help="指定相机后端，默认自动挑在线那台")
    ap.add_argument("--record", default="")
    ap.add_argument("--seconds", type=float, default=0)
    a = ap.parse_args()
    rows, cols = (int(x) for x in a.grid.lower().split("x"))

    cam = open_camera(a.backend, W, H, FPS)
    intr = cam.intr
    # 相机实际给的分辨率可能和请求的不同（挑档挑的是最接近的），以实际为准
    W, H = intr.width, intr.height

    table, err = None, None
    if os.path.isfile(PLANE_FILE) and not a.rezone:
        d = json.load(open(PLANE_FILE))
        if d.get("rows") == rows and d.get("cols") == cols:
            table = Table.from_dict(d)
            print(f"复用已存的桌面平面（要重找加 --rezone）")
    if table is None:
        table, err = build_table(cam, rows, cols)
        if table is None:
            cam.close(); raise SystemExit(f"✗ {err}")

    if a.pick:
        r = do_pick(cam, table)
        cam.close()
        return r

    # 自己框的区域优先；没框过就退回均分网格
    polys = None
    if not a.force_grid and os.path.isfile(ZONES_FILE):
        zs = json.load(open(ZONES_FILE))["zones"]
        polys = {z["name"]: [tuple(q) for q in z["poly_uv"]] for z in zs}
        print(f"用自己框的 {len(polys)} 个区域：{'、'.join(polys)}")
    if polys is None:
        polys = table.grid_polys()
        print(f"用均分网格 {rows}×{cols}（想自己框：先跑 --pick）")

    hands = HandLandmarker.create_from_options(HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=f"{HERE}/models/hand_landmarker.task"),
        running_mode=RunningMode.VIDEO, num_hands=2,
        min_hand_detection_confidence=0.5, min_tracking_confidence=0.5))

    # --record 给了就全程录；没给就用窗口里的 r 键手动控制。
    writer, rec_dir, rec_t0, rec_log = None, None, 0.0, []
    if a.record:
        os.makedirs(a.record, exist_ok=True)
        rec_dir = a.record
        writer = cv2.VideoWriter(os.path.join(a.record, "table.mp4"),
                                 cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    def rec_start(t):
        """开一段新录制。每段独立目录，按下 r 的时刻就是第 0 秒。"""
        d = os.path.join(HERE, "out", time.strftime("rec_%m%d_%H%M%S"))
        os.makedirs(d, exist_ok=True)
        w = cv2.VideoWriter(os.path.join(d, "table.mp4"),
                            cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
        print(f"\n● 开始录制 → {d}\n", flush=True)
        return w, d, t, []

    def rec_stop(w, d, lg, t):
        if w is None:
            return
        w.release()
        if d and lg:
            with open(os.path.join(d, "log.json"), "w") as f:
                json.dump(lg, f, ensure_ascii=False)
        print(f"\n■ 停止录制，{t:.1f} 秒，{len(lg)} 帧 → {d}\n", flush=True)

    # 每个区域的状态要连着 HOLD_N 帧一致才认
    hist = {k: deque(maxlen=HOLD_N) for k in polys}
    stable = {k: "空" for k in polys}
    log, lat, t0, last = [], deque(maxlen=60), time.time(), {}
    print("\n开始。把手伸到桌面上方试试：贴着桌面是「接触」，抬高一点是「悬停」。")
    if a.show:
        print("窗口里：按 r 开始/停止录制，按 q 退出。\n", flush=True)
    else:
        print("", flush=True)
    try:
        for fr in cam.frames():
            t = time.time() - t0
            if a.seconds and t > a.seconds:
                break
            dm, color = fr.depth, fr.color
            ti = time.time()
            hr = hands.detect_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB,
                         data=cv2.cvtColor(color, cv2.COLOR_BGR2RGB)), int(t*1000))
            lat.append((time.time()-ti)*1000)

            # 这一帧每格的原始状态，取该格里最低的那根指尖
            raw = {k: "空" for k in hist}
            marks = []
            for hd in (hr.hand_landmarks or []):
                for tip_i, tip_name in TIPS.items():
                    lm = hd[tip_i]
                    px, py = int(lm.x*W), int(lm.y*H)
                    dd = depth_at(dm, px, py)
                    if not dd:
                        continue
                    p = intr.deproject(float(px), float(py), dd)
                    h = table.height(p)
                    zn = table.zone_of(p, polys)
                    if zn is None or h > HOVER_H or h < -0.05:
                        continue
                    s = "接触" if h < TOUCH_H else "悬停"
                    # 同一区域里有多根指尖时取最靠下那根：伸一根手指去点的时候，
                    # 其余手指还悬在半空，按它们算会把「接触」降级成「悬停」
                    if raw[zn] == "空" or (s == "接触" and raw[zn] == "悬停"):
                        raw[zn] = s
                    marks.append((px, py, h, s, tip_name, zn))

            for k in hist:
                hist[k].append(raw[k])
                if len(hist[k]) == HOLD_N and len(set(hist[k])) == 1:
                    stable[k] = hist[k][0]

            occupied = {k: v for k, v in stable.items() if v != "空"}
            if occupied != last:
                if occupied:
                    txt = "  ".join(f"[{k}]{v}" for k, v in sorted(occupied.items()))
                else:
                    txt = "（全空）"
                print(f"[{t:6.1f}s] {txt}", flush=True)
                last = dict(occupied)
            log.append({"t": round(t,2), "cells": dict(stable)})

            if a.show or writer is not None:
                vis = color.copy()
                for name, poly_uv in polys.items():
                    pts2d = [table.uv_to_px(u, v, intr) for u, v in poly_uv]
                    if any(q is None for q in pts2d) or len(pts2d) < 3:
                        continue
                    poly = np.array(pts2d, np.int32)
                    st = stable[name]
                    col = {"空": (150,150,150), "悬停": (0,200,255),
                           "接触": (40,40,235)}[st]
                    if st != "空":
                        ov = vis.copy()
                        cv2.fillPoly(ov, [poly], col)
                        vis = cv2.addWeighted(ov, 0.35, vis, 0.65, 0)
                    cv2.polylines(vis, [poly], True, col, 2 if st!="空" else 1)
                    ctr = poly.mean(axis=0).astype(int)
                    cv2.putText(vis, name, (ctr[0]-20, ctr[1]+5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
                for px, py, h, s, nm, zn in marks:
                    col = (40,40,235) if s=="接触" else (0,200,255)
                    cv2.circle(vis, (px,py), 7, col, -1)
                    cv2.putText(vis, f"{h*100:.0f}cm", (px+9, py-6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
                cv2.rectangle(vis, (0,0), (W,30), (35,35,35), -1)
                n_t = sum(1 for v in stable.values() if v=="接触")
                n_h = sum(1 for v in stable.values() if v=="悬停")
                cv2.putText(vis, f"touch {n_t}  hover {n_h}   "
                            f"{1000/max(np.mean(lat),1e-6):.0f} FPS",
                            (8,21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230,230,230), 1)
                if writer is not None:
                    el = t - rec_t0
                    # 红点要画在写进文件的那一帧上，这样回放时也看得到在录
                    cv2.circle(vis, (W - 96, 15), 7, (60, 60, 235), -1)
                    cv2.putText(vis, f"REC {int(el)//60:02d}:{int(el)%60:02d}",
                                (W - 82, 21), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (60, 60, 235), 2)
                    writer.write(vis)
                    rec_log.append(log[-1])
                if a.show:
                    cv2.imshow("table_zones", vis)
                    k = cv2.waitKey(1) & 0xFF
                    if k == ord("q"):
                        break
                    if k == ord("r"):
                        if writer is None:
                            writer, rec_dir, rec_t0, rec_log = rec_start(t)
                        else:
                            rec_stop(writer, rec_dir, rec_log, t - rec_t0)
                            writer, rec_dir, rec_log = None, None, []
    finally:
        cam.close(); hands.close()
        # 退出时如果还在录，正常收尾——不然视频尾部写不进去，文件会坏
        if writer is not None:
            rec_stop(writer, rec_dir, rec_log if rec_log else log,
                     (time.time() - t0) - rec_t0)
        if a.show: cv2.destroyAllWindows()
    if log:
        print(f"\n{len(log)} 帧  {log[-1]['t']:.1f} 秒  "
              f"{1000/max(np.mean(lat),1e-6):.1f} FPS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
