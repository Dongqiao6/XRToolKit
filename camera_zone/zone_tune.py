#!/usr/bin/env python3
"""调工位区域的位置和大小。给现场要用的人用，不用改代码。

设计上把两件事分开了：

    平面   —— 区域贴在哪个面上（地面、台面）。拟合一次，之后不用再动
    形状   —— 在那个面上画多大、画在哪、转多少度。随便调，秒出结果

所以调参数不需要重新拟合，也不需要人站四个角：改数字、看图、满意再存。

常用：

    python3 zone_tune.py                      看现在是什么样，出预览图
    python3 zone_tune.py --size 1.5 1.0       改成 1.5×1.0 米的长方形
    python3 zone_tune.py --offset 0.3 -0.2    在平面内挪：前后 +0.3、左右 -0.2 米
    python3 zone_tune.py --at-person          人站进去，区域自动挪到他脚下
    python3 zone_tune.py --rotate 30          在平面内转 30 度
    python3 zone_tune.py --height 1.8         柱体高度（多高以内算进区域）
    python3 zone_tune.py --near 0.8           离边界多近算「旁边有人」
    python3 zone_tune.py --size 1.5 1.0 --save    确认后存盘

    python3 zone_tune.py --refit              重新拟合平面（相机动过了才需要）
    python3 zone_tune.py --refit --range 1.5 5   同上，但只用 1.5~5 m 的点
                                                 （相机架在桌上时排掉桌面）
    python3 zone_tune.py --show               开窗口实时看，不只是出图

偏移和旋转的方向：
    offset 的两个数是「平面内两个方向」。第一个数增大，区域朝画面下方/远处走；
    第二个数增大，区域朝画面右边走。不确定就先试 ±0.2，看预览图往哪动了。

每次改完都会打印四个角在相机系的坐标和实际边长，便于核对。
"""
import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

from camera_backend import open_camera
from zone_monitor import (Ground, ZONE_FILE, ZONE_H, draw_zone,
                          fit_plane_ransac, W, H, FPS)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "zone_tune")
CFG_FILE = os.environ.get("ZONE_CFG", os.path.join(HERE, "zone_config.json"))

DEFAULT_CFG = {
    "zone_id": "default",
    "说明": "工位区域配置。改完用 zone_tune.py 预览，满意再 --save。",
    "plane": {"origin": None, "normal": None,
              "来源": "尚未拟合。跑 zone_tune.py --refit"},
    "shape": {"size_m": [1.2, 1.2], "offset_m": [0.0, 0.0], "rotate_deg": 0.0},
    "height_m": 2.2,
    "near_m": 0.5,
}


def load_cfg():
    if os.path.isfile(CFG_FILE):
        with open(CFG_FILE) as f:
            cfg = json.load(f)
        for k, v in DEFAULT_CFG.items():
            cfg.setdefault(k, v)
        return cfg
    return json.loads(json.dumps(DEFAULT_CFG))


def save_cfg(cfg, corners):
    cfg["corners3d"] = [[round(float(x), 5) for x in p] for p in corners]
    with open(CFG_FILE, "w") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)
    # 同时写判定程序直接读的那份，省得两边不一致
    Ground(corners).save(ZONE_FILE)
    return CFG_FILE, ZONE_FILE


def plane_axes(normal):
    """在平面内取两个正交方向。取法固定，这样同一组参数每次得到同一块区域。"""
    n = np.asarray(normal, float)
    n = n / (np.linalg.norm(n) + 1e-12)
    a = np.array([1.0, 0, 0])
    if abs(float(np.dot(a, n))) > 0.9:
        a = np.array([0, 1.0, 0])
    ex = np.cross(n, a)
    ex /= np.linalg.norm(ex)
    return ex, np.cross(n, ex), n


def build_corners(origin, normal, size_m, offset_m=(0.0, 0.0), rotate_deg=0.0):
    """按「平面 + 形状参数」算四个角（相机系）。

    平面和形状是分开的：平面拟合一次就定了，形状随便调。所以调参数不用
    重新拟合、不用人站四个角——这正是给现场的人调的那个接口。
    """
    o = np.asarray(origin, float)
    ex, ey, _ = plane_axes(normal)
    sx, sy = [float(v) for v in size_m]
    ox, oy = [float(v) for v in offset_m]
    th = math.radians(float(rotate_deg))
    # 在平面内转：先转轴，再按转过的轴画矩形
    rx = ex * math.cos(th) + ey * math.sin(th)
    ry = -ex * math.sin(th) + ey * math.cos(th)
    c = o + ex * ox + ey * oy
    hx, hy = sx / 2, sy / 2
    return [c - hx*rx - hy*ry, c + hx*rx - hy*ry,
            c + hx*rx + hy*ry, c - hx*rx + hy*ry]


def corners_from(cfg):
    """按配置算四个角（相机系）。"""
    pl = cfg["plane"]
    if pl.get("origin") is None:
        sys.exit("还没拟合平面。先跑：python3 zone_tune.py --refit")
    sh = cfg["shape"]
    return build_corners(pl["origin"], pl["normal"], sh["size_m"],
                         sh.get("offset_m", [0, 0]), sh.get("rotate_deg", 0.0))


def refit(cfg, backend, rng=(0.15, 4.0), floor=True, min_below=0.0):
    """重新拟合平面。相机动过、或者第一次用，才需要跑这个。

    rng 是参与拟合的深度范围。默认把整个画面都算进去，但相机架在桌上时
    桌面离镜头最近、点最密，会被当成「那个平面」——人其实站在两三米外的
    地面上。这时候用 --range 1.5 5 把近处排掉。
    """
    cam = open_camera(backend, W, H, FPS)
    try:
        acc = []
        for fr in cam.frames():
            acc.append(fr.depth)
            if len(acc) >= 20:
                break
        dm = np.median(np.stack(acc), axis=0)
        g_vec, _ = cam.read_gravity(2.0)
    finally:
        cam.close()

    intr = cam.intr
    ys, xs = np.mgrid[0:intr.height:4, 0:intr.width:4]
    zs = dm[::4, ::4]
    m = (zs > rng[0]) & (zs < rng[1])
    if m.sum() < 500:
        sys.exit(f"深度 {rng[0]}~{rng[1]} m 内的点太少（{m.sum()} 个），"
                 f"拟合不了平面。把范围放宽些。")
    pts = intr.deproject_many(np.stack([xs[m], ys[m]], axis=1), zs[m])

    if g_vec is not None:
        g = np.asarray(g_vec, float) / np.linalg.norm(g_vec)
        # 用重力把水平面和墙分开：同一个水平面上的点，沿重力的坐标几乎一样，
        # 直方图上堆成尖峰；墙面的点在这个方向上摊开，堆不出峰。
        h = pts @ g                       # 沿重力的坐标：越大越低
        hist, edges = np.histogram(h, bins=np.arange(h.min(), h.max() + 0.02, 0.02))
        # 在所有够密的水平面里挑**最低**的那个——那才是地面。
        # 挑最密的不行：办公室里桌面点最多，相机架在桌上时拟合必中桌面
        # （11-11 实测连中两次：先是自己这张桌 0.70 m，再是对面等高的工作台）。
        centers = (edges[:-1] + edges[1:]) / 2
        # min_below：地面至少在镜头下方多少米。相机装在机器人头上时，前面的
        # 桌子、机械臂把地面挡掉一大半，地面那层点不够密，会退回挑中桌面
        # （11-14 Jetson 实测：挑出「镜头下方 0.92 m」= 1.67 m 机位 − 0.75 m 桌高）。
        # 告诉它地面不会比这更高，就只在这以下找。
        ok = centers >= min_below
        if not ok.any():
            sys.exit(f"镜头下方 {min_below} m 以下一个点都没有，--min-below 设大了")
        peak = int(hist[ok].max())
        # 阈值按这一段里最密那层的比例定，不写死个数：bin 宽只有 2 cm，
        # 一层通常也就几百个点，写死 300 会一层都选不出来（11-11 踩过）。
        cands = [k for k in range(len(hist))
                 if ok[k] and hist[k] >= max(40, peak * 0.25)]
        dense = sorted(((float(centers[k]), int(hist[k])) for k in range(len(hist))
                        if hist[k] >= max(40, int(hist.max()) * 0.1)), key=lambda x: x[0])
        print("各水平层（镜头下方米数: 点数）:",
              "  ".join(f"{h:+.2f}:{c}" for h, c in dense[-12:]))
        k = max(cands) if (cands and floor) else int(np.argmax(hist))
        band = np.abs(h - (edges[k] + edges[k+1]) / 2) < 0.06
        if band.sum() >= 400:
            pts = pts[band]
            print(f"重力筛选：留下 {band.sum()} 个点"
                  f"（{'最低' if floor else '最密'}的那一层，"
                  f"共 {len(cands)} 层够密）")

    c, n, n_in = fit_plane_ransac(pts)
    d = (pts - c) @ n
    ctr = pts[np.abs(d) < 0.015].mean(axis=0)
    cfg["plane"] = {"origin": [round(float(x), 5) for x in ctr],
                    "normal": [round(float(x), 5) for x in n],
                    "内点": int(n_in),
                    "来源": "RANSAC 拟合 + 重力筛选"}
    if g_vec is not None:
        g = np.asarray(g_vec, float) / np.linalg.norm(g_vec)
        tilt = math.degrees(math.acos(min(1.0, abs(float(np.dot(n, g))))))
        cfg["plane"]["与水平面夹角_度"] = round(tilt, 1)
        print(f"平面与水平面差 {tilt:.1f}°" +
              ("  ⚠ 超过 30°，这多半是面墙" if tilt > 30 else "  ✓ 是水平面"))
    print(f"平面原点 ({ctr[0]:+.2f}, {ctr[1]:+.2f}, {ctr[2]:+.2f}) m，内点 {n_in}"
          f"（拟合用了 {rng[0]:.2f}~{rng[1]:.2f} m 内的点）")
    if g_vec is not None and floor:
        g = np.asarray(g_vec, float) / np.linalg.norm(g_vec)
        print(f"这个面在镜头下方 {float(ctr @ g):+.2f} m"
              f"（相机架在桌上的话，地面大约在下方 0.7~1.2 m）")
    if ctr[2] < 1.0:
        print("⚠ 这个面离镜头不到 1 米，多半是桌面而不是人站的地面。"
              "加 --range 1.5 5 重试。")
    return cfg


def at_person(cfg, backend, seconds=4.0):
    """人站进工位中间，按一下就把区域挪到他脚下。

    给现场的人用的：不用懂相机坐标系，也不用试 offset 的正负号——
    站进去，跑一次，区域就在你脚下。

    取的是躯干中心沿重力方向落到平面上的那个点，不是脚的位置：
    脚常被挡住或在画面外，躯干中心几乎总在；而人是直立的，
    躯干中心的垂直投影就是他站的地方。
    """
    from zone_monitor import make_pose, MIN_VIS, TORSO
    import mediapipe as mp

    pl = cfg["plane"]
    if pl.get("origin") is None:
        sys.exit("还没拟合平面。先跑：python3 zone_tune.py --refit")
    o = np.asarray(pl["origin"], float)
    n = np.asarray(pl["normal"], float)
    n = n / np.linalg.norm(n)
    ex, ey, _ = plane_axes(n)

    cam = open_camera(backend, W, H, FPS)
    det = make_pose()
    got = []
    print(f"站到工位中间，别动，{seconds:.0f} 秒…")
    try:
        t0 = time.time()
        for fr in cam.frames():
            el = time.time() - t0
            if el > seconds:
                break
            img = mp.Image(image_format=mp.ImageFormat.SRGB,
                           data=np.ascontiguousarray(fr.color[:, :, ::-1]))
            res = det.detect_for_video(img, int(el * 1000))
            if not res.pose_landmarks:
                continue
            lms = res.pose_landmarks[0]
            pts = []
            for j in TORSO:
                lm = lms[j]
                u, v = lm.x * cam.intr.width, lm.y * cam.intr.height
                if lm.visibility < MIN_VIS or not (0 <= u < cam.intr.width
                                                   and 0 <= v < cam.intr.height):
                    continue
                z = float(fr.depth[int(v), int(u)])
                if 0.15 < z < 6.0:
                    pts.append(cam.intr.deproject(u, v, z))
            if len(pts) >= 2:
                got.append(np.mean(pts, axis=0))
    finally:
        cam.close()

    if len(got) < 10:
        sys.exit(f"只认出 {len(got)} 帧有人，定不了位。站近一点、让上半身整个入镜。")
    c = np.median(np.array(got), axis=0)
    # 沿平面法向投到平面上：人站的那块地
    foot = c - n * float((c - o) @ n)
    cfg["shape"]["offset_m"] = [round(float((foot - o) @ ex), 3),
                                round(float((foot - o) @ ey), 3)]
    print(f"躯干中心 ({c[0]:+.2f}, {c[1]:+.2f}, {c[2]:+.2f}) m，"
          f"{len(got)} 帧取中位")
    print(f"脚下位置 ({foot[0]:+.2f}, {foot[1]:+.2f}, {foot[2]:+.2f}) m"
          f"  →  平面内偏移 {cfg['shape']['offset_m']}")
    return cfg


def preview(cfg, corners, backend, show=False):
    os.makedirs(OUT, exist_ok=True)
    cam = open_camera(backend, W, H, FPS)
    g = Ground(corners)
    try:
        for i, fr in enumerate(cam.frames()):
            vis = fr.color.copy()
            draw_zone(vis, g, cam.intr, (0, 220, 0))
            cv2.putText(vis, f"{cfg['shape']['size_m'][0]:.2f}x"
                             f"{cfg['shape']['size_m'][1]:.2f}m  "
                             f"h={cfg['height_m']:.1f}m", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 2)
            if not show:
                if i >= 12:
                    break
                continue
            cv2.imshow("zone_tune  (q 退出)", vis)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        cam.close()
        cv2.destroyAllWindows()
    p = os.path.join(OUT, "preview.jpg")
    cv2.imwrite(p, vis)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", nargs=2, type=float, metavar=("长", "宽"),
                    help="区域尺寸（米）。正方形就写两个一样的数")
    ap.add_argument("--offset", nargs=2, type=float, metavar=("方向1", "方向2"),
                    help="在平面内平移（米）")
    ap.add_argument("--rotate", type=float, metavar="度", help="在平面内旋转")
    ap.add_argument("--height", type=float, metavar="米",
                    help="柱体高度：离平面多高以内算进区域")
    ap.add_argument("--near", type=float, metavar="米",
                    help="离边界多近算「旁边有人」")
    ap.add_argument("--zone-id", help="工位编号")
    ap.add_argument("--refit", action="store_true", help="重新拟合平面")
    ap.add_argument("--at-person", action="store_true",
                    help="人站进工位中间，把区域挪到他脚下（不用手算偏移）")
    ap.add_argument("--min-below", type=float, default=0.0, metavar="米",
                    help="地面至少在镜头下方多少米。相机装在机器人头上（约 1.67 m）"
                         "时用 --min-below 1.2，排掉桌面")
    ap.add_argument("--densest", action="store_true",
                    help="挑点最密的平面，而不是最低的。"
                         "区域要画在台面上而不是地面上时用")
    ap.add_argument("--range", nargs=2, type=float, default=[0.15, 4.0],
                    metavar=("近", "远"),
                    help="拟合只用这个深度范围内的点（米）。相机架在桌上时"
                         "用 --range 1.5 5 排掉桌面")
    ap.add_argument("--save", action="store_true", help="存盘（不加只预览）")
    ap.add_argument("--show", action="store_true", help="开窗口实时看")
    ap.add_argument("--no-preview", action="store_true", help="不出预览图")
    ap.add_argument("--backend", default=None, choices=["orbbec", "realsense"])
    a = ap.parse_args()

    cfg = load_cfg()
    if a.refit:
        cfg = refit(cfg, a.backend, tuple(a.range), not a.densest, a.min_below)
    if a.at_person:
        cfg = at_person(cfg, a.backend)
    if a.size:
        cfg["shape"]["size_m"] = [float(a.size[0]), float(a.size[1])]
    if a.offset:
        cfg["shape"]["offset_m"] = [float(a.offset[0]), float(a.offset[1])]
    if a.rotate is not None:
        cfg["shape"]["rotate_deg"] = float(a.rotate)
    if a.height is not None:
        cfg["height_m"] = float(a.height)
    if a.near is not None:
        cfg["near_m"] = float(a.near)
    if a.zone_id:
        cfg["zone_id"] = a.zone_id

    corners = corners_from(cfg)
    sides = [float(np.linalg.norm(corners[i] - corners[(i+1) % 4])) for i in range(4)]
    ctr = np.mean(corners, axis=0)

    sh = cfg["shape"]
    print(f"\n工位 {cfg['zone_id']}")
    print(f"  尺寸      {sh['size_m'][0]:.2f} × {sh['size_m'][1]:.2f} m"
          f"   实际边长 {' / '.join(f'{s:.2f}' for s in sides)}")
    print(f"  平面内偏移 {sh.get('offset_m', [0,0])}   旋转 {sh.get('rotate_deg',0):.0f}°")
    print(f"  柱体高度  {cfg['height_m']:.2f} m"
          f"   「旁边有人」门槛 {cfg['near_m']:.2f} m")
    print(f"  中心(相机系) ({ctr[0]:+.2f}, {ctr[1]:+.2f}, {ctr[2]:+.2f}) m")
    print("  四个角:")
    for i, p in enumerate(corners):
        print(f"    {i+1}  ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})")

    if not a.no_preview:
        try:
            p = preview(cfg, corners, a.backend, a.show)
            print(f"\n预览图 → {p}")
        except Exception as e:
            print(f"\n（出不了预览图：{e}）"
                  f"\n 相机被判定程序占着的话，先停了它再来调。")

    if a.save:
        f1, f2 = save_cfg(cfg, corners)
        print(f"\n✓ 已存 {f1}")
        print(f"✓ 已存 {f2}（判定程序直接读这份）")
        print("  判定程序要重启才会读到新区域。")
    else:
        print("\n（只是预览，没存。满意了加 --save）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
