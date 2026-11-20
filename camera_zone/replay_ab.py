#!/usr/bin/env python3
"""在录好的一段帧上，对开/关骨架反推做对照。同一帧喂两套判定。

    python3 replay_ab.py clips/legs_1111.npz
"""
import collections, sys
import numpy as np
import mediapipe as mp
import zone_monitor as zm
from clip_io import ClipReader

LEG = {25: "左膝", 26: "右膝", 27: "左踝", 28: "右踝"}
clip = ClipReader(sys.argv[1])
intr = clip.intr
pose = zm.make_pose()
ground = zm.Ground.load(zm.ZONE_FILE)
ON, OFF = "开", "关"
arms = {ON: zm.PersonTracker(prior=True), OFF: zm.PersonTracker(prior=False)}
jump = {k: {j: [] for j in LEG} for k in arms}
# 按「上一帧是实测还是反推 → 这一帧是实测还是反推」分类记跳动。
# 不分类的话看不出是反推点自己在跳，还是错的实测点被反推纠正了。
kinds = {j: collections.defaultdict(list) for j in LEG}
big = []
prev = {k: {} for k in arms}
seen = {k: collections.Counter() for k in arms}
n_inf, n_pose = 0, 0

for n, (c, dp, el) in enumerate(clip):
    img = mp.Image(image_format=mp.ImageFormat.SRGB,
                   data=np.ascontiguousarray(c[:, :, ::-1]))
    pr = pose.detect_for_video(img, int(el * 1000))
    poses = pr.pose_landmarks if pr.pose_landmarks else []
    n_pose += 1 if poses else 0
    for k, tr in arms.items():
        st, why, info = zm.judge_multi(poses, [], dp, intr, ground, tr,
                                       float(el), None)
        if k == ON:
            n_inf += info.get("inferred_pts", 0)
        pts = info.get("pts3d") or {}
        inf = info.get("inferred") or set()
        for j in LEG:
            if j not in pts:
                prev[k].pop(j, None); continue
            seen[k][j] += 1
            p = np.asarray(pts[j], float)
            tag = "推" if j in inf else "测"
            if j in prev[k]:
                dj = float(np.linalg.norm(p - prev[k][j][0]))
                jump[k][j].append(dj)
                if k == ON:
                    kinds[j][f"{prev[k][j][1]}→{tag}"].append(dj)
                    if dj > 0.2:
                        big.append((int(n), j, f"{prev[k][j][1]}→{tag}", dj,
                                    prev[k][j][0].copy(), p.copy()))
            prev[k][j] = (p, tag)

print(f"\n{len(clip)} 帧，其中 {n_pose} 帧有人；反推共 {n_inf} 点")
print(f"{'关节':6}{'出现帧数 关→开':>20}{'中位跳动 关→开':>22}{'95分位 关→开':>22}")
for j, nm in LEG.items():
    a, b = np.array(jump[ON][j]), np.array(jump[OFF][j])
    f = lambda v, q: f"{np.percentile(v, q)*100:.1f}" if len(v) else "—"
    cnt = f"{seen[OFF][j]} → {seen[ON][j]}"
    print(f"{nm:6}{cnt:>20}"
          f"{f(b,50)+' → '+f(a,50)+' cm':>22}{f(b,95)+' → '+f(a,95)+' cm':>22}")

print("\n开组按转移类型拆跳动（测=深度量到的，推=骨架反推的）：")
for j, nm in LEG.items():
    for kd, v in sorted(kinds[j].items()):
        a = np.array(v)
        print(f"  {nm} {kd}  n={len(a):3d}  中位 {np.median(a)*100:5.1f} cm  "
              f"最大 {a.max()*100:5.1f} cm")
if big:
    print(f"\n跳超 20 cm 的 {len(big)} 次，前 6 次明细：")
    for n, j, kd, dj, p0, p1 in big[:6]:
        print(f"  帧{n:4d} {LEG[j]} {kd} 跳 {dj*100:5.1f} cm  "
              f"({p0[0]:+.2f},{p0[1]:+.2f},{p0[2]:+.2f}) → "
              f"({p1[0]:+.2f},{p1[1]:+.2f},{p1[2]:+.2f})")
