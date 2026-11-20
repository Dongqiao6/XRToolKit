#!/usr/bin/env python3
"""同一段录像上对照 BODY_PARTS=upper 和 full：姿态/手只跑一遍，同一份关键点喂两套判定。

为什么同一进程里跑两套：分两次跑虽然模型输出理论上一样，但跟踪器、运动门
都带状态，放一起逐帧比才能说「这一帧的差别就是部位开关带来的」。

    python3 replay_upper.py clips/legs_1111                    # 多边形区域（zone_ground.json）
    python3 replay_upper.py clips/legs_1111 --zone-mode reach  # 机械臂可达空间（start.sh 默认）
    python3 replay_upper.py clips/legs_1111 --out /tmp/ab_upper  # 另存逐帧结果 + 分歧帧图片

分歧帧分三类（只看「full 报手伸入/占用、upper 没报」这个方向——这是可能的漏报）：
    仅腿脚   full 在区内的点全是膝以下 → upper 去掉的正是这类，算「误报减少」
    含髋     在区内的点只有髋和腿脚，upper 的髋没过核验 → 要看图确认是不是桌面
    其他     还有别的点在区内 → 不该出现，必须逐帧看图
反方向（upper 比 full 更危险）也单独数出来。
录像里没有关节角，所以不算机械臂自遮挡（arm_caps 不传）。
"""
import argparse
import collections
import json
import os
import sys

import cv2
import mediapipe as mp
import numpy as np

import zone_monitor as zm
from clip_io import ClipReader

LEGS, HIPS = set(zm.LOWER_LEG), {23, 24}
DANGER = ("手伸入", "占用")
MODES = ("full", "upper")


def reach_zone(camera_id, margin):
    """和 zone_rerun.py --robot --zone-mode reach 同一套算法（机械臂在零位）。"""
    from jaka_model import JakaModel
    from robot_frames import RobotFrames, align_to_floor
    robot = JakaModel()
    T_cam_base = np.linalg.inv(RobotFrames.load().T_base_optical(camera_id))
    pl = zm._zone_cfg().get("plane") or {}
    if pl.get("origin") and pl.get("normal"):
        T_cam_base, _, _ = align_to_floor(T_cam_base, pl["origin"], pl["normal"])

    def to_cam(p):
        return T_cam_base[:3, :3] @ np.asarray(p, float) + T_cam_base[:3, 3]
    sp = robot.reach_spheres(margin=margin, samples=600)
    return zm.ReachZone([(to_cam(o), r) for o, r, _ in sp])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clip")
    ap.add_argument("--zone-mode", default="polygon", choices=["polygon", "reach"])
    ap.add_argument("--reach-margin", type=float, default=0.25)
    ap.add_argument("--camera-id", default="top")
    ap.add_argument("--no-hands", action="store_true", help="不跑手部模型（快一倍）")
    ap.add_argument("--out", default="", help="逐帧结果和分歧帧图片存到这里")
    a = ap.parse_args()

    clip = ClipReader(a.clip)
    intr = clip.intr
    ground = (reach_zone(a.camera_id, a.reach_margin) if a.zone_mode == "reach"
              else zm.Ground.load(zm.ZONE_FILE))
    pose = zm.make_pose()
    hands = None if a.no_hands else zm.make_hands()
    qgate = zm.QualityGate()
    S = {m: {"pt": zm.PersonTracker(), "gr": zm.Grader(), "seen": -1e9}
         for m in MODES}
    rows = []
    if a.out:
        os.makedirs(a.out, exist_ok=True)

    for n, (c, dp, el) in enumerate(clip):
        img = mp.Image(image_format=mp.ImageFormat.SRGB,
                       data=np.ascontiguousarray(c[:, :, ::-1]))
        ts = int(el * 1000)
        pr = pose.detect_for_video(img, ts)
        hr = hands.detect_for_video(img, ts) if hands is not None else None
        poses = pr.pose_landmarks if pr.pose_landmarks else []
        hl = hr.hand_landmarks if (hr is not None and hr.hand_landmarks) else []
        q_ok, q_why = zm.frame_quality(dp, c)
        blind, blind_s = qgate.feed(q_ok, el)
        row = {"frame": n, "t": round(el, 3), "n_pose": len(poses)}
        for m in MODES:
            s = S[m]
            raw, why, info = zm.judge_multi(poses, hl, dp, intr, ground, s["pt"],
                                            el, (blind, q_why, blind_s), None, m)
            # 和 zone_monitor.main 一样的「人突然没了先按看不清」兜底
            if poses:
                s["seen"] = el
            elif raw == "空闲" and el - s["seen"] < zm.VANISH_HOLD_S:
                raw, why = "看不清", "人突然检不到了"
            inz, hips = set(), {}
            for e in info.get("each") or []:
                inz |= {i for i, p in (e.get("pts3d") or {}).items()
                        if zm.in_zone(p, ground)}
                hips.update(e.get("hips") or {})
            row[m] = {"raw": raw, "state": s["gr"].feed(raw, el), "why": why,
                      "in_zone": sorted(inz), "hips": hips}
        rows.append(row)
        if a.out and (row["full"]["raw"] in DANGER) != (row["upper"]["raw"] in DANGER):
            vis = c.copy()
            for k, m in enumerate(MODES):
                cv2.putText(vis, f"{m}: {row[m]['raw']} in={row[m]['in_zone']}",
                            (8, 24 + 26 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 255) if row[m]["raw"] in DANGER else (0, 160, 0), 2)
            cv2.imwrite(os.path.join(a.out, f"diff_{n:05d}.jpg"), vis)

    # ── 汇总 ──
    N = len(rows)
    print(f"\n{a.clip}：{N} 帧，有人 {sum(r['n_pose'] > 0 for r in rows)} 帧，"
          f"区域 {a.zone_mode}")
    for key, name in (("raw", "单帧原始判断"), ("state", "迟滞后的档位")):
        print(f"\n{name}（full → upper）：")
        for st in zm.ORDER:
            f = sum(r["full"][key] == st for r in rows)
            u = sum(r["upper"][key] == st for r in rows)
            print(f"  {st:<6} {f:5d} → {u:5d}")
    leg_only = sum(1 for r in rows if r["full"]["in_zone"]
                   and set(r["full"]["in_zone"]) <= LEGS)
    print(f"\nfull 下「在区内的点全是膝以下」的帧：{leg_only}")

    miss = collections.defaultdict(list)
    for r in rows:
        if r["full"]["raw"] in DANGER and r["upper"]["raw"] not in DANGER:
            z = set(r["full"]["in_zone"])
            kind = ("仅腿脚" if z and z <= LEGS else
                    "含髋" if z and z <= (LEGS | HIPS) else "其他")
            miss[kind].append(r["frame"])
    more = [r["frame"] for r in rows
            if r["upper"]["raw"] in DANGER and r["full"]["raw"] not in DANGER]
    print("full 报手伸入/占用、upper 没报（单帧原始判断）：")
    for kind in ("仅腿脚", "含髋", "其他"):
        v = miss.get(kind, [])
        print(f"  {kind:<4} {len(v):5d} 帧  {v[:20]}{' …' if len(v) > 20 else ''}")
    print(f"upper 报手伸入/占用、full 没报：{len(more)} 帧  {more[:20]}")
    hip_cnt = collections.Counter(v for r in rows for v in r["upper"]["hips"].values())
    print(f"upper 下髋的处理（人·髋次）：{dict(hip_cnt)}")

    if a.out:
        with open(os.path.join(a.out, "per_frame.json"), "w") as f:
            json.dump(rows, f, ensure_ascii=False)
        print(f"\n逐帧结果 → {a.out}/per_frame.json，分歧帧图片 → {a.out}/diff_*.jpg")
    sys.stdout.flush()
    os._exit(0)      # 跳过 MediaPipe 在 Jetson 上很慢的析构（见 zone_rerun.py 末尾）


if __name__ == "__main__":
    main()
