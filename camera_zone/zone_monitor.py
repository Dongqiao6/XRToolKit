#!/usr/bin/env python3
"""人工操作区占用判定 —— 给节卡机械臂当停机信号用。

和 demo_rerun.py 的区别：那个是空间里一个圆柱体，为了演示好看；这个是**地面上
的一块多边形**，对应现场贴的胶带，为了真的让机器人停下来。

两级精细度（用户 2026-11-05 定）：
    人在哪 —— 看踝关节投到地面的位置，决定「区域是不是被人占了」
    手在哪 —— 看手部关键点投到地面的位置，决定「有没有人伸手进去」

五个档：
    占用      脚在区内
    手伸入    脚在区外、手在区内
    旁边有人  画面里有人，脚和手都在区外
    空闲      画面里没人
    看不清    检到人但关键点丢了 / 没深度

**这个程序只出判断，不决定机器人怎么响应**（用户 2026-11-05 定）。
每帧输出一个档位加一个 risk 等级，执行侧拿去自己决定是停、是减速还是只记一笔。
risk 是建议不是命令，下游可以不听。

「看不清」单独成档，是这套东西和之前演示版最大的区别。实测过 1786 帧里有
149 帧（8.3%）人体识别不出来，而旧逻辑把其中 117 帧判成了「安全」——
看不见人就说安全，失效方向是反的。这里把它拎出来单独报，等级和「占用」
同级，至于要不要真停由执行侧定。

跑法：
    python3 zone_monitor.py --calib          先标定，站四个角各按一次回车
    python3 zone_monitor.py --show           实时监控，开窗口
    python3 zone_monitor.py --record out/xx  监控并录像存证
"""
import argparse
import json
import math
import os
import sys
import time
from collections import deque

import cv2
import mediapipe as mp
import numpy as np
from camera_backend import open_camera
# RANSAC 平面拟合与桌面分区共用一份，别抄第二遍
from table_zones import fit_plane_ransac
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import (HandLandmarker, HandLandmarkerOptions,
                                           PoseLandmarker, PoseLandmarkerOptions,
                                           RunningMode)

HERE = os.path.dirname(os.path.abspath(__file__))
ZONE_FILE = os.environ.get("ZONE_FILE", os.path.join(HERE, "zone_ground.json"))
POSE_MODEL = os.path.join(HERE, "models", "pose_landmarker_full.task")
HAND_MODEL = os.path.join(HERE, "models", "hand_landmarker.task")
W, H, FPS = 848, 480, 30   # 只是向相机**请求**的分辨率。相机可能给不了（Jetson 上
# 实测退到 640×480），换算像素一律用实际图像的 shape / intr.width，别用 W、H

# BlazePose 33 点里我们要的几个
ANKLE_L, ANKLE_R = 27, 28
FOOT_L, FOOT_R = 31, 32          # 脚尖，踝看不见时拿它顶上
# 腿整条出画面时的退路（近距离俯视机位必然如此）：用躯干下缘估人站在哪。
# 比脚差，因为髋膝离地面有高度，投影会偏向相机一侧，所以只用来分
# 「进没进区域」这个粗判断，不用来报距边界多少米。
KNEE_L, KNEE_R = 25, 26
HIP_L, HIP_R = 23, 24
# 身体核心：两肩两髋。这四个点围起来的是躯干，躯干进了区域，就是人进来了。
TORSO = (11, 12, 23, 24)
# 够得着的部位：肘、腕、膝、踝、脚尖。这些伸进去了，但人可能还站在外面。
LIMB_FULL = (13, 14, 15, 16, 25, 26, 27, 28, 31, 32)

# ── 用哪些身体部位判定：BODY_PARTS=upper|full（11-16 用户定，默认 upper）──
# 为什么要只用上半身：人站在桌后、或腿出画面时，MediaPipe 照样给膝/踝/脚的
# 二维坐标，那个像素上的深度其实是桌面，三维点就「落到桌面上」，被判成
# 手伸入/占用。MIN_PRESENCE、贴边、离躯干半径、骨长、骨架反推都只是在补，
# 根子在「腿脚的像素经常不在人身上」。保护的是机械臂可达空间（齐腰往上），
# 腿脚进不进去，上半身（肩、肘、腕、手、头）早一步就进去了。
#   upper  膝 25/26、踝 27/28、脚跟 29/30、脚尖 31/32 完全不参与判定：
#          不进 LIMB、不进骨架反推、不进飞点检查、不算进 MIN_GOOD_LMS，也不画。
#          髋 23/24 留着，但要过 check_hips 的可信检查（髋也常被桌子挡）。
#   full   原来的行为，一行不变，留着做对照。
# 环境变量优先；没设就看 zone_config.json 的 "body_parts"；都没有就 upper。
LOWER_LEG = (25, 26, 27, 28, 29, 30, 31, 32)
LIMB_UPPER = (13, 14, 15, 16)


def _zone_cfg():
    """读 zone_config.json（zone_tune 写的那份）。没有或读坏了就当空。"""
    try:
        with open(os.environ.get("ZONE_CFG", os.path.join(HERE, "zone_config.json"))) as f:
            return json.load(f)
    except Exception:
        return {}


def _body_parts_setting():
    v = os.environ.get("BODY_PARTS", "").strip().lower()
    if not v:
        v = str(_zone_cfg().get("body_parts", "")).strip().lower()
    if v not in ("upper", "full"):
        if v:
            print(f"⚠ BODY_PARTS={v!r} 不认识，按 upper 处理", file=sys.stderr)
        v = "upper"
    return v


BODY_PARTS = _body_parts_setting()
# 模块级的 LIMB 跟着开关走，画点的地方直接用它；判定函数按调用时的 parts 取。
LIMB = LIMB_UPPER if BODY_PARTS == "upper" else LIMB_FULL
# upper 下「够清楚的关键点」只数髋及以上（0~24 共 25 个点）。门槛按比例从
# 12/33 缩成 9/25：不缩的话，腿本来能凑数的人（例如侧身、半个上身被挡）
# 会被当成「没人」报空闲——那是漏报方向，不能为了少误报去换。
MIN_GOOD_LMS_UPPER = int(os.environ.get("MIN_GOOD_LMS_UPPER", "9"))
# 髋的可信检查（upper 才用，见 check_hips）：
#   髋比同侧肩**在水平方向上**离相机近这么多（米），就当它落在了前面的桌子上。
#   人直立或前倾时髋不会比肩更靠近相机；桌子在人和相机之间，取到的深度
#   会明显偏近。0.20 m 给后仰、深度噪声留了余量。
#   必须按水平距离比、不能直接比光轴深度 z：相机是俯拍的（JAKA 上约俯 31°），
#   站直的人髋本来就比肩的 z 大 0.25 m 左右，直接比 z 会把桌子抵消掉。
HIP_NEAR_M = float(os.environ.get("HIP_NEAR_M", "0.20"))


def _up_from_cfg():
    """相机系里「朝上」的单位向量，取自 zone_config.json 实测地面的法向。
    相机系 +Y 朝下，所以朝上的那个法向 Y 分量为负。没有就返回 None。"""
    n = (_zone_cfg().get("plane") or {}).get("normal")
    if not n:
        return None
    n = np.asarray(n, float)
    n = n / (np.linalg.norm(n) + 1e-12)
    return n if n[1] < 0 else -n


UP_CAM = _up_from_cfg()
# 肩到髋（同侧或对侧）的三维距离合理范围。比 BONE_RANGE 的躯干侧边宽一点，
# 因为同侧肩缺了时要拿对侧肩比，是斜对角。
HIP_SHOULDER_RANGE = (0.25, 0.85)
# 头：鼻子和五官。探头进来也算侵入。
HEAD = tuple(range(0, 11))
MIN_VIS = 0.5                    # 关键点可见度低于这个就当没有
# 关键点在不在画面里。MediaPipe 对出画面的点**仍然给坐标**——它外推出一个
# 位置，通常贴在画面边缘上。那个位置的深度是别的东西（桌子、墙），于是
# 三维点就"飞"到那个东西上去了。11-11 用户实测：下半身出画面时，
# 膝和踝的标记点飞到身体前方。飞点要是落进区域，就会误报「占用」。
MIN_PRESENCE = float(os.environ.get("MIN_PRESENCE", "0.5"))
EDGE_MARGIN = float(os.environ.get("EDGE_MARGIN", "0.02"))   # 归一化坐标贴边多少算可疑
# 从躯干中心到身上任何一点的距离上限。肩到指尖约 0.9 m，髋到脚尖约 1.0 m，
# 取 1.3 m 留余量。超过这个数的点不可能还长在同一个人身上。
MAX_BODY_R = float(os.environ.get("MAX_BODY_R", "1.3"))
# 骨头长度的合理范围（米，成人）。比「离躯干多远」精确得多：
# 膝盖飞到桌面上时离躯干可能才 0.8 m，半径检查漏得过去，
# 但「大腿 0.8 m」一眼就不对——人腿没那么长。
# 范围取宽一点，宁可漏掉个别飞点，也别把真关节误删。
BONE_RANGE = {
    (11, 12): (0.22, 0.55),   # 肩宽
    (23, 24): (0.12, 0.45),   # 髋宽
    (11, 23): (0.30, 0.75), (12, 24): (0.30, 0.75),   # 躯干侧边
    (11, 13): (0.16, 0.45), (12, 14): (0.16, 0.45),   # 上臂
    (13, 15): (0.14, 0.42), (14, 16): (0.14, 0.42),   # 前臂
    (23, 25): (0.25, 0.65), (24, 26): (0.25, 0.65),   # 大腿
    (25, 27): (0.22, 0.60), (26, 28): (0.22, 0.60),   # 小腿
    (27, 31): (0.06, 0.32), (28, 32): (0.06, 0.32),   # 脚
}
# MediaPipe 的姿态模型是单人模型：画面里没人它也会硬给一套关键点，
# 只是每个点的 visibility 很低。11-05 实测，办公桌俯视、画面里只有椅子和
# 搭在椅背上的衣服，它照样「检到人」。所以要在整体层面再卡一道：
# 可见度够高的点不到这么多，就当没人。33 个点里要求至少 12 个。
MIN_GOOD_LMS = 12
# 姿态检测置信度。11-05 在办公桌俯视机位实测（画面里只有椅子和搭着的衣服）：
#   0.5 → 346 帧里 283 帧「检到人」（82% 误检）
#   0.8 → 346 帧里 5 帧（1%）
# visibility 在这里救不了：误检时髋和肩的可见度都报 1.00。
# ⚠ 0.8 对真人的检出率还没验过，换机位后要当场测一遍再定。
POSE_CONF = float(os.environ.get("POSE_CONF", "0.8"))
PATCH = 3                        # 深度在关键点周围取 7×7 的中值，单点太跳

# 区域是立体的，不是一张地面上的平面图形：底面是划的多边形，往上拉 ZONE_H 米。
# 人身上任何一个三维点落进这个柱体就算侵入——这样不需要看见脚。
# 原先那套「把脚投影到地面」的做法要求相机能拍到鞋，近距离机位根本做不到；
# 改成三维之后，看得见肩膀就能判。
ZONE_H = float(os.environ.get("ZONE_H", "2.2"))
# 人离区域边界多近算「旁边有人」（米，水平距离）
NEAR_M = float(os.environ.get("NEAR_M", "0.5"))

# 人一旦消失，先按上一个状态保持这么久再松口。人不会在一帧之间凭空消失，
# 突然检不到多半是被挡住了或者转身了——这时候放行比误停危险得多。
VANISH_HOLD_S = 1.5
# 升档（往更危险的方向）立刻生效；降档要连着这么多帧才算数，防止抖动。
RELAX_N = 12
# 越靠前越危险，降档一次只能退一级
ORDER = ["空闲", "旁边有人", "看不清", "手伸入", "占用"]
# 每档给一个风险等级，供执行侧参考。2 = 区域被占，1 = 需要留意，0 = 无人。
# 这里只表达「我看到了什么」，不表达「你该怎么做」——怎么响应是执行侧的事。
RISK = {"空闲": 0, "旁边有人": 1, "看不清": 2, "手伸入": 2, "占用": 2}


def depth_at(depth_m, px, py, patch=PATCH):
    """取一个点附近的中值深度。返回 None 表示这片全是空洞。

    用中值不用均值：深度图的空洞和飞点都是极值，均值会被它们拽走。
    """
    h, w = depth_m.shape
    if not (0 <= px < w and 0 <= py < h):
        return None
    y0, y1 = max(0, py - patch), min(h, py + patch + 1)
    x0, x1 = max(0, px - patch), min(w, px + patch + 1)
    good = depth_m[y0:y1, x0:x1]
    good = good[good > 0]
    return float(np.median(good)) if good.size >= 3 else None


def to_3d(px, py, d, intr):
    """像素 + 深度 → 相机坐标系下的三维点（米）。"""
    return intr.deproject(float(px), float(py), d)


def fit_plane(pts):
    """最小二乘拟合平面，返回 (原点, 法向)。

    四个角点肯定不严格共面（人站的位置有高低、深度有噪声），所以不能直接取
    三点定面——那样第四个点的误差会全压在一条边上。
    """
    pts = np.asarray(pts, float)
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c)
    n = vt[-1]
    return c, n / (np.linalg.norm(n) + 1e-12)


class Ground:
    """地面上的那块多边形区域。

    做法是把三维问题压成二维：先在拟合出的地面上建一个二维坐标系，
    把四个角点和待测点都投上去，然后用射线法判点在不在多边形里。
    这样区域可以是任意四边形，不要求是矩形——现场贴的胶带本来也贴不正。
    """

    def __init__(self, corners3d):
        self.corners3d = np.asarray(corners3d, float)
        self.origin, self.normal = fit_plane(self.corners3d)
        # 在平面上随便找两个正交方向当二维轴
        a = np.array([1.0, 0, 0])
        if abs(np.dot(a, self.normal)) > 0.9:
            a = np.array([0, 1.0, 0])
        self.ex = np.cross(self.normal, a)
        self.ex /= np.linalg.norm(self.ex) + 1e-12
        self.ey = np.cross(self.normal, self.ex)
        self.poly2d = np.array([self.to_2d(p) for p in self.corners3d])

    def to_2d(self, p):
        v = np.asarray(p, float) - self.origin
        return np.array([np.dot(v, self.ex), np.dot(v, self.ey)])

    def height(self, p):
        """点离地面多高（沿法向，米）。符号看法向朝哪，只用绝对值。"""
        return abs(float(np.dot(np.asarray(p, float) - self.origin, self.normal)))

    def inside(self, p, margin=0.0):
        """投影到地面之后，在不在多边形里。margin>0 把区域往外扩一圈。"""
        q = self.to_2d(p)
        poly = self.poly2d
        if margin:
            c = poly.mean(axis=0)
            poly = c + (poly - c) * (1.0 + margin)
        # 射线法：从 q 往 +x 打一条射线，数穿过几条边
        n, inside = len(poly), False
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            if (y1 > q[1]) != (y2 > q[1]):
                xx = x1 + (q[1] - y1) * (x2 - x1) / (y2 - y1 + 1e-12)
                if xx > q[0]:
                    inside = not inside
        return inside

    def dist_to_edge(self, p):
        """投影点到多边形边界的距离，在区内为正。给「还差多远」用。"""
        q, poly = self.to_2d(p), self.poly2d
        best = 1e9
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            ab = b - a
            t = np.clip(np.dot(q - a, ab) / (np.dot(ab, ab) + 1e-12), 0, 1)
            best = min(best, float(np.linalg.norm(q - (a + t * ab))))
        return best if self.inside(p) else -best

    def save(self, path):
        with open(path, "w") as f:
            json.dump({"corners3d": self.corners3d.tolist(),
                       "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                       "说明": "相机坐标系下的四个地面角点，单位米。"
                               "换了相机位置就要重标。"}, f,
                      ensure_ascii=False, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls(json.load(f)["corners3d"])


class ReachZone:
    """用机械臂的**可达空间**当保护范围。

    和划一块固定区域相比，这才是老师说的「不用固定区域」的做法：
    保护范围由机器人自己的结构决定，不用人去量、去贴胶带，换个场地
    也不用重划。

    为什么用「可达空间」而不是「当前姿态的包络」：

        当前包络   范围小，但要实时关节角。执行侧断了就没数据，
                   而且臂一秒就能摆过去，按此刻的位置判等于没有余量
        可达空间   范围大一圈，但装好就固定。人进来就是危险，
                   不管臂现在在哪——这是保护范围该有的语义

    接口跟 Ground 对齐（inside / height / dist_to_edge），所以判定逻辑
    一行都不用改。height 恒为 0：球形范围没有「离地多高」这回事，
    进了球就是进了。
    """

    def __init__(self, spheres, margin=0.0):
        # spheres: [(球心, 半径, 元信息?), …]，必须已经在相机系里
        self.spheres = [(np.asarray(o, float), float(r) + float(margin))
                        for o, r, *_ in spheres]
        self.margin = float(margin)

    def inside(self, p, margin=0.0):
        p = np.asarray(p, float)
        return any(float(np.linalg.norm(p - o)) <= r * (1.0 + margin)
                   for o, r in self.spheres)

    def height(self, p):
        return 0.0          # 球形范围没有高度概念，进了就是进了

    def dist_to_edge(self, p):
        """到球面的距离，球内为正——和 Ground 的约定一致。"""
        p = np.asarray(p, float)
        return max(r - float(np.linalg.norm(p - o)) for o, r in self.spheres)

    def describe(self):
        return "  ".join(f"球心({o[0]:+.2f},{o[1]:+.2f},{o[2]:+.2f}) r={r:.2f}m"
                         for o, r in self.spheres)


class PersonTracker:
    """多人跟踪：把每帧检出的若干人和已有轨迹对上号。

    为什么必须分人跟：运动门（动过才算人）和关节平滑器都带状态。
    两个人共用一套的话，甲走动会让乙（一件挂着的衣服）也被认成人，
    甲的膝盖和乙的膝盖会被当成同一个点来算「跳没跳」——全乱。

    关联用最近邻：这一帧每个人的中心，和上一帧各条轨迹的位置比，
    最近且在 MATCH_M 以内就算同一个人。场景里人不多、帧率够高
    （43~50 FPS，一帧人最多移动几厘米），这么简单就够。
    真要在拥挤场景里分清谁是谁，得上外观特征，那是另一个量级的工程。
    """

    MATCH_M = float(os.environ.get("PT_MATCH_M", "0.9"))   # 帧间匹配距离上限
    DROP_S = float(os.environ.get("PT_DROP_S", "1.5"))     # 多久没更新就丢弃

    def __init__(self, prior=None):
        self.tracks = {}       # id -> {"c":中心, "t":时刻, "gate":…, "jt":…}
        self._next = 1
        # 骨架反推开/关。留这个参数是为了能在同一帧上跑两套做对照——
        # 分两次录像对照不成立，两段里人的站位不一样（11-11 实测踩过）。
        self.prior_on = PRIOR_ON if prior is None else bool(prior)

    def assign(self, centers, t):
        """centers 是这一帧每个人的中心点。返回和它一一对应的轨迹 id 列表。"""
        ids = [None] * len(centers)
        free = {k: v for k, v in self.tracks.items() if t - v["t"] <= self.DROP_S}

        # 贪心最近邻：把所有 (检测, 轨迹) 组合按距离排序，从最近的开始配对
        pairs = []
        for i, c in enumerate(centers):
            for k, tr in free.items():
                d = float(np.linalg.norm(np.asarray(c, float) - tr["c"]))
                if d <= self.MATCH_M:
                    pairs.append((d, i, k))
        pairs.sort()
        used_i, used_k = set(), set()
        for d, i, k in pairs:
            if i in used_i or k in used_k:
                continue
            ids[i] = k
            used_i.add(i); used_k.add(k)

        for i, c in enumerate(centers):
            if ids[i] is None:                      # 没配上，是新出现的人
                ids[i] = self._next
                self._next += 1
                # 骨长先验也按人分开：每个人高矮不同，共用一套会互相带偏
                self.tracks[ids[i]] = {"gate": MotionGate(), "jt": JointTracker(),
                                       # 关掉骨架反推的退路：PRIOR=0 或构造传 prior=False
                                       "prior": SkeletonPrior() if self.prior_on
                                       else None}
            tr = self.tracks[ids[i]]
            tr["c"] = np.asarray(c, float)
            tr["t"] = t

        # 太久没更新的轨迹连同它的状态一起丢掉，免得 id 越积越多
        for k in [k for k, v in self.tracks.items() if t - v.get("t", -1e9) > self.DROP_S]:
            self.tracks.pop(k, None)
        return ids

    def state_of(self, tid):
        tr = self.tracks[tid]
        return tr["gate"], tr["jt"], tr["prior"]

    def n_active(self, t):
        return sum(1 for v in self.tracks.values() if t - v["t"] <= self.DROP_S)


class Grader:
    """把每帧的原始判断收敛成一个稳定的档位。

    两条规矩：
      升档立刻 —— 往危险方向走不等待，人进来了就得马上停
      降档要熬 —— 连着 RELAX_N 帧都更安全才松一级，且一次只退一级
    不对称是故意的：误停的代价是停一下，漏停的代价是撞人。
    """

    def __init__(self):
        self.state = "空闲"
        self.hist = deque(maxlen=RELAX_N)
        self.changed_at = 0.0

    def feed(self, raw, t):
        self.hist.append(raw)
        ri, ci = ORDER.index(raw), ORDER.index(self.state)
        if ri > ci:                                  # 更危险，立刻升
            self.state, self.changed_at = raw, t
        elif (len(self.hist) == RELAX_N
              and all(ORDER.index(h) < ci for h in self.hist)):
            self.state = ORDER[ci - 1]               # 只退一级
            self.changed_at = t
            self.hist.clear()
        return self.state


class MotionGate:
    """静止的东西不是人。但人站着不动，仍然是人。

    为什么要有这道门：11-05 实测，画面里只有一把椅子和搭在上面的衣服，
    姿态模型照样「检到人」，82% 的帧误报。把检测门槛从 0.5 提到 0.8 能压到
    1%，但真人检出率也掉了最多 18 个百分点——漏检真人比误停危险。
    换个角度：衣服不会动，人会动。按位移筛就不用牺牲检出率。

    ⚠️ 11-11 真人实测暴露的坑，以及现在的做法：

    第一版写成「连续动才算人、认定后信任 12 秒」。结果人走进区域站定不动，
    12 秒后系统报「空闲」——人还站在里面，机器人却被告知可以动。
    这是最危险的失效方向。

    错在把门设计成了「持续证明自己是人」。正确的分法是：

        **新出现的目标**  要动过才认 —— 这才是真正要挡的（椅子、衣服）
        **认过的目标**    只要位置连得上就一直认，不管它动不动
        **跟丢之后**      再宽限 LOST_S 秒，超了才撤销认定

    位置连得上的判据是「这一帧的中心离上一帧确认位置不超过 TRACK_M」。
    人站着不动 → 位置几乎不变 → 一直认。人走了 → 检不到 → 宽限后撤销。
    衣服 → 从来没动过 → 从来没被认过。
    """

    # 五个参数都能用环境变量调，现场不用改代码。
    WIN_S = float(os.environ.get("GATE_WIN_S", "1.2"))      # 看最近多久的位移
    MOVE_M = float(os.environ.get("GATE_MOVE_M", "0.12"))   # 净移动多少米算「在动」
    SMOOTH_N = int(os.environ.get("GATE_SMOOTH_N", "5"))    # 中心点中位滤波窗口
    CONFIRM_N = int(os.environ.get("GATE_CONFIRM_N", "8"))  # 连着几帧在动才认定
    TRACK_M = float(os.environ.get("GATE_TRACK_M", "0.60")) # 帧间跳多远就算换了目标
    LOST_S = float(os.environ.get("GATE_LOST_S", "2.0"))    # 跟丢后再宽限多久

    def __init__(self):
        self.raw = deque(maxlen=self.SMOOTH_N)
        self.hist = deque()          # (时刻, 滤波后的中心点)
        self.streak = 0
        self.locked = False          # 已认定是人
        self.last_pos = None         # 认定目标最后的位置
        self.last_seen = -1e9

    def feed(self, pts, t):
        """pts 是这一帧人身上的三维点。返回 (是不是人, 窗口内净位移)。"""
        if not pts:
            self.raw.clear(); self.hist.clear(); self.streak = 0
            # 检不到人：宽限 LOST_S 秒。人转个身、被挡一下不该立刻失去认定。
            if self.locked and (t - self.last_seen) > self.LOST_S:
                self.locked = False
                self.last_pos = None
            return self.locked, 0.0

        self.raw.append(np.median(np.array(pts), axis=0))
        # 中位滤波：误检时关键点会在椅子和显示器之间来回跳，单帧能跳 40 cm，
        # 不滤掉的话一次跳变就骗过门槛（11-11 实测：静物中位抖 3.4 cm，
        # 最大 46 cm，24% 的帧超过 6 cm）。
        c = np.median(np.array(self.raw), axis=0)

        if self.locked:
            # 已经认定是人。只要位置连得上就继续认——不要求他继续动，
            # 站着不动的人仍然是人（这正是最该判占用的时候）。
            if np.linalg.norm(c - self.last_pos) <= self.TRACK_M:
                self.last_pos, self.last_seen = c, t
                return True, 0.0
            # 位置跳了一大截：多半换了目标，重新验证
            self.locked = False
            self.streak = 0
            self.hist.clear()

        self.hist.append((t, c))
        while self.hist and t - self.hist[0][0] > self.WIN_S:
            self.hist.popleft()

        # 用**净位移**（窗口首尾距离）不用极值间距：抖动是随机游走，来回抵消后
        # 净位移很小；真人走动是有方向的，净位移累积得起来。
        net = float(np.linalg.norm(c - self.hist[0][1])) if len(self.hist) >= 2 else 0.0

        # 还要求连着 CONFIRM_N 帧都在动。抖动不会持续，真人走两步就够了。
        self.streak = self.streak + 1 if net >= self.MOVE_M else 0
        if self.streak >= self.CONFIRM_N:
            self.locked = True
            self.last_pos, self.last_seen = c, t
        return self.locked, net


# 画面可不可信的几道门槛。镜头被挡住时人体检测什么也检不到，
# 如果只看人体检测结果，就会得出「没人 → 空闲」——挡住摄像头反而
# 让机器人以为安全。所以必须独立判断画面本身。
MIN_DEPTH_VALID = float(os.environ.get("Q_DEPTH_VALID", "0.25"))  # 深度有效像素占比
MAX_NEAR_COVER = float(os.environ.get("Q_NEAR_COVER", "0.30"))    # 贴镜头的像素占比
NEAR_M_COVER = 0.25      # 多近算「贴在镜头上」
MIN_TEXTURE = float(os.environ.get("Q_TEXTURE", "18"))            # 拉普拉斯方差
DARK_V, BRIGHT_V = 28, 238


class QualityGate:
    """画面质量的时间迟滞。

    为什么需要：一帧看不清就停机器人太敏感——人挥手从镜头前过、反光闪一下、
    深度在门槛上抖一下，都会触发一次暂停。频繁误停的系统，现场的人会想办法
    把它关掉，那就比没有还糟。

    但也不能不管：看不清持续下去，我们是真的不知道区域里有没有人。

    所以按**持续时间**分开：

        闪一下（< BAD_S）   保持上一个判断，只记一笔
        持续（≥ BAD_S）     才真的报「看不清」，升到 risk=2
        恢复后（≥ GOOD_S）  稳定一段才取消

    两个时间都能用环境变量调。BAD_S 调小 = 更敏感更容易停；
    调大 = 更顺畅但遮挡发生后反应慢。现场按误停频率调。
    """

    BAD_S = float(os.environ.get("Q_BAD_S", "0.4"))    # 连续不可信多久才报
    GOOD_S = float(os.environ.get("Q_GOOD_S", "0.3"))  # 恢复后稳定多久才取消

    def __init__(self):
        self.bad_since = None
        self.good_since = None
        self.blind = False      # 当前是否判定为「看不清」

    def feed(self, ok, t):
        """返回 (是否报看不清, 已经连续不可信多少秒)。"""
        if ok:
            self.bad_since = None
            if self.good_since is None:
                self.good_since = t
            if self.blind and (t - self.good_since) >= self.GOOD_S:
                self.blind = False
            return self.blind, 0.0

        self.good_since = None
        if self.bad_since is None:
            self.bad_since = t
        dur = t - self.bad_since
        if dur >= self.BAD_S:
            self.blind = True
        return self.blind, dur


def frame_quality(depth_m, color):
    """这一帧的画面可不可信。返回 (可信, 不可信的原因)。

    不看有没有人，只看「我还能不能看见」。四种看不见：
      深度大面积无效   —— 对着玻璃、强光、或者太近太远
      有东西贴着镜头   —— 手、布、纸挡上来
      太暗或过曝       —— 灯关了、强光直射
      几乎没有纹理     —— 蒙上一块布，亮度正常但什么都分辨不出
    """
    valid = float((depth_m > 0).mean())
    if valid < MIN_DEPTH_VALID:
        return False, f"深度图只有 {valid*100:.0f}% 的像素测得到（门槛 {MIN_DEPTH_VALID*100:.0f}%）"

    near = float(((depth_m > 0) & (depth_m < NEAR_M_COVER)).mean())
    if near > MAX_NEAR_COVER:
        return False, f"{near*100:.0f}% 的画面贴在镜头上（{NEAR_M_COVER*100:.0f} cm 内）"

    g = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    mv = float(g.mean())
    if mv < DARK_V:
        return False, f"画面太暗（平均亮度 {mv:.0f}）"
    if mv > BRIGHT_V:
        return False, f"画面过曝（平均亮度 {mv:.0f}）"

    # 拉普拉斯方差衡量「有多少边缘」。蒙一块布亮度正常，但方差会掉到个位数。
    tex = float(cv2.Laplacian(g, cv2.CV_64F).var())
    if tex < MIN_TEXTURE:
        return False, f"画面几乎没有纹理（{tex:.1f}），镜头多半被挡住了"
    return True, ""


def collect_3d(pose_lms, idxs, depth_m, intr):
    """把一组关键点转成相机系三维坐标。拿不到深度、或明显不可信的直接丢掉。

    三道过滤，针对的都是同一类错误——**点其实不在画面里，但模型给了个
    外推坐标**，于是取到背景物体的深度，三维位置飞出去：

      visibility  模型自己说这点有多可见
      presence    模型自己说这点在不在画面内（和 visibility 不是一回事）
      贴边        外推出来的点几乎都压在画面边缘上

    第四道（离躯干太远）在 prune_flyaway 里做，因为那需要先知道躯干在哪。
    """
    out = {}
    for i in idxs:
        lm = pose_lms[i]
        if getattr(lm, "visibility", 1.0) < MIN_VIS:
            continue
        if getattr(lm, "presence", 1.0) < MIN_PRESENCE:
            continue
        if not (EDGE_MARGIN <= lm.x <= 1 - EDGE_MARGIN
                and EDGE_MARGIN <= lm.y <= 1 - EDGE_MARGIN):
            continue
        px, py = int(lm.x * depth_m.shape[1]), int(lm.y * depth_m.shape[0])
        d = depth_at(depth_m, px, py)
        if d is None or d <= 0:
            continue
        out[i] = to_3d(px, py, d, intr)
    return out


class JointTracker:
    """关节点的时间一致性检查 + 平滑。

    最根本的一条：**人的关节不会瞬移**。挥手最快大约 5 m/s，30 FPS 下
    一帧顶多移动 17 cm。跳得比这还远的，一定是深度取错了——点其实在
    画面外被外推到了边缘，取到背景（桌面、墙、机械臂表面）的深度。

    空间上的检查（离躯干多远、骨头多长）抓不住所有情况：膝盖飞到
    近处桌面上时，离躯干 0.8 m、大腿长度也可能勉强在范围内。但它
    在时间上一定是「上一帧还在身上，这一帧跳到半米外」——这个瞒不住。

    拒绝之后不拿旧位置顶替：那样人真走了会一直用过期位置，更危险。
    直接当这个点这一帧没看到。但连续拒绝 MAX_REJECT 帧就接受新位置，
    免得人快速移动时被永久卡在原地。

    平滑用 EMA，系数 0.5 ≈ 延迟一帧。安全系统里延迟是要付代价的，
    所以不敢压太狠——够压掉深度噪声的抖就行。
    """

    MAX_SPEED = float(os.environ.get("JT_MAX_SPEED", "6.0"))    # m/s
    TOL_M = float(os.environ.get("JT_TOL_M", "0.05"))           # 深度噪声容差
    EMA = float(os.environ.get("JT_EMA", "0.5"))                # 新观测占的权重
    MAX_REJECT = int(os.environ.get("JT_MAX_REJECT", "5"))      # 连拒几帧就认
    FORGET_S = float(os.environ.get("JT_FORGET_S", "1.0"))      # 多久没更新就忘掉

    def __init__(self):
        self.last = {}        # 序号 -> (时刻, 位置, 连续拒绝次数)

    def feed(self, pts, t):
        """pts 是 {序号: 三维点}。返回 (过滤平滑后的点, 拒掉几个)。"""
        out, rejected = {}, 0
        for i, p in pts.items():
            h = self.last.get(i)
            if h is None or (t - h[0]) > self.FORGET_S:
                out[i] = p
                self.last[i] = (t, p, 0)
                continue
            t0, p0, nrej = h
            dt = max(t - t0, 1e-3)
            jump = float(np.linalg.norm(p - p0))
            if jump > self.MAX_SPEED * dt + self.TOL_M and nrej < self.MAX_REJECT:
                # 跳太远：当这个点这一帧没看到。不拿旧位置顶替——
                # 人真走了的话，用过期位置比没有位置更危险。
                self.last[i] = (t0, p0, nrej + 1)
                rejected += 1
                continue
            q = self.EMA * p + (1 - self.EMA) * p0
            out[i] = q
            self.last[i] = (t, q, 0)
        # 太久没出现的忘掉，免得人离开再回来时被旧位置挡住
        for i in [k for k, v in self.last.items() if t - v[0] > self.FORGET_S]:
            self.last.pop(i, None)
        return out, rejected


# 从躯干往外的父子链。反推只能顺着这个方向走：父关节可信了，才能推子关节。
BONE_CHAIN = [
    (11, 13), (13, 15),          # 左肩→肘→腕
    (12, 14), (14, 16),          # 右
    (23, 25), (25, 27), (27, 31),  # 左髋→膝→踝→脚尖
    (24, 26), (26, 28), (28, 32),  # 右
]
# upper 下只反推手臂。腿链去掉——否则髋可信时会从髋把膝「推」出来，
# 等于把刚去掉的腿又加回去。
BONE_CHAIN_UPPER = [ij for ij in BONE_CHAIN if ij[1] not in LOWER_LEG]


class SkeletonPrior:
    """骨长先验：记住**这个人**各段骨头多长，用来反推测不准的关节。

    为什么要反推而不是直接丢：腿在画面里看得清清楚楚，只是深度取错了
    （被外推到画面边缘，取到桌面/墙的深度）。丢掉的话腿就一闪一闪地
    消失，看着像检测失败；而实际上我们**知道**腿在哪——图像上的位置
    是准的，缺的只是距离。

    反推的原理：子关节必须同时满足两件事——
      ① 投影回图像落在模型给的那个像素上（图像位置比深度可靠得多）
      ② 离父关节正好一个骨头的长度
    这就是「一条射线和一个球求交」，解一个二次方程。

    骨长哪来：先用人体统计的中值，之后每当两端都测准了就更新成
    **这个人自己的**长度（取中位数，抗个别坏帧）。同一个人腿长不变，
    测几次就稳了。
    """

    KEEP = 60        # 每条骨保留多少次可信测量
    # 反推点的时间一致性。比实测点卡得严（实测 6 m/s）：反推本来就是
    # 猜的，猜得前后不一致不如不给——11-11 实测左膝在两个解之间来回
    # 跳 78 cm，比不反推还难看。
    MAX_SPEED = float(os.environ.get("INFER_MAX_SPEED", "3.0"))
    SMOOTH = float(os.environ.get("INFER_SMOOTH", "0.5"))   # 新值权重
    STALE_S = float(os.environ.get("INFER_STALE_S", "1.0")) # 多久前的参考还算数
    # 子关节允许比父关节深多少。人正对相机站着时，膝在髋的前下方，
    # 四肢基本朝相机这一侧——所以「远解」（把膝放到髋后面半米）可以直接
    # 排掉。11-11 实测：三次大跳全是选了远解（推出 1.71 m，人其实在 0.7 m），
    # 而且选错后靠「和上一帧连得上」一直锁在错解上，直到实测点出现才跳回来。
    ZBACK_M = float(os.environ.get("INFER_ZBACK_M", "0.20"))

    def __init__(self):
        self.obs = {}                      # (i,j) -> deque[长度]
        # 自己上一帧交出去的点（含反推的）。只靠 tracker 给的 prev 不够：
        # tracker 只记实测点，关节连续几帧都靠推时 prev 是空的，
        # 每帧都退回「随便选一个解」，于是来回跳。
        self.last = {}                     # j -> (时刻, 三维点)

    def feed(self, pts, t=None):
        """两端都可信时，记下这段骨头的实测长度；顺带记住实测位置当参考。"""
        if t is not None:
            for j, p in pts.items():
                self.last[j] = (t, np.asarray(p, float))
        for ij in BONE_CHAIN:
            i, j = ij
            if i in pts and j in pts:
                L = float(np.linalg.norm(pts[i] - pts[j]))
                lo, hi = BONE_RANGE.get(ij, (0.05, 0.8))
                if lo <= L <= hi:          # 明显不合理的不要，免得污染先验
                    self.obs.setdefault(ij, deque(maxlen=self.KEEP)).append(L)

    def length(self, ij):
        """这段骨头多长。有本人实测就用实测的中位数，否则用人体中值。"""
        d = self.obs.get(ij)
        if d and len(d) >= 5:
            return float(np.median(d)), True
        lo, hi = BONE_RANGE.get(ij, (0.2, 0.5))
        return (lo + hi) / 2, False

    def infer(self, pts, pose_lms, intr, prev=None, t=None, chain=None):
        """把测不准的关节从父关节推出来。就地改 pts，返回推了哪些。

        顺着 BONE_CHAIN 从躯干往外推：髋推膝，膝再推踝。所以一条腿
        只要髋是准的，整条腿都能补回来。chain 给 BONE_CHAIN_UPPER 时只推手臂。
        """
        made = {}
        tc = [pts[j] for j in TORSO if j in pts]
        torso_c = np.mean(tc, axis=0) if tc else None
        for ij in (BONE_CHAIN if chain is None else chain):
            i, j = ij
            if i not in pts or j in pts:
                continue                   # 父不可信、或子本来就有，都不用推
            lm = pose_lms[j]
            if getattr(lm, "visibility", 1.0) < MIN_VIS:
                continue                   # 图像上都看不见，没法推
            u, v = lm.x * intr.width, lm.y * intr.height
            if not (0 <= u < intr.width and 0 <= v < intr.height):
                continue
            # 相机光心指向该像素的单位方向。用 deproject 拿方向，
            # 顺带把镜头畸变去掉——335L 的 k1/k2 不为零，不去畸变会偏。
            d = intr.deproject(u, v, 1.0)
            d = d / (np.linalg.norm(d) + 1e-12)

            P = pts[i]
            L, _measured = self.length(ij)
            # 解 |s·d - P| = L  →  s² - 2s(d·P) + |P|² - L² = 0
            b = float(d @ P)
            c = float(P @ P) - L * L
            disc = b * b - c
            # 选解的参考点：优先 tracker 给的实测历史，其次自己上一帧的输出
            ref = None
            if prev is not None and j in prev:
                ref = np.asarray(prev[j], float)
            elif j in self.last and (t is None
                                     or t - self.last[j][0] <= self.STALE_S):
                ref = self.last[j][1]

            if disc < 0:
                # 射线够不着那个球：像素位置和骨长对不上（深度偏、或骨长先验
                # 还没稳）。两个约束不能同时满足时**保骨长**——从父关节朝
                # 射线最近点的方向量出一个骨长。位置会偏离那个像素一点，
                # 但长度一定对，不会出现腿忽长忽短。
                v = b * d - P
                n = float(np.linalg.norm(v))
                if n < 1e-6:
                    continue
                q = P + L * v / n
            else:
                r = np.sqrt(disc)
                cand = [x for x in (b - r, b + r) if x > 1e-3]
                if not cand:
                    continue
                # 先用几何先验淘汰明显不对的解，再谈连续性。顺序很重要：
                # 反过来（先连续性）会把第一次的错解一路锁住。
                ok = [x for x in cand
                      if float((x * d)[2]) <= float(P[2]) + self.ZBACK_M]
                cand = ok or cand
                if ref is not None and len(cand) > 1:
                    s = min(cand, key=lambda x: float(np.linalg.norm(x * d - ref)))
                else:
                    s = min(cand)        # 近的那个：四肢在身体朝相机这一侧
                q = s * d

            # 离躯干太远的不给。实测点有这道（prune_flyaway），反推点
            # 原先绕过了——推出来的点飞到两米外照样会被当成判定依据。
            if torso_c is not None and \
                    float(np.linalg.norm(q - torso_c)) > MAX_BODY_R:
                continue

            # 时间一致性 + 平滑：反推点也要过这道，和实测点一个待遇。
            # 推出来的点跳得比人能走的快，就不给——缺一帧比给个乱跳的好。
            old = self.last.get(j)
            if old is not None and t is not None:
                dt = t - old[0]
                if 0 < dt <= self.STALE_S:
                    if float(np.linalg.norm(q - old[1])) / dt > self.MAX_SPEED:
                        continue
                    q = self.SMOOTH * q + (1 - self.SMOOTH) * old[1]
            pts[j] = q
            if t is not None:
                self.last[j] = (t, q)
            made[j] = ij
        return made


def _horiz(p, up):
    """点离相机的水平距离（去掉沿「上」方向的分量）。没有 up 时退回光轴深度 z。"""
    p = np.asarray(p, float)
    if up is None:
        return float(p[2])
    return float(np.linalg.norm(p - float(p @ up) * up))


def check_hips(torso, pose_lms, intr, up=None):
    """upper 模式下核验髋 23/24。就地改 torso，返回 (每个髋的结论, 被估计点换掉的原始髋点)。

    髋和腿一样常被桌子挡住：像素是对的（模型知道髋在图上哪儿），深度取到的
    却是挡在前面的桌面，三维点落到桌上。所以髋不能不加检查地进「躯干」。
    拿同侧肩当参照（缺了用对侧肩），两条都满足才算可信：
      ① 肩髋距离在人体范围内（同侧用 BONE_RANGE 的躯干侧边，对侧放宽）
      ② 髋在水平方向上不比肩离相机近 HIP_NEAR_M 以上——桌子在人和相机之间，
         落到桌上的髋一定偏近；人直立或前倾时髋不会比肩更靠近相机。
         up 是相机系的「朝上」向量（UP_CAM，来自实测地面），没有就退回比 z
    不可信的髋不直接扔：图像位置还是准的，就沿那个像素的视线放到和肩一样的
    水平距离上，当一个「估计的躯干下缘」用（显示上和骨架反推点一样画小一圈）。这样
    人只把下半身探进区域时，躯干判定仍然有点可用，不至于全靠两肩。
    两肩都没有时没法核验，髋照旧用——没有参照就不删，宁可多停。

    结论取值：可信 / 估计（深度不可信，按肩的距离估）/ 不用 / 未核验（没肩可比，照旧用）
    """
    res, dropped = {}, []
    for h, s_same, s_other in ((23, 11, 12), (24, 12, 11)):
        if s_same in torso:
            ref, rng = torso[s_same], BONE_RANGE[(s_same, h)]
        elif s_other in torso:
            ref, rng = torso[s_other], HIP_SHOULDER_RANGE
        else:
            if h in torso:
                res[h] = "未核验"
            continue
        if h not in torso:
            continue                      # 本来就没测到：和 full 一样当没有，不额外估
        p = torso[h]
        L = float(np.linalg.norm(p - ref))
        if rng[0] <= L <= rng[1] and _horiz(p, up) >= _horiz(ref, up) - HIP_NEAR_M:
            res[h] = "可信"
            continue
        torso.pop(h)
        lm = pose_lms[h]
        u, v = lm.x * intr.width, lm.y * intr.height
        if 0 <= u < intr.width and 0 <= v < intr.height:
            d = intr.deproject(u, v, 1.0)          # 视线方向（已去畸变）
            hd = _horiz(d, up)
            q = d * (_horiz(ref, up) / hd) if hd > 1e-6 else None
            if q is not None and rng[0] <= float(np.linalg.norm(q - ref)) <= rng[1]:
                torso[h] = q
                res[h] = "估计"
                dropped.append(p)         # 原始点留给显示端画成飞点
                continue
        res[h] = "不用"
    return res, dropped


def prune_by_bones(pts, base_c, rounds=2):
    """按骨头长度丢飞点。pts 是 {序号: 三维点}，就地过滤，返回丢了几个。

    判据：一条骨的两端都在，但长度超出人体范围，就说明至少有一端是飞的。
    丢**离躯干更远**的那一端——躯干点基本不飞（它在画面正中），
    飞的几乎总是外推出去的肢体末端。

    迭代两轮：踝飞了会先被「小腿太长」抓住，丢掉踝之后，
    如果膝也是飞的，第二轮「大腿太长」才看得出来。
    """
    dropped = 0
    for _ in range(rounds):
        bad = set()
        for (i, j), (lo, hi) in BONE_RANGE.items():
            if i not in pts or j not in pts:
                continue
            L = float(np.linalg.norm(pts[i] - pts[j]))
            if lo <= L <= hi:
                continue
            di = float(np.linalg.norm(pts[i] - base_c))
            dj = float(np.linalg.norm(pts[j] - base_c))
            bad.add(i if di > dj else j)
        if not bad:
            break
        for k in bad:
            pts.pop(k, None)
            dropped += 1
    return dropped


def prune_flyaway(groups, hand_pts):
    """丢掉离躯干太远的点。

    groups 是 {名字: {序号: 三维点}}，hand_pts 是手部模型给的点列表。
    躯干自己当基准：它一般在画面正中，最不容易飞。躯干也拿不到时退而用
    全部点的中位——那时基准本身就不太可靠，所以只能滤掉最离谱的。

    返回 (过滤后的 groups, 过滤后的 hand_pts, 丢了几个点)。
    """
    torso = list(groups.get("torso", {}).values())
    allp = [p for g in groups.values() for p in g.values()] + list(hand_pts)
    base = torso if torso else allp
    if len(base) < 2:
        return groups, hand_pts, 0

    c = np.median(np.array(base), axis=0)
    dropped = 0
    out = {}
    for name, g in groups.items():
        keep = {}
        for i, p in g.items():
            if float(np.linalg.norm(p - c)) <= MAX_BODY_R:
                keep[i] = p
            else:
                dropped += 1
        out[name] = keep

    # 第二道：按骨头长度查。半径检查只能抓飞得很远的，
    # 飞到近处桌面上的膝盖离躯干可能才 0.8 m，得靠骨长才抓得住。
    merged = {}
    for g in out.values():
        merged.update(g)
    dropped += prune_by_bones(merged, c)
    for name in out:
        out[name] = {i: p for i, p in out[name].items() if i in merged}
    hands_keep = []
    for p in hand_pts:
        if float(np.linalg.norm(p - c)) <= MAX_BODY_R:
            hands_keep.append(p)
        else:
            dropped += 1
    return out, hands_keep, dropped


def draw_zone(vis, ground, intr, col):
    """把区域柱体画到画面上：底面多边形 + 四条竖边 + 顶面。

    不画出来就没法判断「判定对不对」——人站在边上，到底算里还是算外，
    光看状态文字是猜不出来的。
    """
    def proj(p):
        q = intr.project(p)
        return None if q is None else (int(round(q[0])), int(round(q[1])))

    # 区域的「上」是法向，但法向可能朝相机也可能背着，统一取朝上那一侧：
    # 让它和重力反向（重力在相机系里大致是 +Y 朝下）。
    n = ground.normal
    up = n if float(n[1]) < 0 else -n

    base = [proj(p) for p in ground.corners3d]
    top = [proj(p + up * ZONE_H) for p in ground.corners3d]

    def poly(pts, color, th):
        ok = [q for q in pts if q is not None]
        if len(ok) >= 2:
            cv2.polylines(vis, [np.array(ok, np.int32)], len(ok) == len(pts),
                          color, th, cv2.LINE_AA)

    poly(base, col, 2)
    poly(top, (160, 160, 160), 1)
    for a, b in zip(base, top):
        if a and b:
            cv2.line(vis, a, b, (160, 160, 160), 1, cv2.LINE_AA)

    # 标上边长，省得去翻日志
    if base[0] and base[1]:
        side = float(np.linalg.norm(ground.corners3d[0] - ground.corners3d[1]))
        mx = ((base[0][0] + base[1][0]) // 2, (base[0][1] + base[1][1]) // 2)
        cv2.putText(vis, f"{side:.2f}m", mx, cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, col, 2, cv2.LINE_AA)


def in_zone(p, ground):
    """点在不在区域柱体里：底面多边形内 + 高度在 0~ZONE_H 之间。"""
    return ground.inside(p) and ground.height(p) <= ZONE_H


# 关键点落在机械臂表面多近，就认为「看到的是机械臂不是人」（米）。
# 机械臂自己会挡住视野，被挡住的地方深度取到的是臂的表面，
# 于是关键点贴在臂上。那不是「人在那儿」，也不是「那儿没人」，
# 是「那儿看不见」——两者在安全上的含义完全不同。
ARM_SURFACE_M = float(os.environ.get("ARM_SURFACE_M", "0.08"))
# 被机械臂挡掉的关键点占比超过这个，就报「看不清」
ARM_BLOCK_RATIO = float(os.environ.get("ARM_BLOCK_RATIO", "0.5"))


def split_by_arm(pts, capsules):
    """按「是不是贴在机械臂表面」把点分成两堆。

    返回 (人身上的点, 疑似落在机械臂上的点)。
    判据是点到机械臂包络表面的距离：贴得太近（含负数，即在包络内）
    就当成取到了臂的表面。
    """
    if not capsules or not pts:
        return pts, {}
    body, on_arm = {}, {}
    for i, p in pts.items():
        d = np.inf
        for a, b, r in capsules:
            ab = b - a
            L2 = float(ab @ ab) + 1e-12
            tt = min(1.0, max(0.0, float((p - a) @ ab / L2)))
            d = min(d, float(np.linalg.norm(p - (a + tt * ab))) - r)
        (on_arm if d < ARM_SURFACE_M else body)[i] = p
    return body, on_arm


def judge_multi(poses, hands, depth_m, intr, ground, ptrack=None, t=0.0,
                quality=None, arm_caps=None, parts=None):
    """多人版判定。对每个人单独判，取**最危险**的那个当结论。

    为什么取最危险而不是别的：安全判断只关心「要不要停」。三个人在
    画面里，一个踩进区域就得停，另外两个在不在旁边不改变这个结论。

    手怎么分给人：手部模型独立检测，不知道这只手是谁的。按三维位置
    就近归属——手离谁的躯干近就算谁的。人挨得很近时会分错，但那种
    情况下两人的判定结果通常一样，分错不影响结论。
    """
    # 画面不可信这一道在最前面，和有几个人无关
    if quality is not None and quality[0]:
        info = {"edge_m": None, "quality": quality[1],
                "blind_s": round(float(quality[2]), 2), "n_persons": 0}
        return "看不清", f"{quality[1]}（已持续 {quality[2]:.1f} 秒）", info

    poses = poses or []
    if not poses:
        return "空闲", "画面里没人", {"edge_m": None, "n_persons": 0}

    # 每个人的画面中心（归一化坐标即可，只用来做帧间关联）
    # upper 下只用髋及以上的点：出画面的腿给的是外推坐标，会把中心往下拽。
    upper = (parts or BODY_PARTS) == "upper"
    centers = []
    for lms in poses:
        good = [(lm.x, lm.y) for j, lm in enumerate(lms)
                if getattr(lm, "visibility", 1.0) >= MIN_VIS
                and not (upper and j in LOWER_LEG)]
        centers.append(np.median(np.array(good), axis=0) if good
                       else np.array([0.5, 0.5]))
    ids = ptrack.assign(centers, t) if ptrack is not None else [None] * len(poses)

    # 手按最近的人分组
    hand_groups = [[] for _ in poses]
    for hand in (hands or []):
        hc = np.median(np.array([(lm.x, lm.y) for lm in hand]), axis=0)
        k = int(np.argmin([float(np.linalg.norm(hc - c)) for c in centers]))
        hand_groups[k].append(hand)

    best = None
    per_person = []
    for i, lms in enumerate(poses):
        g, jt, pr = (ptrack.state_of(ids[i]) if ptrack is not None
                     else (None, None, None))
        st, why, inf = judge(lms, hand_groups[i], depth_m, intr, ground,
                             g, t, None, jt, arm_caps, pr, parts)
        inf["person"] = ids[i]
        per_person.append((st, why, inf))
        if best is None or ORDER.index(st) > ORDER.index(best[0]):
            best = (st, why, inf)

    st, why, inf = best
    inf = dict(inf)
    inf["n_persons"] = len(poses)
    # 按 poses 的顺序给出每个人的完整中间量，显示端要画所有人
    inf["each"] = [p[2] for p in per_person]
    inf["persons"] = [{"id": p[2].get("person"), "state": p[0],
                       "edge_m": p[2].get("edge_m")} for p in per_person]
    if len(poses) > 1:
        why = f"{why}（画面里 {len(poses)} 个人，报最危险的那个）"
    return st, why, inf


def judge(pose_lms, hands, depth_m, intr, ground, gate=None, t=0.0,
          quality=None, tracker=None, arm_caps=None, prior=None, parts=None):
    """一帧的原始判断。返回 (档位, 给人看的理由, 详细数值)。

    判的是「人体有没有占住那块立体空间」，不是「脚踩在哪」。
    理由：相机架得低或离得近时腿必然出画面，靠脚判等于什么都判不了
    （11-05 实测，98.7% 的帧报看不清）。而安全上真正要避的是人的身体，
    不是人的脚印——躯干进了危险区，哪怕脚还在外面，也得停。
    """
    info = {"edge_m": None, "torso_pts": 0, "limb_pts": 0,
            "head_pts": 0, "hand_pts": 0}
    # 部位开关按调用传入（同一帧跑两套做对照用），不传就用 BODY_PARTS
    parts = parts or BODY_PARTS
    upper = parts == "upper"
    limb_idx = LIMB_UPPER if upper else LIMB_FULL
    info["parts"] = parts

    # 画面不可信就「看不清」，不管有没有检到人。这一道必须在最前面：
    # 镜头被挡住时人体检测自然什么都检不到，走到下面就成了「没人」。
    #
    # quality 现在是 (是否报看不清, 原因, 已持续秒数)——是否报由 QualityGate
    # 按持续时间决定，不是一帧说了算。闪一下不报，持续才报。
    if quality is not None and quality[0]:
        if gate is not None:
            gate.feed([], t)
        info["quality"] = quality[1]
        info["blind_s"] = round(float(quality[2]), 2)
        return "看不清", f"{quality[1]}（已持续 {quality[2]:.1f} 秒）", info

    if pose_lms is None:
        if gate is not None:
            gate.feed([], t)
        return "空闲", "画面里没人", info

    # upper 下膝及以下既不进分子也不进分母：只数 0~24，门槛按比例缩到 9
    n_good = sum(1 for j, lm in enumerate(pose_lms)
                 if getattr(lm, "visibility", 1.0) >= MIN_VIS
                 and not (upper and j in LOWER_LEG))
    min_good = MIN_GOOD_LMS_UPPER if upper else MIN_GOOD_LMS
    info["good_lms"] = n_good
    if n_good < min_good:
        return "空闲", f"画面里没人（只有 {n_good} 个关键点够清楚，不足 {min_good}）", info

    torso = collect_3d(pose_lms, TORSO, depth_m, intr)
    limb = collect_3d(pose_lms, limb_idx, depth_m, intr)
    head = collect_3d(pose_lms, HEAD, depth_m, intr)

    # 手部模型给的 21 点比姿态模型的腕部准，单独收
    hand_pts = []
    for hand in (hands or []):
        for lm in hand:
            px, py = int(lm.x * depth_m.shape[1]), int(lm.y * depth_m.shape[0])
            d = depth_at(depth_m, px, py)
            if d is not None and d > 0:
                hand_pts.append(to_3d(px, py, d, intr))
    # 时间一致性：先按「人不会瞬移」把跳变的点拒掉，再做空间检查。
    # 顺序不能反——空间检查需要一组互相自洽的点，先把时间上的离群点
    # 剔掉，骨长判断才准。
    n_jump = 0
    if tracker is not None:
        merged = {}
        merged.update({("t", i): p for i, p in torso.items()})
        merged.update({("l", i): p for i, p in limb.items()})
        merged.update({("h", i): p for i, p in head.items()})
        flat = {i: p for (_, i), p in merged.items()}
        flat, n_jump = tracker.feed(flat, t)
        torso = {i: flat[i] for i in torso if i in flat}
        limb = {i: flat[i] for i in limb if i in flat}
        head = {i: flat[i] for i in head if i in flat}

    # 机械臂自遮挡：臂挡在人前面时，关键点的深度取到的是臂的表面，
    # 点就贴在臂上。那些点既不能当「人在那儿」（位置是错的），
    # 也不能当「那儿没人」（人可能就在臂后面）——只能当「看不见」。
    n_arm = 0
    if arm_caps:
        for name, g in (("torso", torso), ("limb", limb), ("head", head)):
            keep, on_arm = split_by_arm(g, arm_caps)
            n_arm += len(on_arm)
            g.clear(); g.update(keep)
        hb, ha = split_by_arm({k: v for k, v in enumerate(hand_pts)}, arm_caps)
        n_arm += len(ha)
        hand_pts = list(hb.values())

    # 空间检查：丢掉离躯干太远、或骨头长度不合理的点。
    raw_before = {k: dict(v) for k, v in
                  {"torso": torso, "limb": limb, "head": head}.items()}
    # upper：髋先过可信检查（落到桌面上的换成按肩深度估的点）。放在飞点检查
    # 之前，因为飞点检查拿躯干当基准，落在桌上的髋会把基准拽偏。
    hip_dropped = []
    if upper:
        n = getattr(ground, "normal", None)     # 多边形区域自带地面法向
        up = UP_CAM if n is None else (np.asarray(n, float) if n[1] < 0
                                       else -np.asarray(n, float))
        info["hips"], hip_dropped = check_hips(torso, pose_lms, intr, up)
    groups, hand_pts, n_drop = prune_flyaway(
        {"torso": torso, "limb": limb, "head": head}, hand_pts)
    torso, limb, head = groups["torso"], groups["limb"], groups["head"]

    # 把被丢掉、但图像上看得见的关节从父关节反推回来。
    # 这一步在丢弃之后做：先把错的清掉，再用骨架把缺的补上。
    n_infer = 0
    if prior is not None:
        merged = {}
        merged.update(torso); merged.update(limb); merged.update(head)
        prior.feed(merged, t)               # 先拿这一帧可信的更新骨长先验
        prev = dict(tracker.last and {k: v[1] for k, v in tracker.last.items()}) \
            if tracker is not None else None
        made = prior.infer(merged, pose_lms, intr, prev, t,
                           BONE_CHAIN_UPPER if upper else None)
        n_infer = len(made)
        info["inferred"] = set(made)
        for j in made:
            (limb if j in limb_idx else head if j in HEAD else torso)[j] = merged[j]
        groups = {"torso": torso, "limb": limb, "head": head}
    torso, limb, head = groups["torso"], groups["limb"], groups["head"]
    info["torso_pts"], info["limb_pts"], info["head_pts"] = \
        len(torso), len(limb), len(head)
    info["hand_pts"] = len(hand_pts)
    info["dropped_pts"] = n_drop
    info["jumped_pts"] = n_jump
    info["on_arm_pts"] = n_arm
    info["inferred_pts"] = n_infer
    # 把判定实际用的那份点原样交出去，显示端直接拿这个画。
    # 两边各算一遍的话，演示时看到的点和停机依据的点会对不上。
    info["pts3d"] = {**torso, **limb, **head}
    info.setdefault("inferred", set())
    # 按肩深度估出来的髋也算「推出来的」，显示上画小一圈
    info["inferred"] |= {h for h, r in (info.get("hips") or {}).items()
                         if r == "估计" and h in torso}
    info["flyaway"] = [p for k, g in raw_before.items()
                       for i, p in g.items() if i not in groups[k]] + hip_dropped

    allp = list(torso.values()) + list(limb.values()) + list(head.values()) + hand_pts

    # 机械臂挡掉了大半个人：不知道他在哪，按最坏算。
    # 这跟「画面里没人」是两回事——没人是看清楚了确实没有，
    # 这里是看见了人但看不清他在哪。
    if n_arm and n_arm >= ARM_BLOCK_RATIO * (n_arm + len(allp)):
        return "看不清", (f"人被机械臂挡住了"
                          f"（{n_arm} 个点落在臂上，只剩 {len(allp)} 个可用）"), info

    # 动过才算人。静止的椅子和衣服到这一步会被挡掉。
    if gate is not None:
        is_person, moved = gate.feed(allp, t)
        info["moved_m"] = round(moved, 3)
        if not is_person:
            return "空闲", (f"检到像人的东西，但它从出现到现在没动过"
                            f"（{gate.WIN_S:.1f} 秒内只动了 {moved*100:.1f} cm）"), info

    if not allp:
        # 检到人，却一个三维点都拿不到：被挡住、超出量程、或深度全是空洞。
        # 不知道人在哪就按最坏算——这就是「看不清」。
        return "看不清", "检到人，但身上一个点都测不出三维位置", info

    # dist_to_edge 的约定是**区内为正**。所以要的是 max——最深入区域的那个点，
    # 不是 min（min 会挑出离区域最远的点，判断就反了）。
    #   edge_m > 0  在区域里，数值是进去多深
    #   edge_m < 0  在区域外，数值是离边界多远（取绝对值）
    ds = [ground.dist_to_edge(p) for p in allp]
    k = int(np.argmax(ds))
    edge_m = ds[k]
    info["edge_m"] = round(edge_m, 3)
    # 最深入（或最靠近）区域的那个点，相机系。上层拿它换算到机器人基座系，
    # 就能回答「人身上离机械臂最近的地方在哪」。
    info["nearest_cam"] = [round(float(x), 3) for x in allp[k]]

    if any(in_zone(p, ground) for p in torso.values()):
        return "占用", f"人在区域里（躯干 {len(torso)} 个点）", info

    limb_in = [i for i, p in limb.items() if in_zone(p, ground)]
    hand_in = sum(1 for p in hand_pts if in_zone(p, ground))
    head_in = any(in_zone(p, ground) for p in head.values())

    if head_in:
        # 头进去了躯干还没进 —— 探身、弯腰。按侵入算。
        return "手伸入", "人在区外，头探进区域了", info
    if limb_in or hand_in:
        return "手伸入", f"人在区外，伸进去了（肢体 {len(limb_in)} 点、手 {hand_in} 点）", info

    if edge_m >= -NEAR_M:
        return "旁边有人", f"人在旁边，离边界 {-edge_m:.2f} m", info
    return "空闲", f"人离得远（{-edge_m:.2f} m）", info


# 最多同时跟几个人。安全场景里不是越多越好：模型要为每个人跑一遍
# 后处理，帧率会掉；而且判定只取「最危险的那个人」，第 4 个人在不在
# 画面里不改变结论。3 个够用，现场人多再调。
MAX_PERSONS = int(os.environ.get("MAX_PERSONS", "3"))
PRIOR_ON = os.environ.get("PRIOR", "1") == "1"


# 姿态后端：mp = MediaPipe（CPU，默认）；gpu = gpu_pose.py 里的 YOLOX + RTMW 全身模型
# （onnxruntime + TensorRT，跑在 Jetson GPU 上）。gpu 后端的 pose 和 hands 共用一个模型，
# 同一帧只推理一次；输出和 MediaPipe 同形状，下面的判定代码不区分后端。
POSE_BACKEND = os.environ.get("POSE_BACKEND", "mp")


def make_pose():
    if POSE_BACKEND == "gpu":
        from gpu_pose import make_gpu_pose
        return make_gpu_pose(MAX_PERSONS)
    return PoseLandmarker.create_from_options(PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=POSE_MODEL),
        running_mode=RunningMode.VIDEO, num_poses=MAX_PERSONS,
        min_pose_detection_confidence=POSE_CONF,
        min_pose_presence_confidence=POSE_CONF,
        min_tracking_confidence=POSE_CONF))


def make_hands():
    if POSE_BACKEND == "gpu":
        from gpu_pose import make_gpu_hands
        return make_gpu_hands(MAX_PERSONS)
    return HandLandmarker.create_from_options(HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=HAND_MODEL),
        running_mode=RunningMode.VIDEO, num_hands=2,
        min_hand_detection_confidence=0.5, min_tracking_confidence=0.5))


def do_calib(args):
    """站四个角，各按一次回车。记的是两脚中点。"""
    cam = open_camera(getattr(args, "backend", None), W, H, FPS)
    intr = cam.intr
    pose = make_pose()
    # 重力方向：脚看不见时要靠它把躯干下缘压到地面高度
    g_cal, _gi = cam.read_gravity(2.0)
    if g_cal is not None:
        g_cal = np.asarray(g_cal, float) / np.linalg.norm(g_cal)
    corners, t0 = [], time.time()
    state_used = ["脚"]
    names = ["左前", "右前", "右后", "左后"]
    print("\n标定操作区。按顺时针（或逆时针，别交叉）站到四个角上，")
    print("（拍得到鞋就用鞋定位；拍不到会退到躯干下缘估，精度差几厘米。）")
    print("每站好一个角，在这个终端按一次回车。按 q 回车放弃。\n")
    try:
        while len(corners) < 4:
            print(f">>> 站到第 {len(corners)+1} 个角（{names[len(corners)]}），"
                  f"站稳后按回车：", end="", flush=True)
            # 一边等输入一边刷相机，免得缓冲区积压老帧
            import select
            pts = deque(maxlen=15)
            for fr in cam.frames():
                depth, color = fr.depth, fr.color
                if True:
                    rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
                    res = pose.detect_for_video(
                        mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb),
                        int((time.time() - t0) * 1000))
                    if res.pose_landmarks:
                        lms = res.pose_landmarks[0]
                        # 优先用脚：人站在角上，脚的位置就是角的位置。
                        # 看不到脚就退到躯干下缘（髋、膝），把它沿重力压到
                        # 地面高度——判定逻辑已经不依赖脚了，标定也不该依赖。
                        got, used = [], "脚"
                        for idx in (ANKLE_L, ANKLE_R, FOOT_L, FOOT_R):
                            lm = lms[idx]
                            if getattr(lm, "visibility", 1.0) < MIN_VIS:
                                continue
                            px, py = int(lm.x * depth.shape[1]), int(lm.y * depth.shape[0])
                            dd = depth_at(depth, px, py)
                            if dd:
                                got.append(to_3d(px, py, dd, intr))
                        if not got and g_cal is not None:
                            used = "躯干下缘（沿重力下压，精度差些）"
                            for idx in (KNEE_L, KNEE_R, HIP_L, HIP_R):
                                lm = lms[idx]
                                if getattr(lm, "visibility", 1.0) < MIN_VIS:
                                    continue
                                px, py = int(lm.x * depth.shape[1]), int(lm.y * depth.shape[0])
                                dd = depth_at(depth, px, py)
                                if dd:
                                    got.append(to_3d(px, py, dd, intr))
                            if got:
                                # 髋约在身高的 0.52、膝约 0.28。取 0.40 当平均，
                                # 沿重力方向推到地面。个体差异带来几厘米误差，
                                # 比完全标不了强，但能用脚就别用这条。
                                got = [p + g_cal * 0.40 for p in got]
                        state_used[0] = used
                        if got:
                            pts.append(np.mean(got, axis=0))
                if select.select([sys.stdin], [], [], 0)[0]:
                    line = sys.stdin.readline().strip().lower()
                    if line == "q":
                        print("放弃。"); return 1
                    break
            if not pts:
                print("  ✗ 这个位置既看不到脚也看不到髋膝，往后站一点再按。")
                continue
            # 取最近 15 帧的中位，单帧噪声能有几厘米
            p = np.median(np.array(pts), axis=0)
            corners.append(p)
            print(f"  ✓ 记下 ({p[0]:+.2f}, {p[1]:+.2f}, {p[2]:+.2f}) m "
                  f"（{len(pts)} 帧中位，用的是{state_used[0]}）")
    finally:
        cam.close(); pose.close()

    g = Ground(corners)
    side = [float(np.linalg.norm(g.poly2d[i] - g.poly2d[(i+1) % 4])) for i in range(4)]
    g.save(ZONE_FILE)
    print(f"\n存到 {ZONE_FILE}")
    print(f"四条边长：{' / '.join(f'{s:.2f}' for s in side)} m")
    print("边长明显不对就重标一次——多半是某个角站的位置脚被挡住了。")
    return 0


def do_auto_calib(args):
    """不用人站位：拟合视野里最大的那个平面，取它中心一块正方形当操作区。

    为什么能这么干：操作区本来就在一个平面上（地面或工作台面），而那个平面
    通常是画面里最大的连续平面。RANSAC 找出来之后，区域边界纯粹是在平面内
    画出来的，不读任何一个像素的深度——所以边界不会被深度空洞和飞点带歪。

    代价是区域位置由相机视角决定，不是人指定的。要精确对齐现场贴的胶带，
    还得用 --calib 或 table_zones.py --pick。
    """
    side = float(args.auto_calib)
    cam = open_camera(getattr(args, "backend", None), W, H, FPS)
    try:
        # 多帧取中位：单帧深度的空洞和飞点在中位里会被压掉。
        acc = []
        for fr in cam.frames():
            acc.append(fr.depth)
            if len(acc) >= 20:
                break
        dm = np.median(np.stack(acc), axis=0)
        # 顺便读一下重力。有了它才能分清「地面」和「墙」——两者都是大平面，
        # 只看哪个点多会挑错：11-11 实测，相机俯角 14° 时墙面点比地面多，
        # RANSAC 挑中了正前方 1.75 m 处的竖直面，区域就划到墙上去了。
        g_vec, g_info = cam.read_gravity(2.0)
    finally:
        cam.close()

    intr = cam.intr
    ys, xs = np.mgrid[0:intr.height:4, 0:intr.width:4]      # 抽稀，全分辨率没必要
    zs = dm[::4, ::4]
    m = (zs > 0.15) & (zs < 4.0)
    if m.sum() < 500:
        sys.exit("有效深度点太少，拟合不了平面。检查相机对着的地方是不是太近或太远。")
    pts = intr.deproject_many(np.stack([xs[m], ys[m]], axis=1), zs[m])

    if g_vec is not None and not getattr(args, "any_plane", False):
        g = np.asarray(g_vec, float)
        g /= np.linalg.norm(g)
        # 沿重力方向的坐标 ≈ 高度（符号相反）。同一个水平面上的点，这个值几乎一样，
        # 所以直方图上会堆成一个尖峰；墙面的点在这个方向上是摊开的，堆不出峰。
        h = pts @ g
        hist, edges = np.histogram(h, bins=np.arange(h.min(), h.max() + 0.02, 0.02))
        k = int(np.argmax(hist))
        h0 = (edges[k] + edges[k + 1]) / 2
        band = np.abs(h - h0) < 0.06
        print(f"重力筛选：最密的一层在沿重力 {h0:+.2f} m 处，"
              f"{band.sum()}/{len(pts)} 个点（峰值 {hist[k]} 点/2cm）")
        if band.sum() >= 400:
            pts = pts[band]
        else:
            print("  ⚠ 这一层点太少，退回用全部点拟合")

    c, n, n_in = fit_plane_ransac(pts)
    print(f"主平面：内点 {n_in}/{len(pts)}，法向 ({n[0]:+.3f}, {n[1]:+.3f}, {n[2]:+.3f})")
    if n_in < 300:
        sys.exit("内点太少，画面里没有明显的大平面。")

    if g_vec is not None:
        g = np.asarray(g_vec, float) / np.linalg.norm(g_vec)
        tilt = math.degrees(math.acos(min(1.0, abs(float(np.dot(n, g))))))
        print(f"这个平面和水平面差 {tilt:.1f}°", end="")
        if tilt > 30:
            print("  ⚠ 超过 30°，这多半是面墙而不是地面/台面。")
            print("     要么调相机让地面多露一些，要么加 --any-plane 强行用它。")
            if not getattr(args, "any_plane", False):
                sys.exit("没存。")
        else:
            print("  ✓ 是水平面")

    # 平面内建两个正交轴，以内点质心为中心画正方形
    a = np.array([1.0, 0, 0])
    if abs(float(np.dot(a, n))) > 0.9:
        a = np.array([0, 1.0, 0])
    ex = np.cross(n, a); ex /= np.linalg.norm(ex)
    ey = np.cross(n, ex)
    d = (pts - c) @ n
    ctr = pts[np.abs(d) < 0.015].mean(axis=0)
    h2 = side / 2.0
    corners = [ctr - h2*ex - h2*ey, ctr + h2*ex - h2*ey,
               ctr + h2*ex + h2*ey, ctr - h2*ex + h2*ey]

    g = Ground(corners)
    g.save(ZONE_FILE)
    print(f"\n存到 {ZONE_FILE}")
    print(f"区域中心 ({ctr[0]:+.2f}, {ctr[1]:+.2f}, {ctr[2]:+.2f}) m，"
          f"边长 {side:.2f} m（自动划的正方形）")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", action="store_true", help="标定地面区域（人站四个角）")
    ap.add_argument("--any-plane", action="store_true",
                    help="自动划区时不要求是水平面（默认只认地面/台面）")
    ap.add_argument("--auto-calib", type=float, default=0.0, metavar="边长",
                    help="自动标定：拟合主平面，取平面中心这么大一块正方形当操作区（米）")
    ap.add_argument("--show", action="store_true", help="开实时窗口")
    ap.add_argument("--record", default="", help="录像和日志存到这个目录")
    ap.add_argument("--seconds", type=float, default=0, help="跑多少秒后自动停")
    ap.add_argument("--backend", default=None, choices=["orbbec", "realsense"])
    args = ap.parse_args()

    if args.calib:
        return do_calib(args)
    if args.auto_calib > 0:
        return do_auto_calib(args)

    if not os.path.isfile(ZONE_FILE):
        sys.exit(f"还没标定过。要人站四个角：python3 {sys.argv[0]} --calib\n"
                 f"不想站：python3 {sys.argv[0]} --auto-calib 0.5")
    ground = Ground.load(ZONE_FILE)
    print(f"判定用的身体部位：BODY_PARTS={BODY_PARTS}"
          f"（upper=只用髋及以上，full=含腿脚）")

    cam = open_camera(getattr(args, "backend", None), W, H, FPS)
    intr = cam.intr
    pose, hands = make_pose(), make_hands()
    grader, qgate, ptrack = Grader(), QualityGate(), PersonTracker()
    writer, log = None, []
    if args.record:
        os.makedirs(args.record, exist_ok=True)
        writer = cv2.VideoWriter(os.path.join(args.record, "monitor.mp4"),
                                 cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

    COLOR = {"空闲": (90, 200, 90), "旁边有人": (90, 200, 200),
             "看不清": (200, 120, 255), "手伸入": (60, 140, 255),
             "占用": (60, 60, 230)}
    t0, lat, last_seen, last_state = time.time(), deque(maxlen=60), -1e9, "空闲"
    try:
        for fr in cam.frames():
            t = time.time() - t0
            if args.seconds and t > args.seconds:
                break
            depth, color = fr.depth, fr.color
            rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
            img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

            ti = time.time()
            pr = pose.detect_for_video(img, int(t * 1000))
            hr = hands.detect_for_video(img, int(t * 1000))
            lat.append((time.time() - ti) * 1000)

            poses = pr.pose_landmarks if pr.pose_landmarks else []
            lms = poses[0] if poses else None
            hl = hr.hand_landmarks if hr.hand_landmarks else []
            q_ok, q_why = frame_quality(depth, color)
            blind, blind_s = qgate.feed(q_ok, t)
            raw, why, info = judge_multi(poses, hl, depth, intr, ground,
                                         ptrack, t, (blind, q_why, blind_s))

            # 人刚刚还在、这一帧突然没了：多半是被挡住或转身了，不是真走了。
            # 这段时间里保持原判，别急着放行。
            if lms is not None:
                last_seen = t
            elif raw == "空闲" and t - last_seen < VANISH_HOLD_S:
                raw, why = "看不清", f"人突然检不到了（{t - last_seen:.1f} s）"

            state = grader.feed(raw, t)
            risk = RISK[state]
            log.append({"t": round(t, 2), "raw": raw, "state": state,
                        "risk": risk, "why": why, **info})
            if state != last_state:
                print(f"[{t:6.1f}s] {last_state} → {state}   risk={risk}   {why}",
                      flush=True)
                last_state = state

            if args.show or writer is not None:
                vis = color.copy()
                col = COLOR[state]
                draw_zone(vis, ground, intr, col)
                cv2.rectangle(vis, (0, 0), (W, 34), col, -1)
                cv2.putText(vis, f"{state}  risk={risk}  "
                            f"{1000/max(np.mean(lat),1e-6):.0f}FPS",
                            (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 2)
                if lms:
                    # 画出判定真正用到的点：躯干画大圈（它决定「占用」），
                    # 四肢和头画小圈（它们只决定「伸入」）。
                    for idx in TORSO:
                        lm = lms[idx]
                        if getattr(lm, "visibility", 1.0) >= MIN_VIS:
                            cv2.circle(vis, (int(lm.x*vis.shape[1]), int(lm.y*vis.shape[0])), 7, col, -1)
                    for idx in LIMB + HEAD:
                        lm = lms[idx]
                        if getattr(lm, "visibility", 1.0) >= MIN_VIS:
                            cv2.circle(vis, (int(lm.x*vis.shape[1]), int(lm.y*vis.shape[0])), 3, col, 1)
                for hand in hl:
                    for lm in hand:
                        cv2.circle(vis, (int(lm.x*vis.shape[1]), int(lm.y*vis.shape[0])), 2, (0, 200, 255), -1)
                if writer is not None:
                    writer.write(vis)
                if args.show:
                    cv2.imshow("zone_monitor", vis)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
    finally:
        cam.close(); pose.close(); hands.close()
        if writer is not None:
            writer.release()
        if args.show:
            cv2.destroyAllWindows()

    if args.record:
        with open(os.path.join(args.record, "log.json"), "w") as f:
            json.dump(log, f, ensure_ascii=False)
    if log:
        n = len(log)
        print(f"\n{'='*58}\n{n} 帧，{log[-1]['t']:.1f} 秒，"
              f"{1000/max(np.mean(lat),1e-6):.1f} FPS")
        for s in ORDER:
            k = sum(1 for r in log if r["state"] == s)
            if k:
                print(f"  {s:<8} {k:5d} 帧  {k/n*100:5.1f}%")
        k = sum(1 for r in log if r["risk"] == 2)
        print(f"  ──────\n  risk=2（区域被占）{k} 帧  {k/n*100:.1f}%")
        print("  risk 只是判断结果，怎么响应由执行侧决定。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
