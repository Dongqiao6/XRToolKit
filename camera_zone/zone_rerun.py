#!/usr/bin/env python3
"""在 rerun 里三维看工位区域和人的关系。

为什么要有这个：二维画面上画三维柱体，透视一叠就看不懂了——人到底在
区域里还是在它前面，图上分不出来。放进三维场景就一目了然：可以转着看、
可以从正上方俯视看投影关系。

显示的东西：

    灰点云      深度图抽稀后的场景
    绿/黄/红框  工位区域柱体，颜色跟着判定状态走
    骨架        人体关键点连线。进区域的点标红，区外的标白
    红点        离区域最近（或最深入）的那个点
    面板        当前档位、理由、离边界多远

跑：

    ./rosrun.sh zone_rerun.py            # 不发 ROS 话题也能跑，壳只是为了环境一致
    ./.venv/bin/python zone_rerun.py     # 直接跑也行
    ./.venv/bin/python zone_rerun.py --save out/zone.rrd    # 存成文件回头看

坐标系：rerun 里用的是相机光学系（RDF：+X 右 / +Y 下 / +Z 前），
和代码里一直用的一致，不做任何转换——转换多一道就多一处可能搞反。
"""
import argparse
import os
import sys
import traceback
import time

import cv2
import mediapipe as mp
import numpy as np
import rerun as rr

SHOW_ENVELOPE = os.environ.get("SHOW_ENVELOPE", "0") == "1"   # 画不画机械臂包络的蓝色中轴线
MAX_ERR_FRAMES = int(os.environ.get("MAX_ERR_FRAMES", "30"))   # 连续多少帧检测出错就退出（约 1 秒）
PERF_EVERY = float(os.environ.get("PERF_EVERY", "5"))   # 每几秒往日志打一行逐段计时
HAND_EVERY = max(1, int(os.environ.get("HAND_EVERY", "2")))      # 手部模型几帧跑一次。Jetson 实测：并行+隔帧 8.9 FPS，并行+每帧 4.4 FPS
VIEWER_MEM = os.environ.get("VIEWER_MEM", "600MB")                # rerun 窗口最多攒多少历史
import rerun.blueprint as rrb

import zone_monitor as zm
from camera_backend import open_camera
try:
    from jaka_model import (JakaModel, ARM_LINKS, link_color,
                            rpy_to_R as _rpy)
    from robot_frames import RobotFrames, align_to_floor
except Exception:          # 没解压仿真包时照样能跑，只是没有机器人
    JakaModel = RobotFrames = None
from zone_tune import build_corners, load_cfg, save_cfg
from zone_monitor import (Grader, Ground, MotionGate, RISK, W, H, FPS,
                          ZONE_FILE, QualityGate, frame_quality,
                          judge, make_hands, make_pose, in_zone, MIN_VIS,
                          JointTracker, ReachZone, PersonTracker, judge_multi,
                          BODY_PARTS)

# BlazePose 的骨架连线，画出来才像个人
BONES = [(11, 12), (11, 23), (12, 24), (23, 24),
         (11, 13), (13, 15), (12, 14), (14, 16),
         (23, 25), (25, 27), (24, 26), (26, 28),
         (27, 31), (28, 32), (0, 11), (0, 12)]

COLOR = {"空闲": [60, 200, 90], "旁边有人": [240, 200, 60],
         "看不清": [150, 150, 150], "手伸入": [240, 140, 50],
         "占用": [230, 60, 60]}


def log_floor(g, half=4.0, step=0.5):
    """在实测地面（zone_config 里标的平面）上画一张网格，代替 rerun 默认网格。

    以相机正下方为中心，边长 2*half 米、每 step 米一条线；离相机近的一圈画实一点。
    """
    n = np.asarray(g.normal, float); n /= np.linalg.norm(n)
    o = np.asarray(g.origin, float)
    c = np.dot(o, n) * n          # 相机原点（0,0,0）在地面上的垂足
    a = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 0, 1.0])
    u = np.cross(n, a); u /= np.linalg.norm(u)
    v = np.cross(n, u)
    ks = np.arange(-half, half + 1e-6, step)
    lines = [np.array([c + k * u - half * v, c + k * u + half * v]) for k in ks]
    lines += [np.array([c + k * v - half * u, c + k * v + half * u]) for k in ks]
    rr.log("/world/floor", rr.LineStrips3D(lines, colors=[[150, 150, 150, 110]],
                                           radii=0.003), static=True)


def blueprint():
    return rrb.Blueprint(
        rrb.Horizontal(
            # 默认网格画在相机所在的高度，不是地面；关掉，改画实测地面（见 log_floor）
            rrb.Spatial3DView(origin="/world", name="三维场景",
                              line_grid=rrb.archetypes.LineGrid3D(visible=False)),
            rrb.Vertical(
                rrb.Spatial2DView(origin="/cam", name="相机画面"),
                rrb.TextDocumentView(origin="/status", name="判定"),
                rrb.TimeSeriesView(origin="/plot", name="离边界距离"),
                row_shares=[3, 2, 2],
            ),
            column_shares=[3, 1],
        ),
        collapse_panels=True,
    )


def zone_edges(ground):
    """区域柱体的 12 条棱，按顺序给 rerun 画线段。"""
    n = np.asarray(ground.normal, float)
    # 法向可能朝相机也可能背着。统一取「朝上」那一侧：相机系里 +Y 朝下，
    # 所以朝上的那个法向其 Y 分量为负。
    up = n if float(n[1]) < 0 else -n
    base = np.asarray(ground.corners3d, float)
    # 读模块属性而不是导入时的那份拷贝：滑块调高度时改的是 zone_monitor.ZONE_H，
    # 判定和画线必须用同一个值，否则看到的框和实际判的范围对不上。
    top = base + up * zm.ZONE_H
    strips = [np.vstack([base, base[:1]]), np.vstack([top, top[:1]])]
    strips += [np.vstack([base[i], top[i]]) for i in range(len(base))]
    return strips


# 调区域用键盘，不用滑块：OpenCV 5 的 Qt 后端下 createTrackbar 一律报
# "NULL window handler"，先 imshow 再挂也不行。键盘反而更适合精调——
# 滑块 400 个刻度拖不准，按键每次固定步长，想要 1.25 m 就是 1.25 m。
TUNE_WIN = "zone tune  (keys)"
KEYS = """\
 i / k   长  +- 5cm       w / s   前后 +- 2cm
 j / l   宽  +- 5cm       a / d   左右 +- 2cm
 [ / ]   旋转 -+ 5deg      u / n   高度 +- 10cm
 SPACE   save             q       close this window
 (hold SHIFT-less keys; window must be focused)"""


def tune_init(cfg):
    cv2.namedWindow(TUNE_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(TUNE_WIN, 560, 330)


def tune_key(k, cfg):
    """按一次键改一次参数。返回有没有改动。"""
    sh = cfg["shape"]
    sz, off = sh["size_m"], sh["offset_m"]
    step = {ord("i"): ("sz0", +0.05), ord("k"): ("sz0", -0.05),
            ord("j"): ("sz1", +0.05), ord("l"): ("sz1", -0.05),
            ord("w"): ("of0", +0.02), ord("s"): ("of0", -0.02),
            ord("a"): ("of1", -0.02), ord("d"): ("of1", +0.02),
            ord("["): ("rot", -5.0), ord("]"): ("rot", +5.0),
            ord("u"): ("hgt", +0.10), ord("n"): ("hgt", -0.10)}
    if k not in step:
        return False
    what, dv = step[k]
    if what == "sz0":
        sz[0] = max(0.1, round(sz[0] + dv, 3))
    elif what == "sz1":
        sz[1] = max(0.1, round(sz[1] + dv, 3))
    elif what == "of0":
        off[0] = round(off[0] + dv, 3)
    elif what == "of1":
        off[1] = round(off[1] + dv, 3)
    elif what == "rot":
        sh["rotate_deg"] = round((sh.get("rotate_deg", 0.0) + dv) % 360, 1)
    elif what == "hgt":
        cfg["height_m"] = max(0.2, round(cfg["height_m"] + dv, 2))
    return True


def tune_panel(cfg, corners, saved_msg):
    """把当前数值画在小窗里，省得去别处看。"""
    img = np.full((330, 560, 3), 30, np.uint8)
    sh = cfg["shape"]
    sides = [float(np.linalg.norm(corners[i] - corners[(i+1) % 4])) for i in range(4)]
    ctr = np.mean(corners, axis=0)
    head = [
        f"size   {sh['size_m'][0]:.2f} x {sh['size_m'][1]:.2f} m"
        f"    height {cfg['height_m']:.2f} m",
        f"offset ({sh['offset_m'][0]:+.2f}, {sh['offset_m'][1]:+.2f}) m"
        f"    rot {sh.get('rotate_deg', 0):.0f} deg",
        f"edges  " + " / ".join(f"{x:.2f}" for x in sides),
        f"center ({ctr[0]:+.2f}, {ctr[1]:+.2f}, {ctr[2]:+.2f}) m",
    ]
    y = 30
    for L in head:
        cv2.putText(img, L, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (235, 235, 235), 1, cv2.LINE_AA)
        y += 28
    y += 8
    for L in KEYS.split("\n"):
        cv2.putText(img, L, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (150, 185, 225), 1, cv2.LINE_AA)
        y += 24
    if saved_msg:
        cv2.putText(img, saved_msg, (12, 320), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (120, 240, 140), 1, cv2.LINE_AA)
    cv2.imshow(TUNE_WIN, img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=0)
    ap.add_argument("--save", default="", help="存成 .rrd 而不是开窗口")
    ap.add_argument("--cloud-step", type=int, default=6,
                    help="点云抽稀步长，越大越快。默认 6（约 1.1 万点）")
    ap.add_argument("--zone-mode", default="polygon",
                    choices=["polygon", "reach"],
                    help="保护范围怎么定：polygon=自己划的区域，"
                         "reach=机械臂可达空间（要 --robot）")
    ap.add_argument("--no-floor-align", action="store_true",
                    help="不按实测地面摆正机器人（相机确实按设计装好时用）")
    ap.add_argument("--reach-margin", type=float, default=0.25, metavar="米",
                    help="可达空间再往外放多少余量")
    ap.add_argument("--robot", action="store_true",
                    help="把 JAKA S5 画进场景，并算人到机械臂的距离")
    ap.add_argument("--camera-id", default="top",
                    help="这台相机装在机器人的哪个位置（见 sensors.json）")
    ap.add_argument("--animate", action="store_true",
                    help="让机械臂自己动起来（演示用，正弦摆动）")
    ap.add_argument("--anim-speed", type=float, default=0.25, help="摆动快慢")
    ap.add_argument("--joints-live", nargs="?", const="192.168.71.37,192.168.71.36",
                    default="", metavar="左臂IP,右臂IP",
                    help="机械臂跟真机走：只读从控制器 10000 端口取实时关节角"
                         "（默认 192.168.71.37,192.168.71.36）。不往控制器发任何指令")
    ap.add_argument("--joints", default="",
                    help="关节角，形如 left_joint_2=0.6,right_joint_2=-0.6")
    ap.add_argument("--tune", action="store_true",
                    help="开调节窗边看边调区域，rerun 里实时更新")
    ap.add_argument("--backend", default=None, choices=["orbbec", "realsense"])
    a = ap.parse_args()

    if not os.path.isfile(ZONE_FILE):
        sys.exit(f"还没划区域。先跑 zone_tune.py --refit --save")
    ground = Ground.load(ZONE_FILE)
    # 骨架连线不用另外过滤：upper 下判定根本不产出膝以下的点，pts3d 里没有，
    # 腿的连线自然画不出来。显示和判定用的是同一份点。
    print(f"判定用的身体部位：BODY_PARTS={BODY_PARTS}"
          f"（upper=只用髋及以上，full=含腿脚）")

    rr.init("工位占用判定", spawn=False)
    if a.save:
        os.makedirs(os.path.dirname(a.save) or ".", exist_ok=True)
        rr.save(a.save, default_blueprint=blueprint())
        print(f"存到 {a.save}，回头用 rerun 打开")
    else:
        # rr.spawn 默认去 PATH 里找 rerun，而它装在 venv 里；
        # venv/bin/rerun 只是个壳，rerun_cli 下那个才是真二进制。
        import rerun_cli
        exe = os.path.join(os.path.dirname(rerun_cli.__file__), "rerun")
        # memory_limit：超过就丢最旧的历史帧。不设的话窗口把每一帧的点云、
        # 图像全留着，开 15 分钟涨到 1.9 GB、吃 159% CPU，越开越卡（11-14 实测）。
        rr.spawn(default_blueprint=blueprint(), hide_welcome_screen=True,
                 memory_limit=VIEWER_MEM,
                 executable_path=exe if os.path.isfile(exe) else None)
        # 查看器会记住上次的布局，default_blueprint 盖不掉旧的；这里强制生效，
        # 否则改过的布局（例如关掉相机高度的默认网格）不出现。
        rr.send_blueprint(blueprint(), make_active=True)

    rr.log("/world", rr.ViewCoordinates.RDF, static=True)
    log_floor(ground)
    if a.zone_mode == "polygon":
        rr.log("/world/zone", rr.LineStrips3D(zone_edges(ground),
                                              colors=[90, 200, 120], radii=0.008),
               static=True)

    cam = open_camera(a.backend, W, H, FPS)
    intr = cam.intr
    rr.log("/world/camera",
           rr.Pinhole(resolution=[intr.width, intr.height],
                      focal_length=[intr.fx, intr.fy],
                      principal_point=[intr.ppx, intr.ppy]), static=True)

    pose, hands_det = make_pose(), make_hands()
    grader, qgate, ptrack = Grader(), QualityGate(), PersonTracker()

    # ── 机器人 ──
    # 机器人在 base_link 系里，我们所有的点在相机光学系里。两者差一个固定变换
    # （相机装在机器人上，sensors.json 里有设计值）。统一到相机系来画，
    # 因为点云和人体都已经在相机系——换一边转就得把每帧几万个点都转一遍。
    robot, caps_cam, T_cam_base, mesh_paths, q0 = None, [], None, [], {}
    live = None
    if a.joints_live:
        from jaka_live import JakaLive
        ips = (a.joints_live.split(",") + [""])[:2]
        live = JakaLive({"left": ips[0].strip(), "right": ips[1].strip()}).start()
        a.animate = True        # 网格变换每帧都要更新，走动画那条路
    if a.robot and JakaModel is not None:
        try:
            robot = JakaModel()
            rf = RobotFrames.load()
            T_base_cam = rf.T_base_optical(a.camera_id)
            T_cam_base = np.linalg.inv(T_base_cam)
            # 相机挪动过的话，设计外参会让机器人歪着飘在半空。
            # 用现场测的地面把它立直、落地——地面比设计值可信。
            if not a.no_floor_align:
                try:
                    import json as _json
                    _here = os.path.dirname(os.path.abspath(__file__))
                    _cfg = _json.load(
                        open(os.path.join(_here, "zone_config.json")))
                    _pl = _cfg.get("plane") or {}
                    if _pl.get("origin") and _pl.get("normal"):
                        T_cam_base, _lift, _deg = align_to_floor(
                            T_cam_base, _pl["origin"], _pl["normal"])
                        print(f"按实测地面摆正机器人：落了 {_lift:+.2f} m、"
                              f"转了 {_deg:.1f}°")
                except Exception as _e:
                    print(f"（没按地面摆正：{_e}）")
            # 有现场标定（calib_extrinsic.py 写的 extrinsic_calib.json）就用它代替
            # 上面的「设计值 + 地面摆正」；EXTRINSIC=design 强制不用。日志打印用的哪种。
            try:
                from calib_extrinsic import choose_T_cam_base
                T_cam_base, _ = choose_T_cam_base(
                    T_cam_base, fallback_name="设计值" if a.no_floor_align
                    else "设计值 + 地面摆正")
            except Exception as _e:
                print(f"（外参：没读标定文件，用设计值 + 地面摆正：{_e}）")
            q0 = {}
            for kv in a.joints.split(","):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    q0[k.strip()] = float(v)
            T_links = robot.forward(q0)

            def to_cam(p):
                return (T_cam_base[:3, :3] @ np.asarray(p, float)) + T_cam_base[:3, 3]

            # 机械臂包络（相机系），用来算人离它多远
            caps_cam = [(to_cam(x), to_cam(y), r)
                        for x, y, r in robot.arm_capsules(q0)]

            if a.zone_mode == "reach":
                sp = robot.reach_spheres(margin=a.reach_margin, samples=600)
                ground = ReachZone([(to_cam(o), r) for o, r, _ in sp])
                rr.log("/world/zone", rr.Ellipsoids3D(
                    centers=[o for o, r in ground.spheres],
                    half_sizes=[[r, r, r] for o, r in ground.spheres],
                    colors=[[90, 200, 120]], fill_mode="MajorWireframe"),
                    static=True)
                print(f"保护范围 = 机械臂可达空间（含 {a.reach_margin:.2f} m 余量）"
                      f"  {ground.describe()}")

            # 连杆网格。rerun 的 Asset3D 直接吃 OBJ/STL，不用自己解析顶点。
            n_mesh = 0
            for link, parts in robot.meshes.items():
                if link not in T_links:
                    continue
                c = link_color(link)
                # 一个连杆可能有多个零件（臂管 + 关节壳 + 端盖），每个零件
                # 自己的 origin 不一样，必须分别建 entity——合到一个 entity
                # 上只能有一个 Transform3D，零件就会叠在一起。
                for k, mi in enumerate(parts):
                    if not mi["exists"]:
                        continue
                    M = np.eye(4)
                    M[:3, :3] = _rpy(*mi["rpy"])
                    M[:3, 3] = mi["xyz"]
                    path = f"/world/robot/{link}/p{k}"
                    # 网格只传一次（static），之后每帧只更新 4×4 变换——
                    # 网格几百 KB，变换几十字节，动画才跑得动。
                    rr.log(path, rr.Asset3D(path=mi["file"],
                                            albedo_factor=[c[0], c[1], c[2], 255]),
                           static=True)
                    mesh_paths.append((path, link, M))
                    T = T_cam_base @ T_links[link] @ M
                    rr.log(path, rr.Transform3D(translation=T[:3, 3],
                                                mat3x3=T[:3, :3]),
                           static=not a.animate)
                    n_mesh += 1

            # 包络（算距离用的胶囊中轴线）默认不画：11-16 用户嫌两根蓝色粗线碍眼。
            # 要对照时 SHOW_ENVELOPE=1 再打开。
            if SHOW_ENVELOPE:
                rr.log("/world/robot_envelope",
                       rr.LineStrips3D([np.vstack([x, y]) for x, y, _ in caps_cam],
                                       colors=[[90, 140, 255]], radii=0.012),
                       static=True)
            o = T_cam_base[:3, 3]
            print(f"机器人已接入：{n_mesh} 个网格，{len(caps_cam)} 段包络。"
                  f"基座在相机系 ({o[0]:+.2f}, {o[1]:+.2f}, {o[2]:+.2f}) m")
        except Exception as e:
            print(f"接不上机器人（照常跑，只是没有它）：{e}")
            robot, caps_cam = None, []

    cfg, tuning, saved_msg, save_at = None, False, "", 0.0
    if a.tune and a.zone_mode == "reach":
        print("可达空间模式下没什么可调的（范围由机器人结构定），--tune 已忽略")
        a.tune = False
    if a.tune:
        cfg = load_cfg()
        if cfg["plane"].get("origin") is None:
            # 配置里还没有平面，就从已存的区域反推一个——四个角本来就共面
            cfg["plane"] = {"origin": [float(x) for x in ground.origin],
                            "normal": [float(x) for x in ground.normal],
                            "来源": "从已存的 zone_ground.json 反推"}
        tune_init(cfg)
        tuning = True
        cfg["shape"].setdefault("offset_m", [0.0, 0.0])
        print("调节窗已开（点一下让它获得焦点）。按键改区域，rerun 里实时更新；"
              "空格存盘。")
    ys, xs = np.mgrid[0:intr.height:a.cloud_step, 0:intr.width:a.cloud_step]
    t0, n_f, last = time.time(), 0, None
    n_err = 0
    last_qkey, hr_last = None, None
    from concurrent.futures import ThreadPoolExecutor
    hand_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="hands")

    # 逐段计时，每 PERF_EVERY 秒往日志打一行（帧率、各段耗时、帧龄、CPU）。
    perf = {"wait": [], "infer": [], "judge": [], "rest": [], "age": []}
    perf_t, perf_cpu, perf_n = time.time(), time.process_time(), 0
    it_end = time.perf_counter()
    try:
        for fr in cam.frames():
            p_got = time.perf_counter()
            perf["wait"].append(p_got - it_end)
            t = time.time() - t0
            if a.seconds and t > a.seconds:
                break
            n_f += 1
            rr.set_time("时间", duration=t)

            if robot is not None and a.animate:
                # 关节角跟时间走。可达空间不用重算——它是**所有**姿态的并集，
                # 跟此刻摆到哪儿无关，这正是拿它当保护范围的好处。
                if live is not None:
                    # 跟真机走。某条臂失联时那条臂停在 q0（全零或 --joints 给的），
                    # 不拿随机动画兜底——假的动作比不动更会误导人。
                    q = {**q0, **live.q()}
                else:
                    q = robot.wander(t, speed=a.anim_speed)
                # 姿态没变就整段跳过：正运动学 + 37 个网格变换 + 包络。
                # 真机关节角 2 秒才更新一次，原先每帧都重算——11-14 在 Jetson 上
                # 剖析，这块占了 13% 的时间（一帧算 6 遍 forward）。
                qkey = tuple(round(v, 4) for _, v in sorted(q.items()))
            if robot is not None and a.animate and qkey != last_qkey:
                last_qkey = qkey
                T_links = robot.forward(q)
                for path, link, M in mesh_paths:
                    T = T_cam_base @ T_links[link] @ M
                    rr.log(path, rr.Transform3D(translation=T[:3, 3],
                                                mat3x3=T[:3, :3]))
                caps_cam = [((T_cam_base[:3, :3] @ x) + T_cam_base[:3, 3],
                             (T_cam_base[:3, :3] @ y) + T_cam_base[:3, 3], r)
                            for x, y, r in robot.arm_capsules(q)]
                if SHOW_ENVELOPE:
                    rr.log("/world/robot_envelope",
                           rr.LineStrips3D([np.vstack([x, y]) for x, y, _ in caps_cam],
                                           colors=[[90, 140, 255]], radii=0.012))

            if tuning:
                corners = build_corners(cfg["plane"]["origin"],
                                        cfg["plane"]["normal"],
                                        cfg["shape"]["size_m"],
                                        cfg["shape"]["offset_m"],
                                        cfg["shape"]["rotate_deg"])
                ground = Ground(corners)
                zm.ZONE_H = cfg["height_m"]      # 判定和画线共用这一个
                tune_panel(cfg, corners, saved_msg)
                k = cv2.waitKey(1) & 0xFF
                if k == ord(" "):
                    f1, _ = save_cfg(cfg, corners)
                    saved_msg = f"saved -> {os.path.basename(f1)}"
                    save_at = time.time()
                    print(f"✓ 已存 {f1}")
                elif k == ord("q"):
                    tuning = False
                    cv2.destroyWindow(TUNE_WIN)
                else:
                    tune_key(k, cfg)
                if saved_msg and time.time() - save_at > 3:
                    saved_msg = ""

            depth, color = fr.depth, fr.color
            rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
            img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            ts = int(t * 1000)
            # 姿态和手部两个模型**并行**跑：MediaPipe 推理时会释放 GIL，开一个
            # 线程就能吃到第二个核。一帧的时间从「77 ms + 64 ms」变成「取较慢
            # 的那个」（11-14 Jetson 实测两者串行占每帧 70%）。
            # 手部还可以用 HAND_EVERY 隔帧跑，中间沿用上一帧的结果。
            # 推理或判定出错时不能让整个监控退出（11-16 实测：一帧空检测的下标
            # bug 直接把进程弄崩，崩了也没人报警）。按「宁可多停」：这一帧当
            # 「看不清」报警，接着跑；连续出错超过 MAX_ERR_FRAMES 帧才退出。
            try:
                fut = None
                if hr_last is None or n_f % HAND_EVERY == 0:
                    fut = hand_pool.submit(hands_det.detect_for_video, img, ts)
                pr = pose.detect_for_video(img, ts)
                if fut is not None:
                    hr_last = fut.result()
                hr = hr_last
                p_inf = time.perf_counter()
                perf["infer"].append(p_inf - p_got)
                poses = pr.pose_landmarks if pr.pose_landmarks else []
                lms = poses[0] if poses else None
                hl = hr.hand_landmarks if hr.hand_landmarks else []

                q_ok, q_why = frame_quality(depth, color)
                blind, blind_s = qgate.feed(q_ok, t)
                q = (blind, q_why, blind_s)
                raw, why, info = judge_multi(poses, hl, depth, intr, ground,
                                             ptrack, t, q, caps_cam)
                n_err = 0
            except Exception as e:
                n_err += 1
                hr_last = None
                poses, lms, hl = [], None, []
                q_ok, q_why, blind, blind_s = False, "检测出错", True, 0.0
                raw, info = "看不清", {}
                why = f"检测出错（{type(e).__name__}: {e}），这一帧按看不清处理"
                if n_err == 1 or n_err % 30 == 0:
                    print(f"[{t:6.1f}s] ⚠ {why}（连续第 {n_err} 帧）", flush=True)
                    traceback.print_exc()
                if n_err >= MAX_ERR_FRAMES:
                    print(f"✗ 连续 {n_err} 帧检测出错，退出", flush=True)
                    raise
                p_inf = time.perf_counter()
            state = grader.feed(raw, t)
            risk = RISK[state]
            col = COLOR[state]
            p_jdg = time.perf_counter()
            perf["judge"].append(p_jdg - p_inf)
            if fr.t_sys is not None:
                perf["age"].append(time.time() - fr.t_sys)
            perf_n += 1
            now = time.time()
            if now - perf_t >= PERF_EVERY:
                ms = lambda k: (f"{np.median(perf[k])*1000:.0f}/{np.percentile(perf[k], 90)*1000:.0f}"
                                if perf[k] else "-")
                cpu = (time.process_time() - perf_cpu) / (now - perf_t) * 100
                print(f"[perf] {perf_n/(now-perf_t):.1f} FPS | 中位/p90 ms：等帧 {ms('wait')}"
                      f"  推理 {ms('infer')}  判定 {ms('judge')}  显示 {ms('rest')}"
                      f" | 帧龄(相机给出→判定出结果) {ms('age')} | 本进程 CPU {cpu:.0f}%"
                      f"（100%=1 核）", flush=True)
                perf = {k: [] for k in perf}
                perf_t, perf_cpu, perf_n = now, time.process_time(), 0

            rr.log("/cam/rgb", rr.Image(rgb).compress(jpeg_quality=70))
            if a.zone_mode == "polygon":
                rr.log("/world/zone", rr.LineStrips3D(zone_edges(ground),
                                                      colors=col, radii=0.008))
            else:
                rr.log("/world/zone", rr.Ellipsoids3D(
                    centers=[o for o, r in ground.spheres],
                    half_sizes=[[r, r, r] for o, r in ground.spheres],
                    colors=col, fill_mode="MajorWireframe"))

            # 点云。只画有深度的点，颜色取自彩色图。
            zs = depth[::a.cloud_step, ::a.cloud_step]
            m = (zs > 0.2) & (zs < 4.0)
            if m.sum():
                px = np.stack([xs[m], ys[m]], axis=1)
                P = intr.deproject_many(px, zs[m])
                cols = rgb[ys[m], xs[m]]
                rr.log("/world/cloud", rr.Points3D(P, colors=cols, radii=0.004))

            # 骨架。进区域的点标红，区外的标白——一眼看出判定依据。
            for pi, lms in enumerate(poses[:4]):
                # 直接用判定算出来的那份点，不再自己算一遍——
                # 自己算就会漏掉时间一致性、机械臂遮挡这些过滤，
                # 画面上看到的点和停机依据的点对不上，调试全乱。
                pinfo = (info.get("each") or [{}])[pi] \
                    if pi < len(info.get("each") or []) else {}
                pts3d = pinfo.get("pts3d") or {}
                inferred = pinfo.get("inferred") or set()
                flew = pinfo.get("flyaway") or []
                # 被判为飞点的，单独用深灰小点画出来，能看见它飞去哪了
                if flew:
                    rr.log(f"/world/person/p{pi}/flyaway",
                           rr.Points3D(np.array(flew), colors=[[90, 90, 90]],
                                       radii=0.012))
                else:
                    rr.log(f"/world/person/p{pi}/flyaway",
                           rr.Points3D(np.empty((0, 3))))

                if pts3d:
                    idx_ok = list(pts3d.keys())
                    P = np.array([pts3d[i] for i in idx_ok])
                    C = np.array([[230, 60, 60] if in_zone(pts3d[i], ground)
                                  else [240, 240, 240] for i in idx_ok])
                    # 实测的点画大，骨架推出来的画小一圈 + 偏暗，
                    # 一眼能分出「这是量到的」还是「这是算出来的」
                    R = np.array([0.013 if i in inferred else 0.022
                                  for i in idx_ok])
                    for k, i in enumerate(idx_ok):
                        if i in inferred:
                            C[k] = (C[k] * 0.55).astype(C.dtype)
                    rr.log(f"/world/person/p{pi}/joints",
                           rr.Points3D(P, colors=C, radii=R))
                    segs = [np.vstack([pts3d[i], pts3d[j]])
                            for i, j in BONES if i in pts3d and j in pts3d]
                    rr.log(f"/world/person/p{pi}/bones",
                           rr.LineStrips3D(segs, colors=col, radii=0.006)
                           if segs else rr.LineStrips3D([]))
                else:
                    rr.log(f"/world/person/p{pi}/joints",
                           rr.Points3D(np.empty((0, 3))))
                    rr.log(f"/world/person/p{pi}/bones", rr.LineStrips3D([]))

            # 这一帧没检到的人，把上一帧留下的骨架清掉，免得画面里留残影
            for pi in range(len(poses), 4):
                rr.log(f"/world/person/p{pi}/joints", rr.Points3D(np.empty((0, 3))))
                rr.log(f"/world/person/p{pi}/bones", rr.LineStrips3D([]))
                rr.log(f"/world/person/p{pi}/flyaway", rr.Points3D(np.empty((0, 3))))

            if info.get("nearest_cam"):
                rr.log("/world/person/nearest",
                       rr.Points3D([info["nearest_cam"]],
                                   colors=[[255, 0, 180]], radii=0.035))

            # 人离机械臂多远。这是「区域占用」之外的另一个量：
            # 区域是预先划的固定范围，这个是跟着机械臂当前姿态走的实际距离。
            d_arm = None
            if caps_cam and info.get("nearest_cam"):
                allp = [info["nearest_cam"]]
                d_arm, _ = JakaModel.distance_to(allp, caps_cam)
                rr.log("/plot/dist_arm", rr.Scalars(float(d_arm)))

            e = info.get("edge_m")
            if e is not None:
                rr.log("/plot/edge_m", rr.Scalars(float(e)))
            rr.log("/plot/risk", rr.Scalars(float(risk)))

            # upper 下顺带显示第一个人的髋用的是哪种（可信/估计/不用/未核验）
            hips = ((info.get("each") or [{}])[0].get("hips") or {})
            hips_txt = ("，髋 " + "、".join(f"{k}:{v}" for k, v in sorted(hips.items()))
                        if hips else "")
            rr.log("/status", rr.TextDocument(
                f"# {state}　risk={risk}\n\n"
                f"{why}\n\n"
                f"- 原始判断 `{raw}`\n"
                f"- 机械臂关节角：{('真机 ' + live.status()) if live else ('随机动画' if a.animate else '固定姿态')}\n"
                f"- 离边界 `{'—' if e is None else f'{e:+.3f} m（正=在区域里）'}`\n"
                f"- 画面里 `{info.get('n_persons', 0)}` 个人"
                + ("：" + "、".join(f"#{p['id']} {p['state']}"
                                    for p in info.get("persons", [])[:4])
                   if info.get("n_persons", 0) > 1 else "") + "\n"
                f"- 判定部位 `{BODY_PARTS}`{hips_txt}\n"
                f"- 躯干 {info.get('torso_pts',0)} 点 / 四肢 "
                f"{info.get('limb_pts',0)} 点 / 头 {info.get('head_pts',0)} 点 / "
                f"手 {info.get('hand_pts',0)} 点\n"
                + (f"- 离机械臂 `{d_arm*1000:+.0f} mm`（负=已进包络）\n"
                   if d_arm is not None else "")
                + f"- 丢掉的飞点 `{info.get('dropped_pts', 0)}` 个"
                f"（出画面被外推、取到背景深度的）\n"
                f"- 画面{'不可信：' + q_why if blind else ('可信' if q_ok else f'抖了一下（{blind_s:.1f}s，没到 {qgate.BAD_S}s 不报）')}\n"
                f"- 第 {n_f} 帧，{n_f/max(t,1e-6):.1f} FPS",
                media_type="text/markdown"))

            if state != last:
                print(f"[{t:6.1f}s] {last} → {state}   risk={risk}   {why}",
                      flush=True)
                last = state
            it_end = time.perf_counter()
            perf["rest"].append(it_end - p_jdg)
    except KeyboardInterrupt:
        pass
    finally:
        # 只关相机——这一步决定下次还能不能开。识别模型**不关**：
        # MediaPipe 1.2.3 的 PoseLandmarker.close() 在 Jetson 上要等 15 秒左右
        # （11-14 抓栈确认，停一次程序固定 16.7 秒），进程都要退了，关它纯属白等。
        cam.close()
        if live is not None:
            live.stop()
    print(f"\n{n_f} 帧，{time.time()-t0:.1f} 秒", flush=True)
    return 0


if __name__ == "__main__":
    code = main()
    # 直接结束进程，跳过解释器退出时各对象的析构——模型对象的 __del__ 里还会再
    # 调一次那个慢 close()。相机已在 main 的 finally 里关好；rerun 先把没发完的发掉。
    try:
        rr.flush(blocking=True)
    except Exception:
        pass
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(code)
