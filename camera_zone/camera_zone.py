#!/usr/bin/env python3
"""camera_zone —— 五节点架构里的相机节点。

分工：这个节点只回答「工位现在是什么状态」，不碰机器人。撤不撤运动许可
由 workcell_supervisor 决定，怎么暂停由臂驱动决定。

发出的话题
    /workcell/occupancy    workcell_interfaces/msg/ZoneState

    三值（老师定的）：CLEAR 没人 / OCCUPIED 被占 / UNKNOWN 看不清
    附加字段：near（在区外但靠近，给降速用）、edge_m（离边界多远）、
              evidence（判定依据是躯干还是手）、reason、detail_json

    附加字段是我加的：三值会把「人在旁边但没进去」这一档信息丢掉，
    监督器拿不到就做不了降速。字段定义等和老师对齐后可能调整。

可选：加 --direct-permit 时，这个节点**还会直接撤运动许可**：
      /workcell/permit        workcell_interfaces/msg/MotionPermit（allow=false）
      /workcell/speed_scale   std_msgs/Float32，0~1 的速度系数
      /workcell/confirm_resume  服务，人工确认后才恢复

      默认关着。正常分工里撤许可是 workcell_supervisor 的事，这个开关是
      监督器没上线时的兜底——不然识别得再准，机器人也不会停。

兼容：加 --legacy 会同时发旧的 /human_zone/* 四条 std_msgs 话题，
      方便和还没改的订阅端并存。

没有相机也能跑（给仿真联调用）：
    --fake script     按内置时间表循环发：空 8 秒 → 占用 6 秒 → 空 6 秒 → 看不清 3 秒
    --fake keys       键盘控制，终端里按 c/o/u 分别发 CLEAR/OCCUPIED/UNKNOWN
    --fake clear      一直发 CLEAR
    --fake occupied   一直发 OCCUPIED
  这几种模式不开相机、不加载模型，装了 rclpy 就能跑。

「看不清」为什么也算 risk=2：画面被挡、人贴太近导致测不出位置时，系统
不知道区域里有没有人。不知道就按最坏算——这正是老师要求里「画面被遮挡、
看不清的情况也要处理」那一条。

⚠️ 域隔离：默认 ROS_DOMAIN_ID=42 且 ROS_LOCALHOST_ONLY=1，由 rosrun.sh 设。
   局域网里 domain 0 上跑着真实机器人的控制话题，绝不能把测试节点发到那上面去。
   要联调时再显式改 domain，并且先确认对面在等什么话题。

跑法（在本目录下执行）
    ./rosrun.sh camera_zone.py                 发话题，不开窗
    ./rosrun.sh camera_zone.py --show          同时开本地预览窗
    ./rosrun.sh camera_zone.py --rate 10       限制发布频率（默认跟相机帧率）

看收到了什么（另开一个终端）
    source /opt/ros/humble/setup.bash
    ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 ros2 topic echo /workcell/occupancy
"""
import argparse
import json
import os
import sys
import traceback
import time

import cv2
import mediapipe as mp
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float32, Int32, String
from workcell_interfaces.msg import MotionPermit, ZoneState
from workcell_interfaces.srv import ConfirmResume

try:
    from robot_frames import RobotFrames
except Exception:          # 没解压仿真包时照常跑，只是少了基座系坐标
    RobotFrames = None

from camera_backend import open_camera
from zone_monitor import (Grader, Ground, MotionGate, RISK, W, H, FPS,
                          ZONE_FILE, QualityGate, frame_quality, judge,
                          make_hands, make_pose, BODY_PARTS)


class PermitGate:
    """直接发运动许可（`--direct-permit`）。**默认关着。**

    正常分工里这不是我的活：我只发状态，撤不撤许可由 workcell_supervisor
    决定。开这个开关是为了在监督器还没上线时也能演示完整的
    「识别 → 中断 → 人工确认 → 恢复」——监督器一上线就把开关关掉。

    三条规矩，都是安全侧该有的样子：

      ① **撤销立刻，恢复要等人**。人走了不自动放行——老师明确要求过。
         区域清空只是「可以确认」的前提，不是恢复本身。
      ② **看不清按最坏算**。画面被挡、测不出位置，一样撤许可。
      ③ **区域里还有人时，确认请求直接拒绝**，不给误操作留口子。

    减速走另一条话题 `/workcell/speed_scale`（0~1 的系数），不往
    MotionPermit 里塞字段——那个结构以老师的定义为准，别污染。
    """

    SLOW_SCALE = float(os.environ.get("SLOW_SCALE", "0.3"))   # 旁边有人时降到几成

    def __init__(self, node, qos, source="camera_zone"):
        self.node, self.source = node, source
        self.pub = node.create_publisher(MotionPermit, "/workcell/permit", qos)
        self.pub_speed = node.create_publisher(Float32, "/workcell/speed_scale", qos)
        self.srv = node.create_service(ConfirmResume, "/workcell/confirm_resume",
                                       self._on_confirm)
        self.latched = False      # 撤过许可，还没被人工确认
        self.clear_now = False    # 这一帧区域是不是空的
        self.last = None          # 上次发出去的 (allow, scale)，变了才发

    def _on_confirm(self, req, res):
        who = req.confirmed_by or "（没填）"
        if not self.clear_now:
            res.accepted = False
            res.message = "区域里还有人，不能恢复"
            self.node.get_logger().warn(f"拒绝恢复请求（{who}）：{res.message}")
            return res
        self.latched = False
        res.accepted = True
        res.message = "已恢复运动许可"
        self.node.get_logger().info(f"{who} 确认恢复，放行")
        self._send(True, 1.0, f"{who} 人工确认恢复")
        return res

    def feed(self, state):
        """喂一帧的五档结果，必要时发许可和速度系数。"""
        self.clear_now = state in ("空闲", "旁边有人")
        danger = state in ("占用", "手伸入", "看不清")
        if danger:
            self.latched = True
        allow = not self.latched
        scale = 0.0 if not allow else (self.SLOW_SCALE
                                       if state == "旁边有人" else 1.0)
        why = ("区域被占" if state in ("占用", "手伸入") else
               "画面看不清，按最坏处理" if state == "看不清" else
               "等人工确认恢复" if self.latched else
               "人在旁边，降速" if state == "旁边有人" else "区域空闲")
        self._send(allow, scale, why)

    def _send(self, allow, scale, why):
        if self.last == (allow, round(scale, 3)):
            return                      # 没变就不刷屏，订阅端有 TRANSIENT_LOCAL
        self.last = (allow, round(scale, 3))
        m = MotionPermit()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.allow = bool(allow)
        m.reason = why
        m.source = self.source
        self.pub.publish(m)
        self.pub_speed.publish(Float32(data=float(scale)))
        self.node.get_logger().info(
            f"运动许可 → {'放行' if allow else '撤销'}"
            f"（速度 {scale*100:.0f}%）：{why}")

    def shutdown(self):
        """退出时撤许可。节点没了还留着一条 allow=true 是危险的。"""
        try:
            self.last = None
            self._send(False, 0.0, "相机节点已退出")
        except Exception:
            pass


# 五档 → 三值的映射表和消息构造在 main 和 run_fake 里都要用，提到外面
def _jsonable(o):
    """json.dumps 的兜底：numpy 数组/数值转列表/数字，set 转排好序的列表。"""
    if isinstance(o, np.ndarray):
        return np.round(o.astype(float), 4).tolist()
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, (set, frozenset)):
        return sorted(o, key=str)
    return str(o)


MAX_ERR_FRAMES = int(os.environ.get("MAX_ERR_FRAMES", "30"))   # 连续多少帧检测出错就退出（约 1 秒）
FAKE_SCRIPT = [("空闲", 8.0), ("占用", 6.0), ("空闲", 6.0), ("看不清", 3.0)]
KEY_MAP = {"c": "空闲", "o": "占用", "u": "看不清", "n": "旁边有人"}


def run_fake(a):
    """不开相机，只发状态。给没有这台相机的人联调用。

    为什么要有这个：老师那边是在 MuJoCo 仿真里验证「暂停—减速—恢复」，
    他手上没有我们这台相机。要是拿到包跑不起来，这一轮就白送了。
    """
    import threading

    rclpy.init()
    node = Node("camera_zone")
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    pub = node.create_publisher(ZoneState, "/workcell/occupancy", qos)
    gate = PermitGate(node, qos) if a.direct_permit else None

    TO_ZONE = {"空闲": ZoneState.CLEAR, "旁边有人": ZoneState.CLEAR,
               "看不清": ZoneState.UNKNOWN,
               "手伸入": ZoneState.OCCUPIED, "占用": ZoneState.OCCUPIED}

    def send(st, why):
        m = ZoneState()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = "camera_color_optical_frame"
        m.state = TO_ZONE[st]
        m.zone_id = a.zone_id
        m.near = bool(st == "旁边有人")
        m.edge_m = float("nan")
        m.evidence = (ZoneState.EV_TORSO if st == "占用" else
                      ZoneState.EV_LOST if st == "看不清" else ZoneState.EV_NONE)
        m.reason = why
        m.detail_json = json.dumps({"fake": a.fake, "档位": st},
                                   ensure_ascii=False)
        pub.publish(m)
        if gate is not None:
            gate.feed(st)
        rclpy.spin_once(node, timeout_sec=0.0)

    node.get_logger().info(f"假数据模式 [{a.fake}]，不开相机。"
                           f"域 {os.environ.get('ROS_DOMAIN_ID','0')}")
    t0, last = time.time(), None
    try:
        if a.fake in ("clear", "occupied", "unknown"):
            st = {"clear": "空闲", "occupied": "占用", "unknown": "看不清"}[a.fake]
            while rclpy.ok() and not (a.seconds and time.time()-t0 > a.seconds):
                send(st, f"假数据：固定 {st}")
                time.sleep(0.2)

        elif a.fake == "keys":
            print("按 c=没人  o=被占  u=看不清  n=旁边有人  q=退出，按完回车")
            cur = ["空闲"]

            def reader():
                for line in sys.stdin:
                    k = line.strip().lower()[:1]
                    if k == "q":
                        cur[0] = None; return
                    if k in KEY_MAP:
                        cur[0] = KEY_MAP[k]
                        print(f"  → {cur[0]}", flush=True)
            threading.Thread(target=reader, daemon=True).start()
            while rclpy.ok() and cur[0] is not None:
                if a.seconds and time.time()-t0 > a.seconds:
                    break
                send(cur[0], f"假数据：键盘 {cur[0]}")
                time.sleep(0.2)

        else:   # script
            total = sum(d for _, d in FAKE_SCRIPT)
            print(f"时间表循环，一轮 {total:.0f} 秒："
                  + " → ".join(f"{s} {d:.0f}s" for s, d in FAKE_SCRIPT))
            while rclpy.ok():
                if a.seconds and time.time()-t0 > a.seconds:
                    break
                el = (time.time() - t0) % total
                acc, st = 0.0, FAKE_SCRIPT[0][0]
                for name, dur in FAKE_SCRIPT:
                    if el < acc + dur:
                        st = name; break
                    acc += dur
                if st != last:
                    node.get_logger().info(f"[{time.time()-t0:6.1f}s] → {st}")
                    last = st
                send(st, f"假数据：时间表 {st}")
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        send("看不清", "假数据节点已退出")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--show", action="store_true", help="同时开本地预览窗")
    ap.add_argument("--rate", type=float, default=0.0,
                    help="最高发布频率（Hz）。0 = 每帧都发")
    ap.add_argument("--seconds", type=float, default=0, help="跑多少秒后自动停")
    ap.add_argument("--backend", default=None, choices=["orbbec", "realsense"])
    ap.add_argument("--zone-id", default="default", help="工位编号，多工位时区分")
    ap.add_argument("--direct-permit", action="store_true",
                    help="直接发 /workcell/permit 中断机械臂，不等监督器。"
                         "恢复要调 /workcell/confirm_resume 服务。"
                         "监督器上线后关掉这个开关")
    ap.add_argument("--legacy", action="store_true",
                    help="同时发旧的 /human_zone/* 话题")
    ap.add_argument("--camera-id", default="top",
                    help="这台相机在机器人上的位置，见 sensors.json（top/left_wrist/…）")
    ap.add_argument("--no-base-frame", action="store_true",
                    help="不换算到机器人基座系，只给相机系坐标")
    ap.add_argument("--fake", default="", metavar="模式",
                    choices=["", "script", "keys", "clear", "occupied", "unknown"],
                    help="不接相机，造假状态给仿真联调用")
    a = ap.parse_args()

    if a.fake:
        return run_fake(a)

    if not os.path.isfile(ZONE_FILE):
        sys.exit(f"还没划操作区。先跑：\n"
                 f"  ./rosrun.sh zone_monitor.py --auto-calib 0.5   自动划\n"
                 f"  ./rosrun.sh zone_monitor.py --calib            人站四个角")
    ground = Ground.load(ZONE_FILE)

    rclpy.init()
    node = Node("camera_zone")

    # 安全状态要用「可靠 + 留最后一条」：订阅者晚起也能立刻拿到当前档位，
    # 不用干等下一次状态变化。只留 1 条，老状态没有意义。
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    pub_zone = node.create_publisher(ZoneState, "/workcell/occupancy", qos)
    gate = PermitGate(node, qos) if a.direct_permit else None
    legacy = None
    if a.legacy:
        legacy = (node.create_publisher(String, "/human_zone/state", qos),
                  node.create_publisher(Int32, "/human_zone/risk", qos),
                  node.create_publisher(Bool, "/human_zone/occupied", qos),
                  node.create_publisher(String, "/human_zone/detail", qos))

    # 五档 → 三值。"旁边有人" 归 CLEAR：人确实在区外，没占住工位；
    # 要不要因此降速，由监督器看 near 字段自己决定——那是它的策略，不是我的判断。
    TO_ZONE = {"空闲": ZoneState.CLEAR, "旁边有人": ZoneState.CLEAR,
               "看不清": ZoneState.UNKNOWN,
               "手伸入": ZoneState.OCCUPIED, "占用": ZoneState.OCCUPIED}
    TO_EV = {"EV_NONE": ZoneState.EV_NONE, "EV_TORSO": ZoneState.EV_TORSO,
             "EV_LIMB": ZoneState.EV_LIMB, "EV_HEAD": ZoneState.EV_HEAD,
             "EV_LOST": ZoneState.EV_LOST}

    # 证据枚举不变。BODY_PARTS=upper（默认）时 EV_LIMB 只可能来自肘、腕和
    # 手部模型的点——膝以下不参与判定；EV_TORSO 来自两肩和过了核验的髋。
    def evidence_of(st, inf):
        if st == "占用":
            return ZoneState.EV_TORSO
        if st == "手伸入":
            return ZoneState.EV_HEAD if "头探进" in inf.get("why", "") \
                else ZoneState.EV_LIMB
        if st == "看不清":
            return ZoneState.EV_LOST
        return ZoneState.EV_NONE

    def make_msg(st, why, inf, risk):
        m = ZoneState()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = "camera_color_optical_frame"
        m.state = TO_ZONE.get(st, ZoneState.UNKNOWN)
        m.zone_id = a.zone_id
        m.near = bool(st == "旁边有人")
        e = inf.get("edge_m")
        m.edge_m = float(e) if e is not None else float("nan")
        m.evidence = evidence_of(st, dict(inf, why=why))
        m.reason = why
        # inf 里有 numpy 数组（pts3d）和 set（inferred），json 默认转不了，
        # 检到人就会抛异常退出。11-16 读代码发现，这里统一兜底转换。
        m.detail_json = json.dumps(dict(inf, 档位=st, risk=int(risk)),
                                   ensure_ascii=False, default=_jsonable)
        return m

    dom = os.environ.get("ROS_DOMAIN_ID", "0")
    local = os.environ.get("ROS_LOCALHOST_ONLY", "0")
    node.get_logger().info(f"判定用的身体部位：BODY_PARTS={BODY_PARTS}"
                           f"（upper=只用髋及以上，full=含腿脚）")
    node.get_logger().info(
        f"域 {dom}，仅本机 {local}。"
        + ("" if local == "1" else "  ⚠ 没开仅本机，话题会发到局域网上"))

    # 相机到机器人基座的变换。读 sensors.json 的设计值——相机固定装在
    # 机器人上，这组数在设计图里就定了，不用现场标。但那是设计值不是实测值
    # （文件自己标着 extrinsics_calibrated: false），真机算距离前要实地量。
    rf = None
    if RobotFrames is not None and not a.no_base_frame:
        try:
            rf = RobotFrames.load()
            o = rf.T_base_optical(a.camera_id)[:3, 3]
            node.get_logger().info(
                f"相机 [{a.camera_id}] 光心在机器人基座系 "
                f"({o[0]:+.3f}, {o[1]:+.3f}, {o[2]:+.3f}) m"
                f"（设计值，未实测标定）")
        except Exception as e:
            node.get_logger().warn(f"读不到相机外参，只给相机系坐标：{e}")
            rf = None
    # 有现场标定（extrinsic_calib.json）就用标定值换算到基座系；EXTRINSIC=design 强制设计值
    to_base = (lambda p: rf.to_base(p, a.camera_id)) if rf is not None else None
    if rf is not None:
        try:
            from calib_extrinsic import choose_T_cam_base
            _T_fb = np.linalg.inv(rf.T_base_optical(a.camera_id))
            _T_sel, _ = choose_T_cam_base(_T_fb, log=node.get_logger().info,
                                          fallback_name="设计值")
            if _T_sel is not _T_fb:
                _T_bc = np.linalg.inv(_T_sel)
                to_base = lambda p: _T_bc[:3, :3] @ np.asarray(p, float) + _T_bc[:3, 3]
        except Exception as e:
            node.get_logger().warn(f"外参：没读标定文件，用设计值：{e}")

    cam = open_camera(a.backend, W, H, FPS)
    pose, hands_det = make_pose(), make_hands()
    grader, gate, qgate = Grader(), MotionGate(), QualityGate()
    period = 1.0 / a.rate if a.rate > 0 else 0.0

    t0, last_pub, n, last_state = time.time(), 0.0, 0, None
    n_err = 0
    try:
        for fr in cam.frames():
            if not rclpy.ok():
                break
            t = time.time() - t0
            if a.seconds and t > a.seconds:
                break
            n += 1

            rgb = cv2.cvtColor(fr.color, cv2.COLOR_BGR2RGB)
            img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            ts = int(t * 1000)
            # 推理或判定出错时不让节点退出：这一帧按「看不清」报警（会撤销运动
            # 许可），接着跑；连续 MAX_ERR_FRAMES 帧都出错才退出（退出时 finally
            # 照样发「看不清」并撤销许可）。11-16 实测一帧空检测 bug 能把进程弄崩。
            try:
                pr = pose.detect_for_video(img, ts)
                hr = hands_det.detect_for_video(img, ts)
                lms = pr.pose_landmarks[0] if pr.pose_landmarks else None
                hlist = hr.hand_landmarks if hr.hand_landmarks else []

                q_ok, q_why = frame_quality(fr.depth, fr.color)
                blind, blind_s = qgate.feed(q_ok, t)
                raw, why, info = judge(lms, hlist, fr.depth, cam.intr, ground,
                                       gate, t, (blind, q_why, blind_s))
                n_err = 0
            except Exception as e:
                n_err += 1
                raw, info = "看不清", {}
                why = f"检测出错（{type(e).__name__}: {e}），这一帧按看不清处理"
                if n_err == 1 or n_err % 30 == 0:
                    node.get_logger().error(f"{why}（连续第 {n_err} 帧）\n"
                                            + traceback.format_exc())
                if n_err >= MAX_ERR_FRAMES:
                    node.get_logger().fatal(f"连续 {n_err} 帧检测出错，节点退出")
                    raise
            state = grader.feed(raw, t)
            risk = RISK[state]

            if period and (time.time() - last_pub) < period:
                continue
            last_pub = time.time()

            inf = dict(info, t=round(t, 2), frame=n, raw=raw)
            if rf is not None and info.get("nearest_cam") is not None:
                try:
                    pb = to_base(info["nearest_cam"])
                    inf["nearest_base"] = [round(float(x), 3) for x in pb]
                except Exception:
                    pass
            pub_zone.publish(make_msg(state, why, inf, risk))
            if gate is not None:
                gate.feed(state)
            if legacy:
                legacy[0].publish(String(data=state))
                legacy[1].publish(Int32(data=int(risk)))
                legacy[2].publish(Bool(data=bool(risk >= 2)))
                legacy[3].publish(String(data=json.dumps(
                    {"state": state, "raw": raw, "risk": int(risk), "why": why,
                     "t": round(t, 2), "frame": n, **info},
                    ensure_ascii=False, default=_jsonable)))
            rclpy.spin_once(node, timeout_sec=0.0)

            if state != last_state:
                node.get_logger().info(f"[{t:6.1f}s] {last_state} → {state}"
                                       f"   risk={risk}   {why}")
                last_state = state

            if a.show:
                vis = fr.color.copy()
                col = [(0, 200, 0), (0, 180, 255), (0, 0, 255)][risk]
                cv2.rectangle(vis, (0, 0), (vis.shape[1]-1, vis.shape[0]-1), col, 6)
                cv2.putText(vis, f"{state}  risk={risk}", (14, 36),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, col, 2)
                cv2.imshow("zone_ros", vis)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前把状态置成「看不清」，不要让订阅者拿着一条过期的「空闲」
        # 以为还在监控。节点没了但那条留存消息还在，这是 TRANSIENT_LOCAL 的坑。
        try:
            pub_zone.publish(make_msg("看不清", "监控节点已退出",
                                      {"edge_m": None}, 2))
            if gate is not None:
                gate.shutdown()
            if legacy:
                legacy[0].publish(String(data="看不清"))
                legacy[1].publish(Int32(data=2))
                legacy[2].publish(Bool(data=True))
                legacy[3].publish(String(data=json.dumps(
                    {"state": "看不清", "why": "监控节点已退出", "risk": 2},
                    ensure_ascii=False)))
            rclpy.spin_once(node, timeout_sec=0.2)
        except Exception:
            pass
        cam.close(); pose.close(); hands_det.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"\n{n} 帧，{time.time()-t0:.1f} 秒")
    return 0


if __name__ == "__main__":
    sys.exit(main())
