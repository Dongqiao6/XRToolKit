"""Hand-only observation in a fixed, user-defined directional frame. No robot."""
from collections import deque, Counter
import json
import time

import cv2
import numpy as np
import yaml

from .capture import DualCapture, PairBuffer
from .hand_tracking import TrackingWorker, VideoHand
from .hand_control import geometry
from .live import RecordedCapture
from .palm_reference import PalmReference, overlay_palm


def directional_basis(right, forward):
    right, forward = np.asarray(right, float), np.asarray(forward, float)
    if not np.isfinite([right, forward]).all():
        raise ValueError("方向数据无效")
    if min(np.linalg.norm(right), np.linalg.norm(forward)) < .10:
        raise ValueError("移动不足10厘米，请重新定向")
    x = right / np.linalg.norm(right)
    f = forward / np.linalg.norm(forward)
    if abs(x @ f) > .5:
        raise ValueError("右移和前移方向不够垂直，请重新定向")
    y = f - x * (x @ f)
    y /= np.linalg.norm(y)
    return np.column_stack((x, y, np.cross(x, y)))


class DirectionReference:
    def __init__(self):
        self.endpoints = []
        self.basis = self.origin = None
        self.history = deque(maxlen=30)
        self.up_start = None
        self.z_sign = None

    @property
    def complete(self):
        return self.basis is not None and self.z_sign is not None

    @property
    def step(self):
        if self.complete:
            return 6
        return len(self.endpoints) if self.basis is None else (4 if self.up_start is None else 5)

    def advance(self, now):
        if self.complete:
            return "定向已完成；无需继续确认。位置归零不会改变轴向。"
        if self.basis is None:
            self.confirm(now)
        else:
            self.confirm_up(now)
        return "定向完成：X右、Y前、Z上。" if self.complete else "本步已记录，请按下一步提示移动。"

    def add(self, timestamp, point):
        if self.history and timestamp <= self.history[-1][0]:
            return
        self.history.append((timestamp, np.asarray(point).copy()))

    def stable_point(self, now):
        recent = [p for t, p in self.history if now-t <= .5]
        times = [t for t, _ in self.history if now-t <= .5]
        if len(recent) < 5 or times[-1]-times[0] < .25 or now-times[-1] > .15:
            raise ValueError("需要新鲜有效的腕部观测，先停稳半秒")
        points = np.asarray(recent)
        if np.max(np.linalg.norm(points-points.mean(axis=0), axis=1)) > .01:
            raise ValueError("手还在动，请在端点停稳半秒")
        return points.mean(axis=0)

    def confirm(self, now):
        if self.basis is not None:
            return
        point = self.stable_point(now)
        if len(self.endpoints) == 1 and np.linalg.norm(point-self.endpoints[0]) < .10:
            raise ValueError("右移不足10厘米；保留起点，请继续向右移动。")
        if len(self.endpoints) == 3:
            basis = directional_basis(self.endpoints[1]-self.endpoints[0], point-self.endpoints[2])
            self.basis = basis
            self.origin = point.copy()
        self.endpoints.append(point)
        self.history.clear()

    def confirm_up(self, now):
        if self.complete:
            return
        point = self.stable_point(now)
        self.history.clear()
        if self.up_start is None:
            self.up_start = point
            self.z_sign = None
            return
        delta = self.basis.T @ (point-self.up_start)
        if abs(delta[2]) < .10 or np.linalg.norm(delta[:2]) > abs(delta[2])*.5:
            raise ValueError("向上位移不足或横移过大；保留起点，请调整后确认。")
        self.z_sign = 1. if delta[2] > 0 else -1.
        self.up_start = None

    def position(self, point):
        if self.z_sign is None:
            raise ValueError("尚未确认上移方向")
        # Display-coordinate sign only: do not feed this reflection to SO(3).
        return (self.basis.T @ (point-self.origin))*[1., 1., self.z_sign]


STEPS = (
    ("1 / 6  右移起点", "手放舒服的位置，停稳后确认。"),
    ("2 / 6  右移终点", "向本人右侧移动15–25厘米，停稳确认。"),
    ("3 / 6  前移起点", "手回到舒服的位置，停稳确认。"),
    ("4 / 6  前移终点", "向身体正前方移动15–25厘米，停稳确认。"),
    ("5 / 6  上移起点", "手回到舒服的位置，停稳确认。"),
    ("6 / 6  上移终点", "向上移动15–25厘米，停稳确认。"),
)
BUTTONS = {"advance": (16, 730, 238, 784), "origin": (250, 730, 436, 784),
           "palm": (448, 730, 690, 784), "reset": (702, 730, 918, 784), "quit": (930, 730, 1184, 784)}


def view_state(ref, now, pos_ok, reason, palm=None):
    enabled = False
    readiness = reason
    if pos_ok:
        try:
            ref.stable_point(now)
            enabled, readiness = True, "已停稳，可以确认"
        except ValueError as exc:
            readiness = str(exc)
    palm_enabled, palm_readiness = False, ""
    if palm is not None:
        try:
            palm.candidate(now)
            palm_enabled, palm_readiness = True, "手姿稳定，可设为姿态基准"
        except ValueError as exc:
            palm_readiness = str(exc)
    return dict(title="定向完成" if ref.complete else STEPS[ref.step][0],
                instruction="自由移动右手，检查右/前/上是否增大。" if ref.complete else STEPS[ref.step][1],
                advance_enabled=enabled and not ref.complete, origin_enabled=enabled and ref.complete,
                readiness=readiness, complete=ref.complete,
                palm_enabled=palm_enabled, palm_readiness=palm_readiness)


class ObserverActions:
    def __init__(self):
        self.last_press = -1e9
        self.reset_pending = False

    def dispatch(self, action, ref, now, pos_ok, palm=None):
        elapsed = now-self.last_press
        self.last_press = now
        if elapsed < .65:
            return ref, None
        if action == "reset":
            if not self.reset_pending:
                self.reset_pending = True
                return ref, "再点一次重新定向才会清空；其他操作取消。"
            self.reset_pending = False
            return DirectionReference(), "已重新开始；没有修改相机标定。"
        self.reset_pending = False
        if action == "palm" and palm is not None:
            palm.capture(now)
            return ref, "姿态基准已设；位置零点和方向轴不变。"
        if action == "advance" and ref.complete:
            return ref, "已完成，重复确认不会清除结果。"
        if not pos_ok:
            raise ValueError("当前腕部观测不可用；已有定向结果保留。")
        if action == "advance":
            return ref, ref.advance(now)
        if action == "origin" and ref.complete:
            ref.origin = ref.stable_point(now)
            return ref, "位置已归零；X/Y/Z方向不变。"
        return ref, None


def draw_palm_comparison(palm, now):
    plot = np.full((178, 340, 3), 35, np.uint8)
    center = np.array([170,96])
    projection = np.array([[.85,-.65,0],[.3,.4,-1]])
    def endpoint(v):
        return tuple(np.rint(center+projection@v*58).astype(int))
    if palm.reference is not None:
        for axis in np.eye(3):
            end = np.array(endpoint(axis))
            for f in np.arange(0,1,.18):
                a = np.rint(center+(end-center)*f).astype(int)
                b = np.rint(center+(end-center)*min(1,f+.09)).astype(int)
                cv2.line(plot, tuple(a), tuple(b), (165,165,165), 2)
    if palm.current(now):
        view = palm.reference.T if palm.reference is not None else np.eye(3)
        points = (palm.points-palm.points[1:].mean(axis=0))@view.T
        xy = np.rint(center+points@projection.T*850).astype(int)
        cv2.polylines(plot, [xy[[0,1,2,3]]], True, (95,95,95), 1)
        for axis, color, label in zip((view@palm.rotation).T, ((80,80,240),(70,220,70),(255,175,65)), "WFN"):
            end = endpoint(axis)
            cv2.arrowedLine(plot, tuple(center), end, color, 2)
            cv2.putText(plot, label, end, 0, .45, color, 1)
        cv2.circle(plot, tuple(center), 4, (255,255,255), 1)
    return plot


def render_observer(ref, latest, point, now, pos_ok, reason, notice, fonts, reset_pending=False, palm=None, calibration=None):
    from PIL import Image, ImageDraw
    font, heading = fonts
    canvas = np.full((800, 1200, 3), 27, np.uint8)
    if latest:
        for i, frame in enumerate(latest[:2]):
            img = cv2.resize(frame.image, (600, 338))
            for hand in frame.metadata["hand"]["points"]:
                for x, y in hand:
                    cv2.circle(img, (int(x*600/frame.image.shape[1]), int(y*338/frame.image.shape[0])), 2, (0,255,0), -1)
            if palm is not None and calibration is not None:
                overlay_palm(img, palm, calibration, i, now)
            canvas[:338, i*600:(i+1)*600] = img
    state = view_state(ref, now, pos_ok, reason, palm)
    displayed = ref.position(point) if pos_ok and ref.complete else None
    # Independent clipped plot surfaces cannot overwrite instructions or controls.
    views = [(np.array([[.85, -.65, 0], [.3, .4, -1]]), "3D"),
             (np.array([[1, 0, 0], [0, -1, 0]]), "XY"),
             (np.array([[1, 0, 0], [0, 0, -1]]), "XZ"),
             (np.array([[0, 1, 0], [0, 0, -1]]), "YZ")]
    extent = max(.3, float(np.max(np.abs(displayed)))*1.2) if displayed is not None else .3
    scale = 65/extent
    for i, (projection, title) in enumerate(views):
        width = 222 if palm is not None and i > 0 else 340
        plot = np.full((178, width, 3), 35, np.uint8)
        center = np.array([width//2, 94])
        def pixel(p):
            return tuple(np.rint(center+projection@p*scale).astype(int))
        cv2.putText(plot, title, (10, 23), 0, .6, (220,220,220), 1)
        for axis, color, label in zip(np.eye(3), [(80,80,240),(70,200,70),(240,160,60)], "XYZ"):
            if np.linalg.norm(projection@axis) < 1e-8:
                continue
            end = pixel(axis*extent*.8)
            cv2.arrowedLine(plot, tuple(center), end, color, 2)
            cv2.putText(plot, label, end, 0, .45, color, 1)
        if displayed is not None:
            cv2.circle(plot, pixel(displayed), 5, (255,255,255), -1)
        cv2.putText(plot, f"range +/-{extent*100:.0f} cm", (10,168), 0, .4, (165,165,165), 1)
        x, y = ((504,346) if i == 0 else (504+(i-1)*233,532)) if palm is not None else (504+(i%2)*348,346+(i//2)*186)
        canvas[y:y+178, x:x+width] = plot
    if palm is not None:
        canvas[346:524, 852:1192] = draw_palm_comparison(palm, now)
    pil = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil)
    def text(x, y, value, color="white", face=font, width=468):
        line = ""
        for char in value:
            if draw.textlength(line+char, font=face) > width:
                draw.text((x,y), line, font=face, fill=color)
                line, y = "", y+24
            line += char
        draw.text((x,y), line, font=face, fill=color)
    for x, label in ((10,"左相机"),(610,"右相机")):
        draw.rectangle((x-4, 4, x+115, 32), fill="#202020")
        text(x, 5, label)
    if palm is not None:
        text(862, 349, "掌姿态（独立于位置XYZ）", width=324)
        text(862, 497, "灰虚线=零姿态；彩色=当前" if palm.reference is not None else "先摆舒服的手姿，再设基准",
             "#bcbcbc", width=324)
    pose_first = palm is not None and ref.step == 0
    text(16, 350, "手掌定位与姿态" if pose_first else state["title"], "#76df9c" if ref.complete else "white", heading)
    text(16, 388, "摆舒服的C形手姿，点击设为姿态基准。" if pose_first else state["instruction"])
    text(16, 428, "X  本人右侧       Y  身体前方       Z  向上", "#c9d5df")
    if ref.complete:
        text(16, 458, "腕位置 / cm（定向已保留）", "#c9d5df")
        for i, label in enumerate("XYZ"):
            value = "--" if displayed is None else f"{displayed[i]*100:+.1f}"
            text(16+i*150, 484, f"{label}  {value}", face=heading, width=148)
        text(16, 520, "位置观测有效" if pos_ok else "位置暂不可用；定向结果保留", "#76df9c" if pos_ok else "#ffca7a")
    else:
        status = ["完成" if len(ref.endpoints)>=2 else "待确认",
                  "完成" if ref.basis is not None else "待确认",
                  "完成" if ref.complete else "待确认"]
        text(16, 467, "位置方向未设，不影响观察掌姿态。" if pose_first else "右移：%s    前移：%s    上移：%s" % tuple(status))
        text(16, 507, "需要位置XYZ时，再用空格开始定向。" if pose_first else state["readiness"],
             "#a6a6a6" if pose_first else "#76df9c" if state["advance_enabled"] else "#ffca7a")
    if palm is not None:
        angle = palm.angle_deg(now)
        baseline = "未设置" if palm.reference is None else "已保留"
        value = "--" if angle is None else f"{angle:.1f}°"
        text(16, 558, f"姿态基准：{baseline}   总旋转差：{value}", "#76df9c" if angle is not None else "#c9d5df")
        text(16, 589, state["palm_readiness"], "#a6c8de")
        text(16, 624, notice, "#ffca7a")
        text(16, 688, "白圈=指根中心；W掌宽 / F掌前 / N法向", "#a6a6a6")
    else:
        text(16, 605, notice, "#ffca7a")
        text(16, 682, "仅位置观察 · 无机器人 · 无录像", "#a6a6a6")
    labels = {"advance": "已完成" if ref.complete else "位置定向 [空格]" if pose_first else "确认本步 [空格]", "origin": "位置归零 [O]",
              "palm": "重设姿态基准" if palm is not None and palm.reference is not None else "设为姿态基准",
              "reset": "确认重新定向" if reset_pending else "重新定向", "quit": "退出 [Q]"}
    for name, bounds in BUTTONS.items():
        enabled = (state["advance_enabled"] if name == "advance" else state["origin_enabled"] if name == "origin" else
                   state["palm_enabled"] if name == "palm" else True)
        draw.rectangle(bounds, fill="#355b49" if enabled else "#343434", outline="#606060")
        text(bounds[0]+18, bounds[1]+15, labels[name], "white" if enabled else "#929292")
    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR), state


def run(root, config, dry_run=None, headless=False, duration=None):
    from PIL import ImageFont
    from .calibration_state import load_bound_calibration
    cal, _, _ = load_bound_calibration(root, config)
    cams, settings = config["cameras"], {**config["replay"], **config["live"]}
    rgbd_mode = config.get('position_input','stereo') == 'primary_rgbd'
    primary_role = cams.get('depth_role','left')
    if rgbd_mode and (not cams.get('depth') or primary_role != 'left'):
        raise ValueError('RGB-D observer currently requires left primary depth; no silent role switching')
    primary_frames = deque(maxlen=1)
    rgbd_counts = Counter()
    rgbd_inference_ms = []
    rgbd = None
    position_stamp = -1e9
    wrist = None
    if cal["camera_serials"] != [cams[r+"_serial"] for r in ("left", "right")] or cal["image_size"] != [cams["width"], cams["height"]]:
        raise ValueError("Camera roles/resolution differ from calibration")
    paired = PairBuffer(cams["queue_capacity"], cams["max_pair_delta_ms"], cams["max_age_ms"], lambda row: None)
    workers, source = {}, None
    ref = DirectionReference()
    palm = PalmReference(settings, config.get("observer", {}).get("palm_reference_duration_s", .6))
    latest, g, stamp = None, None, -1e9
    notice = "姿态可直接设基准，无需先做六步位置定向；不运行机器人、不录像。"
    if rgbd_mode:
        notice = '位置：左相机RGB-D腕表面代理；姿态：原双目。无机器人、不录像。'
    fonts = (ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 17),
             ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 23))
    actions, clicks = ObserverActions(), deque()
    window = "Hand coordinates - no robot - not recording"
    last_key = None
    if not headless:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(window, 1080, 720)
        def mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONUP:
                for name, (x0,y0,x1,y1) in BUTTONS.items():
                    if x0 <= x <= x1 and y0 <= y <= y1:
                        clicks.append(name)
                        break
        cv2.setMouseCallback(window, mouse)
    start, next_render = time.monotonic(), 0.
    try:
        def completed(role, frame):
            if rgbd_mode and role == primary_role:
                primary_frames.append(frame)
                result = frame.metadata['hand']['rgbd_position']
                rgbd_counts['frames'] += 1
                rgbd_counts['valid' if result['valid'] else result['reason']] += 1
                rgbd_inference_ms.append(frame.metadata['hand']['inference_ms'])
            paired.push(role,frame)
        for role in ("left", "right"):
            workers[role] = TrackingWorker(role, lambda: VideoHand(root/settings["hand_model"]), completed, lambda row: None,
                                           config['rgbd_position'] if rgbd_mode and role==primary_role else None)
        for worker in workers.values():
            if not worker.ready.wait(30):
                raise RuntimeError("Tracker startup timeout")
            worker.check()
        def on_frame(frame):
            role = frame.metadata["camera_role"]
            if frame.metadata["camera_serial"] != cams[role+"_serial"] or list(frame.image.shape[1::-1]) != cal["image_size"]:
                raise ValueError("Frame serial/resolution mismatch")
            workers[role].submit(frame)
        source = (RecordedCapture(root/dry_run, cams, lambda row: None, on_frame, duration) if dry_run else
                  DualCapture(cams, event=lambda row: None, on_frame=on_frame))
        source.start()
        start = time.monotonic()
        while duration is None or time.monotonic()-start < duration:
            now = time.monotonic()
            for worker in workers.values():
                worker.check()
            while source.get_pair() is not None:
                pass
            while (pair := paired.pop()) is not None:
                latest = pair
                g = geometry(*pair[:2], cal)
                stamp = min(pair[0].time_ms, pair[1].time_ms)/1000
                palm.observe(g, stamp)
                if not rgbd_mode and g["valid"] and g["ray_gap_m"][0] <= settings["position_ray_gap_m"] and g["ray_angle_sine"][0] >= settings["min_ray_angle_sine"]:
                    ref.add(stamp, g["points"][0])
                elif not rgbd_mode:
                    ref.history.clear()
            if rgbd_mode and primary_frames:
                primary = primary_frames.pop()
                rgbd = primary.metadata['hand']['rgbd_position']
                position_stamp = primary.time_ms/1000
                wrist = np.asarray(rgbd['position_m']) if rgbd['valid'] else None
                if wrist is not None:
                    ref.add(position_stamp,wrist)
                else:
                    ref.history.clear()
            fresh = now-stamp <= settings["stale_after_s"]
            pos_ok = fresh and g is not None and g["valid"] and g["ray_gap_m"][0] <= settings["position_ray_gap_m"] and g["ray_angle_sine"][0] >= settings["min_ray_angle_sine"]
            if rgbd_mode:
                pos_ok = now-position_stamp<=settings['stale_after_s'] and wrist is not None
            else:
                wrist = g['points'][0] if pos_ok else None
            if now < next_render:
                time.sleep(.002)
                continue
            next_render = now+1/settings["render_fps"]
            reason = ("观测过期，等待新帧；定向不会丢失。" if not fresh else
                      g["reason"] if g is not None and not g["valid"] else
                      "腕部双视角几何不一致；已有结果保留。")
            if rgbd_mode:
                reason = ('RGB-D观测过期；已有定向保留。' if now-position_stamp>settings['stale_after_s'] else
                          'RGB-D: '+(rgbd or {}).get('reason','waiting_for_depth'))
            canvas, state = render_observer(ref, latest, wrist if pos_ok else None,
                                            time.monotonic(), pos_ok, reason, notice, fonts, actions.reset_pending, palm, cal)
            if headless:
                continue
            cv2.imshow(window, canvas)
            key = cv2.waitKey(1) & 255
            action = clicks.popleft() if clicks else None
            if key != last_key:
                action = {32:"advance",ord('o'):"origin",ord('O'):"origin",ord('q'):"quit",ord('Q'):"quit"}.get(key, action)
            last_key = key
            if key in (ord('u'), ord('U'), ord('r'), ord('R'), ord('c'), ord('C')):
                notice = "旧U/R/C键已停用；定向只用空格，重来请点重新定向。"
            if action == "quit" or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            try:
                if action:
                    ref, message = actions.dispatch(action, ref, time.monotonic(), pos_ok, palm)
                    if message:
                        notice = message
            except ValueError as exc:
                notice = str(exc)
    finally:
        try:
            if source is not None:
                source.stop()
        finally:
            try:
                errors = []
                for worker in workers.values():
                    try:
                        worker.close()
                    except Exception as exc:
                        errors.append(str(exc))
                if errors:
                    raise RuntimeError("; ".join(errors))
            finally:
                paired.close()
                cv2.destroyAllWindows()
                if rgbd_mode and headless:
                    report=dict(primary_role=primary_role,counts=dict(rgbd_counts),
                        inference_ms_median=float(np.median(rgbd_inference_ms)) if rgbd_inference_ms else None,
                        no_robot=True,no_images_saved=True,
                        limits='Runtime check only; depth validity does not establish wrist accuracy')
                    out=root/'results/rgbd_check'; out.mkdir(parents=True,exist_ok=True)
                    (out/'observer.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
                    print(json.dumps(report))
