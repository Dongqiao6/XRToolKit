"""One live entry; recorded-camera dry runs use the same tracker/control path."""
from collections import Counter
import hashlib
import json
import threading
import time

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from .capture import DualCapture, Frame, PairBuffer
from .hand_tracking import TrackingWorker, VideoHand
from .hand_control import HandControl, geometry
from .recording import Recorder, PreviewOnly


def orientation_key_held():
    """Read Shift only while one of this process's windows has focus."""
    import os
    if os.name != "nt":
        return False
    import ctypes
    from ctypes import wintypes
    user32 = ctypes.windll.user32
    user32.GetForegroundWindow.restype = wintypes.HWND
    user32.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
    pid = wintypes.DWORD()
    user32.GetWindowThreadProcessId(user32.GetForegroundWindow(),ctypes.byref(pid))
    return pid.value == os.getpid() and bool(user32.GetAsyncKeyState(0x10) & 0x8000)


def finite_json(value):
    if isinstance(value,np.ndarray): return finite_json(value.tolist())
    if isinstance(value,dict): return {k:finite_json(v) for k,v in value.items()}
    if isinstance(value,list): return [finite_json(v) for v in value]
    return None if isinstance(value,float) and not np.isfinite(value) else value


class RecordedCapture:
    def __init__(self, session, config, event, on_frame, duration=None, clock_ns=time.monotonic_ns, precise_wait=False):
        self.clock_ns = clock_ns
        self.precise_wait = precise_wait
        if precise_wait and clock_ns is not time.perf_counter_ns:
            raise ValueError('Precise replay wait requires the perf_counter clock domain')
        rows = [json.loads(line) for line in (session/"metadata.jsonl").read_text().splitlines()]
        self.entries = {role:sorted([r for r in rows if r["type"] == "video_frame" and r["role"] == role],key=lambda r:r["video_index"])
                        for role in ("left","right")}
        self.t0 = min(r["host_arrival_monotonic_ns"] for rows in self.entries.values() for r in rows)
        self.buffer = PairBuffer(config["queue_capacity"],config["max_pair_delta_ms"],config["max_age_ms"],event,clock_ns=clock_ns)
        self.session, self.on_frame, self.duration = session,on_frame,duration
        self.stop_event = threading.Event()
        self.threads, self.errors = [],[]
        self.done = {r:threading.Event() for r in self.entries}

    def start(self):
        self.start_ns = self.clock_ns()+100_000_000
        for role in self.entries:
            t = threading.Thread(target=self._run,args=(role,),name=f"recorded-{role}",daemon=True)
            self.threads.append(t)
            t.start()
        return self

    def _run(self,role):
        cap = cv2.VideoCapture(str(self.session/f"{role}.avi"))
        from .wilor_timing import ControlWait
        waiter=ControlWait() if self.precise_wait else None
        try:
            for index,row in enumerate(self.entries[role]):
                if row["video_index"] != index:
                    raise ValueError("Non-contiguous recorded video index")
                offset = (row["host_arrival_monotonic_ns"]-self.t0)/1e9
                if self.duration is not None and offset > self.duration:
                    break
                ok,image = cap.read()
                if not ok:
                    raise ValueError("Cannot decode indexed recording")
                depth = None
                if row.get('depth_filename'):
                    depth_path = (self.session/row['depth_filename']).resolve()
                    if not depth_path.is_relative_to(self.session.resolve()):
                        raise ValueError('Depth recording path escapes session')
                    depth = cv2.imread(str(depth_path),cv2.IMREAD_UNCHANGED)
                    if depth is None or depth.dtype!=np.uint16 or depth.shape!=image.shape[:2]:
                        raise ValueError('Missing or invalid recorded depth')
                target_ns = self.start_ns+int(offset*1e9)
                if waiter:
                    target=target_ns/1e9
                    while not self.stop_event.is_set() and time.perf_counter()<target:
                        waiter.wait(min(target,time.perf_counter()+.01),self.stop_event)
                else:
                    self.stop_event.wait(max(0.,(target_ns-self.clock_ns())/1e9))
                if self.stop_event.is_set():
                    break
                frame = Frame(image,{**{k:v for k,v in row.items() if k != "type"},"recorded_host_arrival_monotonic_ns":row["host_arrival_monotonic_ns"],
                                     "host_arrival_monotonic_ns":target_ns,"host_clock":self.clock_ns.__name__,
                                     "playback_wait_backend":waiter.info if waiter else dict(backend='legacy_event_wait'),
                                     "playback_dispatch_monotonic_ns":self.clock_ns()},depth)
                self.buffer.push(role,frame)
                self.on_frame(frame)
        except Exception as exc:
            self.errors.append(f"{role}: {exc}")
        finally:
            cap.release()
            if waiter:waiter.close()
            self.done[role].set()

    def check(self):
        if self.errors:
            raise RuntimeError("; ".join(self.errors))

    def get_pair(self):
        self.check()
        return self.buffer.pop()

    def stop(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join(5)
            if thread.is_alive():
                raise RuntimeError("Recorded source did not stop")
        self.buffer.close()


def run(root,config,robot,dry_run=None,headless=False,duration=None,record_replay_frames=False,no_record=False):
    if config.get('position_input','stereo') == 'primary_rgbd':
        raise ValueError('RGB-D input is currently enabled for observe only; robot integration remains paused')
    import mujoco
    import mujoco.viewer
    from PIL import Image, ImageDraw, ImageFont
    from .robot import SimArm
    settings = {**config["replay"],**config["live"]}
    calibration_path = root/config["calibration_file"]
    from .calibration_state import load_bound_calibration
    calibration, _, _ = load_bound_calibration(root, config)
    if calibration["camera_serials"] != [config["cameras"][r+"_serial"] for r in ("left","right")]:
        raise ValueError("Camera roles differ from calibration")
    if calibration["image_size"] != [config["cameras"]["width"],config["cameras"]["height"]]:
        raise ValueError("Capture resolution differs from calibration")
    arm = SimArm(robot,with_gripper=robot == "franka_fr3" and settings.get("show_gripper",False))
    settings["init_alignment_required"] = arm.with_gripper
    control = HandControl(*arm.pose(),settings)
    recorder = PreviewOnly() if no_record else Recorder(root/"results"/"live_hand",config)
    recorder.record_video.set()
    recorder.event(dict(type="provenance",robot=robot,dry_run=str(dry_run) if dry_run else None,
                        settings=settings,calibration=str(calibration_path),calibration_sha256=hashlib.sha256(calibration_path.read_bytes()).hexdigest(),
                        observation="MediaPipe VIDEO, two independent instances; no landmark confidence available",
                        timebase="Host arrival proxy, NOT exposure time; dry run reanchors recorded arrivals"))
    counts = Counter()
    timing, controls = [],[]
    def event(row):
        counts[row["type"]] += 1
        recorder.event(row)
    paired = PairBuffer(config["cameras"]["queue_capacity"],config["cameras"]["max_pair_delta_ms"],config["cameras"]["max_age_ms"],
                        lambda row:event({**row,"type":"processed_"+row["type"]}))
    workers, source, renderer, viewer = {},None,None,None
    started = time.monotonic()
    error = None
    lock = threading.Lock()
    stop_control = threading.Event()
    control_errors = []
    control_thread = None
    confirm_request = threading.Event()
    exit_request = threading.Event()
    font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc",16)
    latest = None
    state_snapshot = {}

    def simulation_loop():
        deadline = time.perf_counter()
        try:
            while not stop_control.is_set():
                now = time.monotonic()
                with lock:
                    pressed = False if headless else orientation_key_held()
                    used = control.step(now,.01,pressed)
                    arm.set_target(control.target.position,control.target.rotation)
                    arm.step(.01)
                    p,r = arm.pose()
                    row = dict(type="robot",host_monotonic_s=now,time_s=now-started,state=control.state,reason=control.reason,
                               initialized=control.initialized,position_used=bool(used[0]),orientation_used=bool(used[1]),
                               orientation_mode=control.orientation_mode,orientation_clutch_pressed=pressed,
                               source_pair_id=control.source_pair_id,mapped_human_position=control.raw_p.tolist(),
                               mapped_human_rotation=control.raw_r.tolist(),
                               source_age_ms=(now-control.source_time)*1000 if control.source_time > 0 else None,
                               desired_position=control.desired_p.tolist(),commanded_position=control.target.position.tolist(),actual_position=p.tolist(),
                               commanded_rotation=control.target.rotation.tolist(),actual_rotation=r.tolist(),
                               position_error_m=float(np.linalg.norm(p-control.target.position)),
                               orientation_error_deg=float(np.degrees(Rotation.from_matrix(r@control.target.rotation.T).magnitude())),
                               joint_positions=arm.data.qpos.tolist())
                    state_snapshot.clear()
                    state_snapshot.update(row)
                controls.append(row)
                recorder.event(row)
                deadline += .01
                delay = deadline-time.perf_counter()
                if delay < 0:
                    counts["control_deadline_miss"] += 1
                    deadline = time.perf_counter()
                else:
                    stop_control.wait(delay)
        except Exception as exc:
            control_errors.append(str(exc))

    try:
        for role in ("left","right"):
            workers[role] = TrackingWorker(role,lambda:VideoHand(root/settings["hand_model"]),paired.push,event)
        for worker in workers.values():
            if not worker.ready.wait(30):
                raise RuntimeError("Hand tracker startup timeout")
            worker.check()
        def on_frame(frame):
            if list(frame.image.shape[1::-1]) != calibration["image_size"] or frame.metadata["camera_serial"] != config["cameras"][frame.metadata["camera_role"]+"_serial"]:
                raise ValueError("Frame serial/resolution mismatch")
            if dry_run is None or record_replay_frames:
                recorder.frame(frame)
            workers[frame.metadata["camera_role"]].submit(frame)
        source = (RecordedCapture(root/dry_run,config["cameras"],event,on_frame,duration) if dry_run else
                  DualCapture(config["cameras"],event=event,on_frame=on_frame))
        renderer = mujoco.Renderer(arm.model,height=480,width=640)
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(arm.model,camera)
        camera.lookat[:] = (control.p0+np.array([0,0,1.1]))/2
        camera.distance,camera.azimuth,camera.elevation = 1.7,180,-25
        reference_image = None
        if arm.with_gripper:
            closeup = mujoco.MjvCamera()
            mujoco.mjv_defaultFreeCamera(arm.model,closeup)
            closeup.lookat[:] = control.p0-control.r0[:,2]*.04
            closeup.distance,closeup.azimuth,closeup.elevation = .40,180,-20
            renderer.update_scene(arm.data,camera=closeup)
            reference_image = cv2.resize(cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR),(224,168))
        if not headless:
            def viewer_key(key):
                if key in (ord('r'),ord('R')): confirm_request.set()
                if key in (ord('q'),ord('Q')): exit_request.set()
            viewer = mujoco.viewer.launch_passive(arm.model,arm.data,key_callback=viewer_key)
            viewer.cam.lookat[:] = camera.lookat
            viewer.cam.distance,viewer.cam.azimuth,viewer.cam.elevation = camera.distance,camera.azimuth,camera.elevation
        if dry_run:
            control.confirmed = True
            event(dict(type="right_hand_confirmation",mode="simulated_for_recorded_dry_run"))
        started = time.monotonic()
        source.start()
        control_thread = threading.Thread(target=simulation_loop,name="mink-control",daemon=True)
        control_thread.start()
        next_render,finished_at = 0.,None
        while True:
            now = time.monotonic()
            if exit_request.is_set(): break
            if confirm_request.is_set():
                with lock:
                    control.confirmed = True
                confirm_request.clear()
                event(dict(type="right_hand_confirmation",mode="viewer_key"))
            recorder.check()
            source.check()
            for worker in workers.values():
                worker.check()
            if control_errors:
                raise RuntimeError(control_errors[0])
            while source.get_pair() is not None:
                pass
            while (pair:=paired.pop()) is not None:
                left,right,info = pair
                g = geometry(left,right,calibration)
                timestamp = (left.time_ms+right.time_ms)/2000
                with lock:
                    accepted = control.observe(g,timestamp)
                    if accepted is False:
                        event(dict(type="control_observation_skipped",pair_id=info["pair_id"],reason="non_increasing_host_pair_time"))
                        continue
                    control.source_pair_id = info["pair_id"]
                age = (time.monotonic()-min(left.time_ms,right.time_ms)/1000)*1000
                timing.append(dict(time_s=now-started,age_ms=age,left_ms=left.metadata["hand"]["inference_ms"],right_ms=right.metadata["hand"]["inference_ms"]))
                event(dict(type="geometry",**finite_json(g),
                           pair_id=info["pair_id"],source_host_monotonic_s=timestamp,age_ms=age))
                latest = pair
            if time.perf_counter() >= next_render:
                next_render = max(next_render+1/settings["render_fps"],time.perf_counter())
                with lock:
                    renderer.update_scene(arm.data,camera=viewer.cam if viewer else camera)
                    for point,color in ((control.desired_p,[1,.4,0,1]),(control.target.position,[0,1,1,1])):
                        scene = renderer.scene
                        if scene.ngeom < scene.maxgeom:
                            mujoco.mjv_initGeom(scene.geoms[scene.ngeom],mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.008]*3),point,np.eye(3).ravel(),np.array(color,dtype=np.float32))
                            scene.ngeom += 1
                    snapshot = dict(state_snapshot)
                    initializing = not control.initialized
                    guide_wrist = None if control.guide_wrist is None else control.guide_wrist.copy()
                    arrow_rotation = (control.guide_rotation.copy() if control.alignment_required else
                                      None if control.measured_palm_rotation is None else control.measured_palm_rotation.copy())
                    if viewer:
                        viewer.sync()
                sim = cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR)
                panel = np.zeros((480,640,3),np.uint8)
                if latest:
                    for i,f in enumerate(latest[:2]):
                        image = cv2.resize(f.image,(320,180))
                        for hand in f.metadata["hand"]["points"]:
                            for x,y in hand:
                                cv2.circle(image,(int(x*320/f.image.shape[1]),int(y*180/f.image.shape[0])),2,(0,255,0),-1)
                        if initializing and arm.with_gripper and guide_wrist is not None and arrow_rotation is not None:
                            from .gripper import guide_arrows
                            guide_arrows(image,guide_wrist,arrow_rotation,calibration,i)
                        panel[:180,i*320:(i+1)*320] = image
                texts = ["PREVIEW ONLY - NOT SAVING | R: initialize | Q: exit" if no_record else
                         ("RECORDED INPUT DRY RUN" if dry_run else "LIVE RIGHT HAND | R: initialize | Q: finish"),
                         snapshot.get("state",control.state),snapshot.get("reason",control.reason),
                         f"Processed pairs: {len(timing)/max(now-started,.01):.1f} Hz | current source age: {snapshot.get('source_age_ms',0):.0f} ms" if timing and snapshot.get('source_age_ms') is not None else "Waiting for paired observations",
                         f"Command error: {snapshot.get('position_error_m',0)*1000:.1f} mm / {snapshot.get('orientation_error_deg',0):.1f} deg",
                         "Orange: desired | cyan: bounded command | arm: actual",
                         "Scale 0.5; optical right/down/forward -> -Y/-Z/+X",
                         "Geometry proxy is NOT accuracy or real-robot safety"]
                for i,text in enumerate(texts):
                    if i != 2:
                        cv2.putText(panel,text,(8,205+i*33),cv2.FONT_HERSHEY_SIMPLEX,.45,(240,240,240),1)
                canvas = Image.fromarray(panel)
                draw = ImageDraw.Draw(canvas)
                reason = texts[2]
                draw.text((8,253),reason[:45],font=font,fill=(240,240,240))
                if len(reason)>45: draw.text((8,272),reason[45:90],font=font,fill=(240,240,240))
                if initializing and reference_image is not None:
                    draw.rectangle((0,292,639,479),fill=(0,0,0))
                    canvas.paste(Image.fromarray(reference_image),(408,305))
                    instructions = (("1. 手呈 C 形，想象握着右侧夹爪", "2. 掌前方沿绿箭头，拇指侧沿黄箭头",
                                     "3. 对齐后按 R，再停稳约两秒", "开始后按住 Shift 调姿态；Q 退出") if control.alignment_required else
                                    ("1. 参考夹爪，摆舒服的 C 形手姿", "2. 箭头是观测轴，不是要求你对齐的方向",
                                     "3. 按 R 设为起点，再停稳约两秒", "绿：腕到指根；黄：掌宽；Shift 调姿态"))
                    for i,text in enumerate(instructions):
                        draw.text((8,310+36*i),text,font=font,fill=(240,240,240))
                    draw.text((408,291),"起始夹爪（固定参考）",font=font,fill=(240,240,240))
                panel = np.asarray(canvas).copy()
                composite = np.hstack([panel,sim])
                counts["rendered"] += 1
                recorder.frame(Frame(composite,dict(camera_role="preview",frame_id=f"preview:{counts['rendered']}",
                    host_arrival_monotonic_ns=time.monotonic_ns(),video_fps=settings["render_fps"])))
                if not headless:
                    cv2.imshow("Right hand to simulation",composite)
                    key = cv2.waitKey(1)&255
                    if key in (ord('q'),ord('Q')) or not viewer.is_running():
                        break
                    if key in (ord('r'),ord('R')):
                        with lock:
                            control.confirmed = True
                        event(dict(type="right_hand_confirmation",mode="user_key"))
                    if key in (ord('1'),ord('2'),ord('3')):
                        viewer.cam.azimuth = {ord('1'):180,ord('2'):135,ord('3'):90}[key]
            if dry_run and all(e.is_set() for e in source.done.values()):
                finished_at = finished_at or now
                if now-finished_at > .5:
                    break
            if not dry_run and duration is not None and now-started >= duration:
                break
            time.sleep(.001)
    except BaseException as exc:
        error = str(exc) or type(exc).__name__
    finally:
        stop_control.set()
        if control_thread:
            control_thread.join(10)
            if control_thread.is_alive():
                error = (error or "")+" control thread did not stop"
        cleanup = ([source.stop] if source else [])+[w.close for w in workers.values()]+[paired.close]
        if viewer: cleanup.append(viewer.close)
        if renderer: cleanup.append(renderer.close)
        cleanup.extend([cv2.destroyAllWindows,recorder.close])
        for close in cleanup:
            try: close()
            except Exception as exc: error = (error or "")+"; "+str(exc)
        elapsed = max((r["time_s"] for r in controls),default=time.monotonic()-started)
        def stat(values):
            return {"median":float(np.median(values)),"p95":float(np.percentile(values,95)),"max":float(np.max(values))} if values else None
        report = dict(error=error,dry_run=str(dry_run) if dry_run else None,robot=robot,elapsed_s=elapsed,
                      raw_video_recorded=dry_run is None or record_replay_frames,
                      processed_pairs=len(timing),processed_pair_hz=len(timing)/max(elapsed,.01),counts=dict(counts),
                      observation_interval_hz=(len(timing)-1)/(timing[-1]["time_s"]-timing[0]["time_s"]) if len(timing)>1 else None,
                      initialized=control.initialized,age_ms=stat([r["age_ms"] for r in timing]),
                      inference_ms={k:stat([r[k+"_ms"] for r in timing]) for k in ("left","right")},
                      control_hz=len(controls)/max(elapsed,.01),states=dict(Counter(r["state"] for r in controls)),
                      max_orientation_error_deg=max((r["orientation_error_deg"] for r in controls),default=None),
                      max_position_error_m=max((r["position_error_m"] for r in controls),default=None),
                      processed_dispositions=dict(paired.counts),recorder_drops=dict(recorder.drops),
                      limits="Dry-run timings include recorded playback scheduling, not live camera or true motion-to-photon latency. Ray gap is a consistency proxy, not an accuracy bound. No human usability PASS.")
        if not no_record:
            (recorder.path/"summary.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
            print(json.dumps(report,indent=2))
            print(recorder.path)
    if error:
        if not headless:
            import ctypes
            ctypes.windll.user32.MessageBoxW(None,"体验已停止，相机已释放。请把以下错误发给我：\n"+error,
                                            "运行错误（未静默退出）",0x10)
        raise RuntimeError(error)
    if no_record:
        return None
    from .live_report import summarize
    summarize(recorder.path)
    return recorder.path
