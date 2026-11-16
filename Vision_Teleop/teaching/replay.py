"""Offline hand-to-Mink wiring. Conservative diagnostic replay, not live control."""
from collections import Counter
from datetime import datetime
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def palm_rotation(points):
    wrist, index, middle, pinky = np.asarray(points, float)
    x, y = index-pinky, middle-wrist
    if not np.isfinite(points).all() or min(np.linalg.norm(x), np.linalg.norm(y)) < 1e-4:
        return None
    x /= np.linalg.norm(x)
    y -= x*np.dot(x, y)
    if np.linalg.norm(y) < .005:
        return None
    y /= np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


class SmoothTarget:
    def __init__(self, position, rotation, settings):
        self.position = np.asarray(position, float).copy()
        self.rotation = np.asarray(rotation, float).copy()
        self.settings = settings

    def update(self, position, rotation, dt):
        if dt <= 0:
            raise ValueError("dt must be positive")
        alpha = min(1., dt/self.settings["smoothing_time_s"])
        if position is not None:
            delta = np.asarray(position)-self.position
            norm = np.linalg.norm(delta)
            if not np.isfinite(norm):
                raise ValueError("Nonfinite position target")
            self.position += delta*min(alpha, self.settings["max_target_speed_m_s"]*dt/max(norm, 1e-12))
        if rotation is not None:
            rotation = np.asarray(rotation)
            if not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-6) or np.linalg.det(rotation) < .999:
                raise ValueError("Invalid rotation target")
            delta = Rotation.from_matrix(rotation@self.rotation.T).as_rotvec()
            angle = np.linalg.norm(delta)
            fraction = min(alpha, self.settings["max_target_angular_speed_rad_s"]*dt/max(angle, 1e-12))
            self.rotation = Rotation.from_rotvec(delta*fraction).as_matrix()@self.rotation


def task_orientation(mode, initial_rotation, mapped_rotation):
    """Task requirement, independent of observation validity; None means free."""
    if mode == "follow":
        return mapped_rotation
    if mode == "fixed":
        return initial_rotation.copy()
    if mode == "free":
        return None
    raise ValueError("orientation task must be follow, fixed or free")


def task_anchor(settings, home_position, home_rotation):
    """Explicit world task pose is independent of the robot mounting."""
    if settings is None:
        return home_position.copy(), home_rotation.copy()
    p = np.asarray(settings["position_world_m"], float)
    r = np.asarray(settings["rotation_world"], float)
    if p.shape != (3,) or not np.isfinite(p).all():
        raise ValueError("Finite world task position required")
    if r.shape != (3, 3) or not np.isfinite(r).all() or not np.allclose(r.T@r, np.eye(3), atol=1e-6) or np.linalg.det(r) < .999:
        raise ValueError("World task rotation must be proper, not mirrored")
    return p.copy(), r.copy()


def preposition(arm, target, position, rotation, settings, stream):
    """Separate simulation phase: no human frames or source clock consumed."""
    for key in ("timeout_s", "position_tolerance_m", "orientation_tolerance_deg", "stable_duration_s"):
        if not np.isfinite(settings[key]) or settings[key] <= 0:
            raise ValueError(f"Invalid preposition.{key}")
    stable = 0.
    last = None
    for step in range(int(np.ceil(settings["timeout_s"]/.01))):
        target.update(position, rotation, .01)
        arm.set_target(target.position, target.rotation)
        arm.step(.01)
        p, r = arm.pose()
        pe = float(np.linalg.norm(p-position))
        re = float(np.degrees(Rotation.from_matrix(r@rotation.T).magnitude()))
        within = pe <= settings["position_tolerance_m"] and re <= settings["orientation_tolerance_deg"]
        stable = stable+.01 if within else 0.
        last = dict(elapsed_s=(step+1)*.01, position_error_m=pe, orientation_error_deg=re,
                    stable_s=stable, actual_position=p.tolist(), actual_rotation=r.tolist(),
                    commanded_position=target.position.tolist(), commanded_rotation=target.rotation.tolist(),
                    joint_positions=arm.data.qpos.tolist())
        stream.write(json.dumps(last, allow_nan=False)+"\n")
        if stable+1e-9 >= settings["stable_duration_s"]:
            return dict(status="ready", **last)
    return dict(status="timeout", **last)


def run(root, config, robot, geometry_file=None, orientation_task="follow", mount_degrees=0):
    import mujoco
    from .robot import SimArm
    task_orientation(orientation_task, np.eye(3), None)
    s = config["replay"]
    if not np.isfinite(mount_degrees):
        raise ValueError("Finite mount angle required")
    if mount_degrees != 0 and not s.get("task_anchor"):
        raise ValueError("Nonzero mount requires replay.task_anchor in fixed world coordinates")
    if s.get("task_anchor"):
        task_anchor(s["task_anchor"], np.zeros(3), np.eye(3))
    for key in ("smoothing_time_s", "max_target_speed_m_s", "max_target_angular_speed_rad_s",
                "workspace_half_extent_m", "geometry_screen_px", "stale_after_s", "stop_after_s"):
        if not np.isfinite(s[key]) or s[key] <= 0:
            raise ValueError(f"Invalid replay.{key}")
    a = np.asarray(s["camera_to_task_rotation"], float)
    if a.shape != (3,3) or not np.allclose(a.T@a, np.eye(3)) or np.linalg.det(a) < .999:
        raise ValueError("Camera/task map must be a proper rotation")
    path = (root / (geometry_file or s["geometry_file"])).resolve()
    audit = json.loads((path.parent / "recording_check.json").read_text())
    calibration = (root / audit["calibration"]).resolve()
    if calibration != (root / config["calibration_file"]).resolve():
        raise ValueError("Replay geometry and configured calibration differ")
    session = root / audit["session"]
    metadata = [json.loads(line) for line in (session/"metadata.jsonl").read_text().splitlines()]
    frames = {r["frame_id"]:r for r in metadata if r["type"] == "video_frame"}
    pairs = {r["pair_id"]:r for r in metadata if r["type"] == "pair"}
    observations_path = root/audit["observations_file"] if audit.get("observations_file") else session/"metadata.jsonl"
    observations = {r["pair_id"]:r for line in observations_path.read_text().splitlines()
                    if (r:=json.loads(line))["type"] == "keypoints_2d"}
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows.sort(key=lambda r:r["common_host_time_s"])
    t0 = min(r["host_arrival_monotonic_ns"]/1e9 for r in frames.values())
    duration = max(r["host_arrival_monotonic_ns"]/1e9 for r in frames.values())-t0
    out = root/"results"/"hand_replay"/(datetime.now().strftime("%Y%m%d_%H%M%S_%f")+"_"+robot+"_"+orientation_task)
    out.mkdir(parents=True, exist_ok=False)
    arm = SimArm(robot, mount_degrees=mount_degrees)
    home_p, home_r = arm.pose()
    initial_p, initial_r = task_anchor(s.get("task_anchor"), home_p, home_r)
    target = SmoothTarget(home_p, home_r, s)
    human_p0 = human_r0 = None
    desired_p, desired_r = initial_p.copy(), initial_r.copy()
    source = None
    next_row, last_position_time = 0, -1e9
    states, log, error_text = Counter(), [], None
    preparation = dict(status="not_requested")
    caps = [cv2.VideoCapture(str(session/f"{role}.avi")) for role in ("left", "right")]
    writer = cv2.VideoWriter(str(out/"replay.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 20, (1280,480))
    if not writer.isOpened():
        for cap in caps: cap.release()
        raise RuntimeError("Cannot open replay video")
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(arm.model, camera)
    camera.lookat[:] = [0, 0, 1.25]
    camera.distance, camera.azimuth, camera.elevation = 2.5, 130, -20
    preview = np.zeros((480,640,3), np.uint8)
    last_drawn = None
    try:
        if s.get("task_anchor"):
            preparation = dict(status="error")
            with (out/"preposition.jsonl").open("w", encoding="utf-8") as stream:
                preparation = preposition(arm, target, initial_p, initial_r, s["preposition"], stream)
            if preparation["status"] != "ready":
                raise RuntimeError("Preposition timed out; human replay did not start")
        with mujoco.Renderer(arm.model, height=480, width=640) as renderer:
            for step in range(int(np.ceil(duration/.01))+1):
                t = step*.01
                while next_row < len(rows) and rows[next_row]["common_host_time_s"]-t0 <= t:
                    source = rows[next_row]
                    next_row += 1
                fresh = source is not None and t-(source["common_host_time_s"]-t0) <= s["stale_after_s"]
                position_ok = orientation_ok = False
                observed_r = mapped_r = None
                xyz = None
                if fresh and source.get("single_candidate_both"):
                    xyz = np.asarray(source["palm_points_left_camera_m"], float)
                    errors = np.asarray(source["reprojection_error_px"], float)
                    positive = np.asarray(source["positive_depth"], bool)
                    observed_r = palm_rotation(xyz)
                    has_score = source.get("per_point_confidence_supported", True)
                    position_ok = bool(positive[0] and np.isfinite(xyz[0]).all() and errors[0] <= s["geometry_screen_px"]
                                       and (not has_score or source.get("wrist_clear")))
                    orientation_ok = bool(positive.all() and np.isfinite(xyz).all() and
                                          np.all(errors <= s["geometry_screen_px"]) and
                                          (not has_score or source.get("palm_clear")))
                if position_ok:
                    if human_p0 is None: human_p0 = xyz[0].copy()
                    raw_p = initial_p+a@(s["position_scale"]*(xyz[0]-human_p0))
                    desired_p = np.clip(raw_p, initial_p-s["workspace_half_extent_m"], initial_p+s["workspace_half_extent_m"])
                    last_position_time = source["common_host_time_s"]-t0
                if orientation_ok:
                    hr = observed_r
                    orientation_ok = hr is not None
                    if orientation_ok:
                        if human_r0 is None: human_r0 = hr.copy()
                        mapped_r = a@hr@human_r0.T@a.T@initial_r
                requested_r = task_orientation(orientation_task, initial_r, mapped_r)
                if requested_r is not None:
                    desired_r = requested_r
                orientation_available = orientation_task != "follow" or orientation_ok
                if not position_ok:
                    state = "SAFE_STOP" if t-last_position_time > s["stop_after_s"] else "TARGET_HOLD"
                    reason = "position unavailable, stale, or geometry screen failed"
                elif not orientation_available:
                    state, reason = "POSE_HOLD", "orientation unavailable or geometry screen failed"
                else:
                    state, reason = "TASK_TRACKING", "explicit orientation task: " + orientation_task
                target.update(desired_p if position_ok else None, desired_r if position_ok and orientation_available and requested_r is not None else None, .01)
                if position_ok and orientation_task == "free":
                    target.rotation = arm.pose()[1]
                # Free orientation is a task choice, not a missing observation.
                # On position loss restore pose holding, preserving the stop behavior.
                arm.eef.set_orientation_cost(0.0 if position_ok and orientation_task == "free" else 1.0)
                arm.set_target(target.position, target.rotation)
                arm.step(.01)
                states[state] += 1
                if step % 5:
                    continue
                actual_p, actual_r = arm.pose()
                record = dict(time_s=t, state=state, reason=reason, source_pair_id=source["pair_id"] if source else None,
                    position_used=position_ok, orientation_used=position_ok and orientation_task == "follow" and orientation_ok,
                    orientation_task=orientation_task, observed_orientation_valid=orientation_ok,
                    observed_rotation=None if observed_r is None else observed_r.tolist(),
                    mapped_observed_rotation=None if mapped_r is None else mapped_r.tolist(),
                    task_rotation=None if requested_r is None else requested_r.tolist(),
                    orientation_constraint_active=orientation_task != "free" or not position_ok,
                    observed_wrist_m=xyz[0].tolist() if xyz is not None and np.isfinite(xyz[0]).all() else None,
                    desired_position=desired_p.tolist(), commanded_position=target.position.tolist(), actual_position=actual_p.tolist(),
                    commanded_rotation=target.rotation.tolist(), actual_rotation=actual_r.tolist(),
                    position_command_error_m=float(np.linalg.norm(actual_p-target.position)),
                    position_task_error_m=float(np.linalg.norm(actual_p-desired_p)) if position_ok else None,
                    orientation_task_error_deg=float(np.degrees(Rotation.from_matrix(actual_r@requested_r.T).magnitude())) if position_ok and requested_r is not None else None,
                    orientation_command_error_deg=float(np.degrees(Rotation.from_matrix(actual_r@target.rotation.T).magnitude())),
                    joint_positions=arm.data.qpos.tolist(), min_contact_distance_m=min((c.dist for c in arm.data.contact), default=None))
                log.append(record)
                if source and source["pair_id"] != last_drawn:
                    preview[:] = 0
                    obs = observations[source["pair_id"]]
                    pair = pairs[source["pair_id"]]
                    for i,role in enumerate(("left", "right")):
                        entry = frames[pair[role+"_frame_id"]]
                        caps[i].set(cv2.CAP_PROP_POS_FRAMES, entry["video_index"])
                        ok, image = caps[i].read()
                        if not ok: raise ValueError("Cannot decode indexed source image")
                        points = np.asarray(obs[role][0])
                        if points.shape == (1,21,2):
                            for point in points[0,[0,5,9,17]]:
                                cv2.circle(image, tuple(np.rint(point).astype(int)), 5, (0,255,0), -1)
                        preview[:180,i*320:(i+1)*320] = cv2.resize(image,(320,180))
                    last_drawn = source["pair_id"]
                panel = preview.copy()
                texts = [f"OFFLINE {robot} | {t:.2f}s", state, "Source images above: last received observation",
                         f"Position: {position_ok} | orientation task: {orientation_task}",
                         f"Command tracking error: {record['position_command_error_m']*1000:.1f} mm",
                         "Diagnostic 2px screen; input validity != task choice",
                         "Camera proxy axes; no task-frame calibration; no real robot"]
                for i,text in enumerate(texts):
                    cv2.putText(panel,text,(12,210+i*34),cv2.FONT_HERSHEY_SIMPLEX,.51,(230,230,230),1)
                renderer.update_scene(arm.data,camera=camera)
                rendered = cv2.cvtColor(renderer.render(),cv2.COLOR_RGB2BGR)
                writer.write(np.hstack([panel,rendered]))
    except Exception as exc:
        error_text = str(exc)
    finally:
        writer.release()
        for cap in caps: cap.release()
        with (out/"trajectory.jsonl").open("w",encoding="utf-8") as stream:
            for row in log: stream.write(json.dumps(row,allow_nan=False)+"\n")
        report = dict(robot=robot, orientation_task=orientation_task, source_geometry=str(path), source_observations=str(observations_path),
            mount_degrees=mount_degrees, task_anchor_mode="fixed_world" if s.get("task_anchor") else "robot_home_legacy",
            task_initial_position_world=initial_p.tolist(), task_initial_rotation_world=initial_r.tolist(),
            calibration=str(calibration), input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            calibration_sha256=hashlib.sha256(calibration.read_bytes()).hexdigest(), settings=s,
            video_frames=len(log), input_duration_s=duration, simulated_duration_s=sum(states.values())*.01,
            state_duration_s={k:v*.01 for k,v in states.items()}, error=error_text,
            preposition=preparation,
            human_initial_position_m=None if human_p0 is None else human_p0.tolist(),
            human_initial_rotation=None if human_r0 is None else human_r0.tolist(),
            robot_initial_position=home_p.tolist(), robot_initial_rotation=home_r.tolist(),
            max_position_task_error_m=max((r["position_task_error_m"] for r in log if r["position_task_error_m"] is not None),default=None),
            max_orientation_task_error_deg=max((r["orientation_task_error_deg"] for r in log if r["orientation_task_error_deg"] is not None),default=None),
            max_position_command_error_m=max((r["position_command_error_m"] for r in log),default=None),
            max_orientation_command_error_deg=max((r["orientation_command_error_deg"] for r in log),default=None),
            limits="Offline diagnostic only; 2px screen is not task acceptance; camera-proxy task axes; independent first-usable position/orientation references; holds are not valid demonstration; command tracking error is not human tracking accuracy")
        (out/"summary.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(report,indent=2))
    print(out)
    if error_text: raise RuntimeError(error_text)
