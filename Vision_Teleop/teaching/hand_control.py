"""Relative hand mapping; geometric consistency is not an accuracy guarantee."""
import cv2
import numpy as np
from OneEuroFilter import OneEuroFilter
from scipy.spatial.transform import Rotation

from .observation import triangulate
from .replay import palm_rotation


def geometry(left, right, calibration):
    points = [np.asarray(f.metadata["hand"]["points"], float) for f in (left, right)]
    if any(p.shape != (1, 21, 2) or not np.isfinite(p).all() for p in points):
        return dict(valid=False, reason=f"左 {len(points[0])} 只 / 右 {len(points[1])} 只，需各一只且点完整")
    xy = [p[0, [0, 5, 9, 17]] for p in points]
    c = {k: np.asarray(calibration[k], float) for k in ("K1", "D1", "K2", "D2", "R", "T")}
    xyz, positive = triangulate(*xy, calibration)
    if not positive[0]:
        return dict(valid=False, reason="腕部三维计算无效或在相机后方")
    rays = [np.column_stack((cv2.undistortPoints(p.reshape(-1,1,2), c[k], c[d]).reshape(-1,2), np.ones(4)))
            for p,k,d in zip(xy,("K1","K2"),("D1","D2"))]
    rays = [v/np.linalg.norm(v,axis=1)[:,None] for v in rays]
    rays[1] = rays[1]@c["R"]
    cross = np.cross(*rays)
    sine = np.linalg.norm(cross, axis=1)
    gap = np.abs(cross@(-c["R"].T@c["T"].reshape(3)))/np.maximum(sine,1e-12)
    projected = [cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),c["K1"],c["D1"])[0][:,0],
                 cv2.projectPoints(xyz,cv2.Rodrigues(c["R"])[0],c["T"],c["K2"],c["D2"])[0][:,0]]
    error = np.maximum(*[np.linalg.norm(p-q,axis=1) for p,q in zip(projected,xy)])
    return dict(valid=True, points=xyz, positive_depth=positive, ray_gap_m=gap, ray_angle_sine=sine, reprojection_px=error,
                rotation=palm_rotation(xyz), reason="Geometric proxy only, not measured physical accuracy")


class ResponsiveTarget:
    """Normal tracking has no 0.5s lag; only reacquisition blends over that time."""
    def __init__(self, position, rotation, settings):
        self.position, self.rotation = position.copy(), rotation.copy()
        self.s = settings
        self.active = [False, False]
        self.recovery = [0., 0.]
        self.starts = [position.copy(), rotation.copy()]
        self.ever = [False, False]

    def update(self, position, rotation, dt):
        for i, value in enumerate((position, rotation)):
            if value is None:
                self.active[i] = False
                continue
            current = self.position if i == 0 else self.rotation
            if not self.active[i]:
                self.starts[i] = current.copy()
                self.recovery[i] = 0. if self.ever[i] else self.s["recovery_time_s"]
            self.active[i] = self.ever[i] = True
            self.recovery[i] += dt
            fraction = min(1., self.recovery[i]/self.s["recovery_time_s"])
            if i == 0:
                wanted = self.starts[i]+fraction*(value-self.starts[i])
                delta = wanted-current
                self.position += delta*min(1., self.s["max_target_speed_m_s"]*dt/max(np.linalg.norm(delta),1e-12))
            else:
                wanted = Rotation.from_rotvec(Rotation.from_matrix(value@self.starts[i].T).as_rotvec()*fraction).as_matrix()@self.starts[i]
                delta = Rotation.from_matrix(wanted@current.T).as_rotvec()
                self.rotation = Rotation.from_rotvec(delta*min(1.,self.s["max_target_angular_speed_rad_s"]*dt/max(np.linalg.norm(delta),1e-12))).as_matrix()@current


class HandControl:
    def __init__(self, position, rotation, settings):
        self.p0, self.r0, self.s = position.copy(), rotation.copy(), settings
        for key in ("recovery_time_s", "max_target_speed_m_s", "max_target_angular_speed_rad_s", "filter_min_cutoff_hz",
                    "position_ray_gap_m", "orientation_ray_gap_m", "min_ray_angle_sine", "init_duration_s",
                    "init_position_range_m", "init_angle_deg", "workspace_half_extent_m", "stale_after_s", "stop_after_s"):
            if not np.isfinite(settings[key]) or settings[key] <= 0:
                raise ValueError(f"Invalid live setting: {key}")
        self.a = np.asarray(settings["camera_to_task_rotation"],float)
        if not np.allclose(self.a.T@self.a,np.eye(3)) or np.linalg.det(self.a) < .999:
            raise ValueError("Camera/task mapping must be a proper rotation")
        self.target = ResponsiveTarget(position,rotation,settings)
        self.confirmed = self.initialized = False
        self.history = []
        self.source_time = self.last_good = -1e9
        self.pos_ok = self.rot_ok = False
        self.desired_p, self.desired_r = position.copy(), rotation.copy()
        self.raw_p, self.raw_r = position.copy(), rotation.copy()
        self.source_pair_id = None
        self.filters = [OneEuroFilter(30,settings["filter_min_cutoff_hz"],settings["filter_beta"],1.) for _ in range(3)]
        self.reason = "请只展示右手，按 R 确认，再停稳约两秒"
        self.state = "WAITING_FOR_R"
        self.orientation_mode = settings.get("orientation_mode", "clutch")
        if self.orientation_mode not in ("clutch", "continuous"):
            raise ValueError("Unknown orientation mode")
        self.observed_r = None
        self.clutch_reference = None
        from .gripper import PALM_TO_TOOL
        self.alignment_required = bool(settings.get("init_alignment_required",False) and
                                       settings.get("camera_to_task_calibrated",False))
        self.guide_rotation = self.a.T@self.r0@PALM_TO_TOOL.T
        self.guide_wrist = None
        self.alignment_error_deg = None
        self.measured_palm_rotation = None
        self.position_rejection = "尚无双路观测"

    def observe(self, g, timestamp):
        if timestamp <= self.source_time:
            return False
        self.source_time = timestamp
        self.pos_ok = self.rot_ok = False
        self.position_rejection = g.get("reason", "双路观测无效")
        if g["valid"]:
            self.pos_ok = bool(g["ray_gap_m"][0] <= self.s["position_ray_gap_m"] and g["ray_angle_sine"][0] >= self.s["min_ray_angle_sine"])
            self.rot_ok = bool(self.pos_ok and g["rotation"] is not None and np.isfinite(g["points"]).all() and max(g["ray_gap_m"]) <= self.s["orientation_ray_gap_m"]
                               and min(g["ray_angle_sine"]) >= self.s["min_ray_angle_sine"])
            if g["ray_gap_m"][0] > self.s["position_ray_gap_m"]:
                self.position_rejection = (f"双侧腕点不一致：视线间距 {g['ray_gap_m'][0]*1000:.1f}mm，"
                                           f"上限 {self.s['position_ray_gap_m']*1000:.0f}mm")
            elif g["ray_angle_sine"][0] < self.s["min_ray_angle_sine"]:
                self.position_rejection = "腕部双视线交角不足"
            else:
                self.position_rejection = ""
        self.guide_wrist = g["points"][0].copy() if g["valid"] else None
        self.measured_palm_rotation = g["rotation"].copy() if self.rot_ok else None
        self.alignment_error_deg = (float(np.degrees(Rotation.from_matrix(g["rotation"]@self.guide_rotation.T).magnitude()))
                                    if self.alignment_required and self.rot_ok else None)
        if not self.initialized:
            if not self.rot_ok:
                self.history.clear()
                if not g["valid"]:
                    self.reason = "双侧需各看见一只完整手："+g.get("reason", "观测不可用")
                elif g["rotation"] is None:
                    self.reason = "掌部方向不可用，请让两侧看清手掌"
                elif min(g["ray_angle_sine"]) < self.s["min_ray_angle_sine"]:
                    self.reason = "双视线交角不足，深度不稳定；请先反馈，不要反复尝试"
                else:
                    self.reason = (f"几何检查未通过：腕 {g['ray_gap_m'][0]*1000:.1f}/"
                                   f"{self.s['position_ray_gap_m']*1000:.0f}mm，掌 "
                                   f"{max(g['ray_gap_m'])*1000:.1f}/{self.s['orientation_ray_gap_m']*1000:.0f}mm")
                return
            if self.alignment_required and self.alignment_error_deg > self.s["init_alignment_max_deg"]:
                self.history.clear()
                self.reason = f"先对齐夹爪方向：当前偏差 {self.alignment_error_deg:.0f}°；沿绿/黄箭头摆手"
                return
            if not self.confirmed:
                self.reason = ("方向已接近：按 R 确认，再停稳两秒开始" if self.alignment_required else
                               "摆好舒服的手姿，按 R 将当前姿态设为起点")
                return
            p, r = g["points"][0], g["rotation"]
            if self.history and (timestamp-self.history[-1][0] > self.s["stale_after_s"] or
                np.linalg.norm(p-self.history[0][1]) > self.s["init_position_range_m"] or
                Rotation.from_matrix(r@self.history[0][2].T).magnitude() > np.radians(self.s["init_angle_deg"])):
                self.history.clear()
            self.history.append((timestamp,p.copy(),r.copy()))
            elapsed = timestamp-self.history[0][0]
            self.reason = f"已收到 R；停稳进度 {elapsed:.1f}/{self.s['init_duration_s']:.1f} 秒"
            if elapsed < self.s["init_duration_s"]:
                return
            self.hp0 = np.mean([h[1] for h in self.history],axis=0)
            self.hr0 = Rotation.from_matrix(np.array([h[2] for h in self.history])).mean().as_matrix()
            self.history.clear()
            self.initialized = True
        if self.pos_ok:
            raw = self.p0+self.a@(self.s["position_scale"]*(g["points"][0]-self.hp0))
            self.raw_p = raw.copy()
            self.desired_p = np.clip([f(v,timestamp) for f,v in zip(self.filters,raw)],self.p0-self.s["workspace_half_extent_m"],self.p0+self.s["workspace_half_extent_m"])
            self.last_good = timestamp
        if self.rot_ok:
            self.observed_r = g["rotation"].copy()
            if self.orientation_mode == "continuous":
                self.desired_r = self.a@g["rotation"]@self.hr0.T@self.a.T@self.r0
                self.raw_r = self.desired_r.copy()

    def step(self, now, dt, orientation_pressed=False):
        fresh = now-self.source_time <= self.s["stale_after_s"]
        p = self.initialized and self.pos_ok and fresh
        r = p and self.rot_ok
        if self.orientation_mode == "clutch":
            r = r and orientation_pressed
            if r:
                if self.clutch_reference is None:
                    self.clutch_reference = (self.observed_r.copy(), self.target.rotation.copy())
                human_start, robot_start = self.clutch_reference
                self.desired_r = self.a@self.observed_r@human_start.T@self.a.T@robot_start
                self.raw_r = self.desired_r.copy()
            else:
                # Release or lost focus/observations freezes the reached command,
                # not the farther desired orientation still waiting on rate limits.
                self.clutch_reference = None
                self.desired_r = self.target.rotation.copy()
                self.raw_r = self.desired_r.copy()
        if not self.initialized:
            self.state = "INITIALIZING" if self.confirmed else "WAITING_FOR_R"
            if self.confirmed and not fresh:
                self.reason = "已收到 R，等待新的双路识别结果"
        elif not p:
            self.state = "SAFE_STOP" if now-self.last_good > self.s["stop_after_s"] else "TARGET_HOLD"
            rejection = (f"双路观测已过期 {(now-self.source_time)*1000:.0f}ms" if not fresh else
                         self.position_rejection)
            self.reason = rejection+"；已保持仿真目标"
        else:
            self.state = "FULL_TRACKING" if r else "POSE_HOLD"
            self.reason = "初始化完成，正在跟随右手" if r else "位置继续跟随；掌部姿态暂时保持"
            if self.orientation_mode == "clutch":
                self.state = "ORIENTATION_ADJUSTING" if r else "POSITION_TRACKING"
                self.reason = ("Shift 已按住：位置继续跟随，转掌调整姿态" if r else
                               "只跟随位置；按住 Shift 转掌，松开即保持姿态")
                if orientation_pressed and not self.rot_ok:
                    self.reason = "Shift 已收到，但掌部观测不可用；姿态保持"
        self.target.update(self.desired_p if p else None,self.desired_r if r else None,dt)
        if r and self.target.recovery[1] < self.s["recovery_time_s"]:
            self.state = "ORIENTATION_RECOVERING"
        return p,r
