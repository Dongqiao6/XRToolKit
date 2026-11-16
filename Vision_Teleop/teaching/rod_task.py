"""Approved rod task: explicit phases, path audit, and local endpoint priority."""
import json
from pathlib import Path

import mink
import numpy as np
from scipy.spatial.transform import Rotation

from .position_priority import CartesianPosition, PreservePose
from .tool_axis import ToolAxis


def load_task(root):
    return json.loads((Path(root)/'config/rod_task.json').read_text())


def axis_angle(a,b):
    a,b=np.asarray(a,float),np.asarray(b,float)
    if a.shape!=(3,) or b.shape!=(3,) or not np.isfinite([a,b]).all():
        raise ValueError('Finite 3D axes required')
    if not np.isclose(np.linalg.norm(a),1.) or not np.isclose(np.linalg.norm(b),1.):
        raise ValueError('Unit axes required')
    return float(np.degrees(np.arccos(np.clip(a@b,-1,1))))


def axis_rotation(current,axis):
    """Minimal rotation from current axis. Reject the ambiguous antipodal jump."""
    axis=np.asarray(axis,float);old=current[:,2]
    axis_angle(old,axis)  # Validate, but avoid arccos round-off at parallel axes.
    cross=np.cross(old,axis);norm=np.linalg.norm(cross);dot=float(old@axis)
    if norm<1e-12:
        if dot>0:return current.copy()
        raise ValueError('Antipodal tool axis is ambiguous; target rejected')
    angle=np.arctan2(norm,dot)
    return Rotation.from_rotvec(cross/norm*angle).as_matrix()@current


def solve_axis_first(arm,dt):
    """Preserve primary C translation + axis increments before posture preference.

    This is a local equality, not a nonlinear/global guarantee. The axis has
    rank two: use SVD to remove the redundant equality row before the QP.
    """
    target=arm.eef.transform_target_to_world
    position=CartesianPosition(arm.site,target.translation())
    axis=ToolAxis(arm.site,target.rotation().as_matrix()[:,2])
    v1=mink.solve_ik(arm.configuration,[position,axis],dt,'daqp',limits=arm.limits,
                     safety_break=True,primal_tol=1e-9)
    j=np.vstack([position.compute_jacobian(arm.configuration),axis.compute_jacobian(arm.configuration)])
    u,s,_=np.linalg.svd(j,full_matrices=False)
    rank=int(np.sum(s>1e-10))
    reduced=u[:,:rank].T@j
    preserve=PreservePose(reduced,reduced@v1*dt)
    preserve.k=rank
    fallback=False
    try:
        v=mink.solve_ik(arm.configuration,[arm.posture],dt,'daqp',limits=arm.limits,
                        constraints=[preserve],safety_break=True,primal_tol=1e-9)
    except mink.NoSolutionFound:
        v=v1;fallback=True
    residual=float(np.linalg.norm(j@(v-v1)*dt))
    violation=0.
    for limit in arm.limits:
        c=limit.compute_qp_inequalities(arm.configuration,dt)
        if not c.inactive:violation=max(violation,float(np.max(c.G@(v*dt)-c.h)))
    if residual>1e-7 or violation>1e-7:raise RuntimeError('Rod endpoint priority/safety constraint violation')
    arm.priority_diagnostics=dict(max_inequality_violation=violation,endpoint_increment_preservation=residual,
        endpoint_rank=rank,secondary_fallback=fallback,commanded_peak_joint_speed_rad_s=float(np.max(np.abs(v))))
    return v


def inside(point,lo,hi):
    point=np.asarray(point,float)
    return bool(point.shape==(3,) and np.isfinite(point).all() and
                np.all(point>=np.asarray(lo)-1e-9) and np.all(point<=np.asarray(hi)+1e-9))


class PathAudit:
    def __init__(self,task):
        self.task=task;self.previous=None;self.front_crossed=False;self.passed=False
        self.failures=[]

    def update(self,p,now,phase):
        p=np.asarray(p,float);t=self.task;reasons=[]
        if phase=='turn':
            if not inside(p,t['turn_min_m'],t['turn_max_m']):reasons.append('outside_turn_region')
        elif phase=='transport':
            lo=t['obstacle_min_m'][1];hi=t['obstacle_max_m'][1]
            points=[]
            if lo<=p[1]<=hi:points.append(p)
            if self.previous is not None:
                dy=p[1]-self.previous[1]
                if abs(dy)>1e-12:
                    for boundary in (lo,hi):
                        f=(boundary-self.previous[1])/dy
                        if 0<=f<=1:
                            q=self.previous+f*(p-self.previous);points.append(q)
                            legal=inside(q,t['gate_min_m'],t['gate_max_m'])
                            if dy>0 and boundary==lo and legal:self.front_crossed=True
                            if dy>0 and boundary==hi and legal and self.front_crossed:self.passed=True
            if any(not inside(q,t['gate_min_m'],t['gate_max_m']) for q in points):
                reasons.append('wrong_side_or_height_at_column')
            if p[1]>hi and not self.passed:reasons.append('gate_not_traversed')
        for reason in reasons:self.failures.append(dict(time_s=now,reason=reason,position_m=p.tolist()))
        self.previous=p.copy()
        return reasons


class RodTask:
    def __init__(self,config):
        self.config=config;self.phase='transport';self.started_s=None;self.paused=True
        self.events=[];self.rows=[];self.actual_path=PathAudit(config);self.raw_path=PathAudit(config)
        self.event_file=None
        self.stop_requested_s=None;self.entered_s=None;self.confirmed_s=None;self.user_confirmed_s=None
        self.currently_aligned=False;self.last_raw=None;self.auxiliary=False

    def event(self,kind,now,accepted=True,**detail):
        row=dict(event=kind,time_s=float(now),accepted=accepted,**detail)
        self.events.append(row)
        if self.event_file is not None:
            with self.event_file.open('a') as log:log.write(json.dumps(row)+'\n')

    def engage(self,now,position,position_fresh,axis_fresh):
        t=self.config
        ready=position_fresh and (self.phase=='transport' or axis_fresh)
        if self.started_s is None:
            ready=ready and inside(position,np.array(t['start_m'])-t['start_half_extent_m'],
                                    np.array(t['start_m'])+t['start_half_extent_m'])
        self.event('start' if self.started_s is None else 're_zero_resume',now,ready)
        if ready:
            if self.started_s is None:self.started_s=now
            else:self.auxiliary=True
            self.paused=False
        return ready

    def pause(self,now):
        self.paused=True;self.auxiliary=True;self.event('pause',now)

    def request_turn(self,now,position,position_fresh,axis_fresh):
        t=self.config
        ready=(self.started_s is not None and not self.paused and self.phase=='transport' and
               position_fresh and axis_fresh and self.actual_path.passed and self.raw_path.passed and
               inside(position,t['turn_min_m'],t['turn_max_m']) and self.last_raw is not None and
               inside(self.last_raw,t['turn_min_m'],t['turn_max_m']))
        self.event('turn',now,ready)
        if ready:self.phase='turn'
        return ready

    def request_finish(self,now):
        ready=self.phase=='turn' and not self.paused and self.stop_requested_s is None
        self.event('stop_request',now,ready)
        if ready:self.stop_requested_s=now
        return ready

    def confirm(self,now):
        ready=(self.currently_aligned and self.entered_s is not None and
               now-self.entered_s>=self.config['criteria']['hold_s'] and not self.paused)
        self.event('user_confirm',now,ready)
        if ready:self.user_confirmed_s=now
        return ready

    def observe(self,now,actual,rotation,raw,raw_axis,position_fresh,axis_fresh,safety,**extra):
        if self.started_s is None:return None
        t=self.config;c=t['criteria'];a=rotation[:,2];actual=np.asarray(actual)
        required_fresh=bool(position_fresh and (self.phase=='transport' or axis_fresh))
        actual_reasons=self.actual_path.update(actual,now,self.phase)
        raw_reasons=[]
        if raw is not None:
            self.last_raw=np.asarray(raw).copy()
            raw_reasons=self.raw_path.update(raw,now,self.phase)
        else:
            # Do not certify traversal by drawing a straight line across missing observations.
            self.raw_path.previous=None
        tip=actual+t['tool']['front_m']*a
        back=actual+t['tool']['back_m']*a
        goal=np.array(t['goal_tip_m'])
        pe=float(np.linalg.norm(tip-goal));ae=axis_angle(a,t['goal_axis'])
        radial=max(np.linalg.norm(point[1:]-goal[1:]) for point in (tip,back))+t['tool']['radius_m']
        extent_ok=all(t['corridor_x_m'][0]<=point[0]-t['tool']['radius_m'] and
                      point[0]+t['tool']['radius_m']<=t['corridor_x_m'][1] for point in (tip,back))
        corridor=bool(radial<=t['corridor_radius_m'] and extent_ok)
        self.currently_aligned=bool(self.phase=='turn' and not self.paused and required_fresh and
            pe<=c['goal_tip_m'] and ae<=c['axis_deg'] and corridor and not actual_reasons and
            safety.get('violation') is None)
        if self.currently_aligned and self.stop_requested_s is not None:
            if self.entered_s is None:self.entered_s=now
            if now-self.entered_s>=c['hold_s'] and self.confirmed_s is None:self.confirmed_s=now
        else:self.entered_s=None
        raw_axis=np.array(t['transport_axis']) if self.phase=='transport' else raw_axis
        row=dict(time_s=now,elapsed_s=now-self.started_s,phase=self.phase,paused=self.paused,
            required_fresh=required_fresh,position_fresh=bool(position_fresh),axis_fresh=bool(axis_fresh),
            actual_m=actual.tolist(),actual_axis=a.tolist(),tip_m=tip.tolist(),
            raw_m=None if raw is None else np.asarray(raw).tolist(),
            raw_axis=None if raw_axis is None else np.asarray(raw_axis).tolist(),
            position_error_m=None if raw is None else float(np.linalg.norm(actual-raw)),
            axis_error_deg=None if raw_axis is None else axis_angle(a,raw_axis),
            tip_error_m=None if raw is None or raw_axis is None else float(np.linalg.norm(tip-(raw+t['tool']['front_m']*np.asarray(raw_axis)))),
            goal_tip_error_m=pe,goal_axis_error_deg=ae,corridor=corridor,
            actual_path_failures=actual_reasons,raw_path_failures=raw_reasons,safety=safety,**extra)
        self.rows.append(row)
        return row

    @property
    def status(self):
        if self.user_confirmed_s is not None:return '用户已确认结束'
        if self.confirmed_s is not None and self.currently_aligned:return '保持已达标，Enter确认；过程结果另行评估'
        if self.stop_requested_s is not None:return '完成请求已记录；继续观察追赶与对准'
        if self.phase=='turn':return '转向：杆轴对准+X；到位按F'
        return '运输：沿立柱+X侧过门，到蓝区按T转向'
