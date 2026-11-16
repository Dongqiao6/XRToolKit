"""Isolated adapters for the frozen bare-arm comparison, not a live entry."""
import json
from pathlib import Path

import mujoco
import numpy as np

from .rod_scene import ExplicitPairs, RodSafety, RodSafetyError


class PublishedTargets:
    """Consume only the most recent already-published record, aging both channels."""
    def __init__(self, rows):
        self.rows=rows;self.index=-1;self.time=-float('inf')
        times=[r['elapsed_s'] for r in rows]
        if not times or not np.isfinite(times).all() or np.any(np.diff(times)<=0):
            raise ValueError('Strictly increasing finite publication times required')

    def at(self,t):
        if not np.isfinite(t) or t<self.time:raise ValueError('Reader time must be monotonic')
        self.time=t
        while self.index+1<len(self.rows) and self.rows[self.index+1]['elapsed_s']<=t:
            self.index+=1
        if self.index<0:return dict(source_row=None,active=False,position_fresh=False,pose_fresh=False,
            raw_p=None,raw_r=None,position_age_ms=None,orientation_age_ms=None)
        r=self.rows[self.index];age=(t-r['elapsed_s'])*1000
        pa=None if r['position_age_ms'] is None else r['position_age_ms']+age
        ra=None if r['orientation_age_ms'] is None else r['orientation_age_ms']+age
        active=bool(r['active'] and r['reference_available'])
        pv=bool(active and r['position_valid'] and r['world_position_m'] is not None and pa is not None and pa<=150)
        rv=bool(active and r['orientation_source_valid'] and r['orientation_fresh'] and
            r['world_rotation'] is not None and ra is not None and ra<=150)
        return dict(source_row=r['source_row'],active=active,position_fresh=pv,pose_fresh=rv,
            raw_p=r['world_position_m'],raw_r=r['world_rotation'],position_age_ms=pa,orientation_age_ms=ra,
            orientation_source_stamp_s=r['orientation_source_stamp_s'],source_time_s=r['source_time_s'])


class BareSafety(RodSafety):
    """Reuse distance inspection/unit correction with native pairs and floor only.

    The standard SimArm optional task_safety hook is used without modifying its
    defaults, robot geometry, rod code or live behavior.
    """
    def __init__(self,arm):
        self.arm=arm;self.model=arm.model;model=self.model
        if arm.with_gripper or arm.site!='attachment_site':raise ValueError('Bare native attachment site required')
        self.task={'criteria':{'clearance_m':.005,'detection_distance_m':.03}}
        physical=[i for i in range(model.ngeom) if model.geom_contype[i] or model.geom_conaffinity[i]]
        self.names={i:model.geom(i).name or f'geom_{i}' for i in physical}
        floor=model.geom('floor').id
        native=set(arm.limits[-1].geom_id_pairs)
        mandatory={tuple(sorted((floor,g))) for g in physical if g!=floor}
        self.pairs=sorted(native|mandatory);self.pair_array=np.array(self.pairs,dtype=int)
        self.radii=model.geom_rbound[self.pair_array]
        self.limit=ExplicitPairs(model,[([a],[b]) for a,b in self.pairs],
            minimum_distance_from_collisions=.005,collision_detection_distance=.03)
        self.scratch=mujoco.MjData(model);self.trial=mujoco.MjData(model)
        self.audit=dict(native_pairs=len(native),pairs=len(self.pairs),mandatory_floor_pairs=len(mandatory),
            pair_names=[[self.names[a],self.names[b]] for a,b in self.pairs],
            geom_names={i:dict(geom=self.names[i],body=model.body(int(model.geom_bodyid[i])).name) for i in physical},
            exclusions='Native Mink rigid/adjacent self-pair policy preserved; explicit all physical geoms vs floor.',
            distance_cap_m=.03,check='2ms integrated q refreshed with mj_kinematics plus q midpoints; transaction commit only if all pass. Not a continuous proof or physical braking.')
        self.begin_step()

    def _check(self,data):
        info=self.inspect(data)
        speed=float(np.max(np.abs(data.qvel)))
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RodSafetyError('nonfinite_actual')
        self.last['samples']+=1
        if info['minimum_distance_m']<self.last['minimum_distance_m']:
            self.last.update(minimum_distance_m=info['minimum_distance_m'],nearest_pair=info['nearest_pair'])
        prior=self.last['joint_margin_rad']
        self.last['joint_margin_rad']=min(info['joint_margin_rad'],prior if prior is not None else float('inf'))
        self.last['max_actual_joint_speed_rad_s']=max(self.last['max_actual_joint_speed_rad_s'],speed)
        reason=('clearance' if info['minimum_distance_m']<.005-1e-9 else
                'joint_range' if info['joint_margin_rad']<-1e-9 else
                'actual_joint_speed' if speed>.6+1e-7 else None)
        if reason:self.last['violation']=reason;raise RodSafetyError(reason)

    def commit_checked_step(self,command_q,dt):
        self.begin_step();self.last['committed']=False
        self.scratch.qpos[:]=command_q;mujoco.mj_kinematics(self.model,self.scratch)
        command=self.inspect(self.scratch)
        ctrl=command_q[self.arm.q_indices]
        margin=float(np.min(np.minimum(ctrl-self.model.actuator_ctrlrange[:,0],self.model.actuator_ctrlrange[:,1]-ctrl)))
        speed=float(np.max(np.abs(ctrl-self.arm.data.ctrl)/dt))
        self.last.update(command=command,command_q=command_q.tolist(),command_speed_rad_s=speed,control_margin=margin)
        if command['minimum_distance_m']<.005-1e-9 or command['joint_margin_rad']<-1e-9 or margin<-1e-9 or speed>.6+1e-7:
            self.last.update(violation='command_constraint',retained_actual=self.inspect(self.arm.data))
            raise RodSafetyError('command_constraint')
        mujoco.mj_copyData(self.trial,self.model,self.arm.data);self.trial.ctrl[:]=ctrl
        try:
            for _ in range(round(dt/self.model.opt.timestep)):
                old=self.trial.qpos.copy();mujoco.mj_step(self.model,self.trial)
                # Refresh geometry without mutating dynamics solver/warm-start state.
                mujoco.mj_kinematics(self.model,self.trial)
                self.scratch.qpos[:]=(old+self.trial.qpos)/2
                self.scratch.qvel[:]=self.trial.qvel
                mujoco.mj_kinematics(self.model,self.scratch)
                self._check(self.scratch);self._check(self.trial)
        except RodSafetyError:
            rejected=dict(self.last)
            self.last=dict(**self.inspect(self.arm.data),command=command,command_q=command_q.tolist(),
                command_speed_rad_s=speed,control_margin=margin,committed=False,
                violation='predicted_'+str(rejected.get('violation')),rejected_step=rejected,
                rejected_q=self.trial.qpos.tolist(),rejected_qvel=self.trial.qvel.tolist(),rejected_time_s=float(self.trial.time))
            raise
        mujoco.mj_copyData(self.arm.data,self.model,self.trial)
        self.last['committed']=True


def attach_bare_safety(arm):
    guard=BareSafety(arm);arm.task_safety=guard;arm.limits[-1]=guard.limit
    return guard
