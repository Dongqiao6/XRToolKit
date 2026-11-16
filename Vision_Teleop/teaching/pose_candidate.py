"""Isolated integrated-pose candidate; not imported by the default live/demo."""
import numpy as np
import mujoco
from .hand_tracking import select_right_hand
from .basic_execution import ReferenceGovernor
from .rod_scene import RodSafetyError

# Majority overlap of the smaller landmark envelope is an ambiguity test,
# not identity proof. Frozen before candidate tests; no score/size preference.
CONFLICT_OVERLAP = .5

def select_candidate_hand(observation):
    selected = select_right_hand(observation)
    labels = observation.get('handedness', [])
    points = np.asarray(observation.get('points', []))
    conflicts = []
    if points.shape == (len(labels),21,2) and np.isfinite(points).all():
        boxes = np.concatenate([points.min(axis=1),points.max(axis=1)],axis=1)
        for i in range(len(labels)):
            for j in range(i):
                if {labels[i].get('label'),labels[j].get('label')} != {'Left','Right'}:continue
                a,b=boxes[i],boxes[j];area=min(np.prod(a[2:]-a[:2]),np.prod(b[2:]-b[:2]))
                overlap=float(np.prod(np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2])))/area) if area>0 else 0.
                if overlap>=CONFLICT_OVERLAP:conflicts.append(dict(indices=[j,i],overlap_smaller=overlap))
    selected['selection']['baseline_reason']=selected['selection']['reason']
    selected['selection']['region_conflicts']=conflicts
    if conflicts:
        selected['selection'].update(reason='overlapping_handedness_conflict',index=None)
        for key in ('points','world_points_m','handedness'):selected[key]=[]
        selected.pop('rgbd_position',None)
    return selected

class BoundaryGovernor(ReferenceGovernor):
    """V2 plus one bounded inward reference when its old-reference fallback fails.

    Predictions do not halt model time on accepted steps. An unresolved failure
    is a blocked simulation condition, never reported as physical braking.
    """
    def commit_checked_step(self,command_q,dt):
        raw=command_q.copy();previous=raw.copy();previous[self.arm.q_indices]=self.arm.data.ctrl
        site=self.arm.configuration.data.site(self.arm.site)
        self.execution=dict(raw_ik_q=raw.tolist(),raw_ik_p=site.xpos.tolist(),raw_ik_r=site.xmat.reshape(3,3).tolist(),previous_reference_q=previous.tolist(),attempts=[],horizon_s=2*dt,bisections=self.bisections,retreat=False)
        origin=previous
        if self.evaluate(raw,dt,1.):
            issued=raw;fraction=1.
        else:
            if not self.evaluate(previous,dt,0.):
                failure=dict(self.probe.last)
                rejected=np.array(failure.get('rejected_q',[]))
                ranges=self.model.jnt_range
                if rejected.shape!=raw.shape:
                    self.last=failure;raise RodSafetyError('boundary_backup_missing_predicted_state')
                upper=rejected>ranges[:,1];lower=rejected<ranges[:,0]
                if not np.any(upper|lower):
                    self.last=failure;raise RodSafetyError('boundary_backup_not_joint_range')
                retreat=previous.copy()
                retreat[upper]-=.6*dt;retreat[lower]+=.6*dt
                # Reference bounds only; never clip qpos/qvel or reset dynamics.
                retreat=np.clip(retreat,ranges[:,0],ranges[:,1])
                self.execution.update(retreat=True,retreat_joints=np.flatnonzero(upper|lower).tolist(),retreat_reference=retreat.tolist(),previous_reference_failure=failure)
                if not self.evaluate(retreat,dt,-1.):
                    self.last=dict(self.probe.last);raise RodSafetyError('boundary_inward_reference_not_safe')
                origin=retreat
            lo,hi=0.,1.
            # Retain the v2 safe-segment search, now from the tested fallback.
            for _ in range(self.bisections):
                mid=(lo+hi)/2
                if self.evaluate(origin+mid*(raw-origin),dt,mid):lo=mid
                else:hi=mid
            fraction=lo;issued=origin+fraction*(raw-origin)
        self.execution.update(fraction=fraction,issued_q=issued.tolist(),max_joint_adjustment_rad=float(np.max(np.abs(issued-raw))),committed=True)
        mujoco.mj_copyData(self.arm.data,self.model,self.accepted)
        self.arm.configuration.update(issued);self.last=self.best_last

def attach_candidate_governor(arm):
    guard=BoundaryGovernor(arm);arm.task_safety=guard;arm.limits[-1]=guard.limit;return guard
