"""Isolated computation-equivalent prediction loop, never a live default."""
from pathlib import Path
import ctypes as C
import mujoco
import numpy as np
from .basic_comparison import BareSafety
from .pose_candidate import BoundaryGovernor
from .rod_scene import RodSafetyError

ROOT=Path(__file__).resolve().parents[1]
DLL=ROOT/'results/pose_diagnostic/20260917_native_prediction_01/native_prediction.dll'
class Inspection(C.Structure):
    _fields_=[('distance',C.c_double),('margin',C.c_double),('a',C.c_int),('b',C.c_int)]
class Result(C.Structure):
    _fields_=[('command',Inspection),('aggregate',Inspection),('command_speed',C.c_double),
        ('control_margin',C.c_double),('max_speed',C.c_double),('samples',C.c_int),('reason',C.c_int)]

class NativeSafety(BareSafety):
    def __init__(self,arm):
        super().__init__(arm)
        self.library=C.CDLL(str(DLL))
        if mujoco.__version__!='3.12.0' or self.library.prediction_version()!=3012000:
            raise RuntimeError('Pinned MuJoCo ABI mismatch')
        self.native=self.library.prediction_interval
        self.native.argtypes=[C.c_void_p]*5+[C.c_int]+[C.c_void_p]*2+[C.c_double,C.c_int,C.c_void_p,C.POINTER(Result)]
        self.native.restype=None
        self.native_pairs=np.ascontiguousarray(self.pair_array,dtype=np.int32)
        self.native_indices=np.ascontiguousarray(arm.q_indices,dtype=np.int32)
        self.old=np.empty(self.model.nq);self.output=Result()

    def info(self,v):
        return dict(minimum_distance_m=v.distance,nearest_pair=None if v.a<0 else [self.names[v.a],self.names[v.b]],joint_margin_rad=v.margin)

    def commit_checked_step(self,command_q,dt):
        # Preserve Python behavior on invalid inputs; native fast path is finite only.
        if not np.isfinite(command_q).all() or not np.isfinite(self.arm.data.qpos).all() or not np.isfinite(self.arm.data.qvel).all():
            return super().commit_checked_step(command_q,dt)
        command_q=np.ascontiguousarray(command_q,dtype=np.float64)
        self.begin_step();self.last['committed']=False
        self.native(self.model._address,self.arm.data._address,self.scratch._address,self.trial._address,
            self.native_pairs.ctypes.data,len(self.pairs),self.native_indices.ctypes.data,command_q.ctypes.data,
            dt,round(dt/self.model.opt.timestep),self.old.ctypes.data,C.byref(self.output))
        o=self.output;command=self.info(o.command)
        self.last.update(command=command,command_q=command_q.tolist(),command_speed_rad_s=o.command_speed,control_margin=o.control_margin)
        if o.reason==1:
            self.last.update(violation='command_constraint',retained_actual=self.inspect(self.arm.data))
            raise RodSafetyError('command_constraint')
        self.last.update(self.info(o.aggregate))
        if not o.samples:self.last['joint_margin_rad']=None
        self.last.update(samples=o.samples,max_actual_joint_speed_rad_s=o.max_speed)
        if o.reason:
            reason={2:'clearance',3:'joint_range',4:'actual_joint_speed',5:'nonfinite_actual'}[o.reason]
            if o.reason!=5:self.last['violation']=reason
            rejected=dict(self.last)
            self.last=dict(**self.inspect(self.arm.data),command=command,command_q=command_q.tolist(),
                command_speed_rad_s=o.command_speed,control_margin=o.control_margin,committed=False,
                violation='predicted_'+str(rejected.get('violation')),rejected_step=rejected,
                rejected_q=self.trial.qpos.tolist(),rejected_qvel=self.trial.qvel.tolist(),rejected_time_s=float(self.trial.time))
            raise RodSafetyError(reason)
        mujoco.mj_copyData(self.arm.data,self.model,self.trial);self.last['committed']=True

class NativeBoundaryGovernor(BoundaryGovernor):
    def __init__(self,arm):
        super().__init__(arm)
        self.probe=NativeSafety(self.probe.arm)
        self.candidate=mujoco.MjData(self.model)

    def evaluate(self,q,dt,fraction):
        mujoco.mj_copyData(self.probe.arm.data,self.model,self.arm.data);first=None
        try:
            self.probe.commit_checked_step(q,dt);first=dict(self.probe.last)
            mujoco.mj_copyData(self.candidate,self.model,self.probe.arm.data)
            self.probe.commit_checked_step(q,dt);second=dict(self.probe.last)
            mujoco.mj_copyData(self.accepted,self.model,self.candidate);self.best_last=first;ok=True
        except RodSafetyError:second=dict(self.probe.last);ok=False
        self.execution['attempts'].append(dict(fraction=fraction,accepted=ok,issued_interval=first,held_next_interval=second))
        return ok

def attach_native_governor(arm):
    guard=NativeBoundaryGovernor(arm);arm.task_safety=guard;arm.limits[-1]=guard.limit
    return guard
