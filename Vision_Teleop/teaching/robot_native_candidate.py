"""Isolated synchronous process boundary for unchanged FR3 computation.

One request in flight, no queue of future commands and no independent stepping.
The parent MjData is a display mirror only; worker owns physical integration.
"""
from pathlib import Path
import multiprocessing as mp
import time,traceback
import numpy as np
import mujoco

def _worker(conn,folder,robot='franka_fr3',mount=0,condition=None,gravity_compensation=False,task_profile=None):
    from teaching.robot import SimArm
    from scripts.pose_diagnostic import prepare
    from scripts.run_basic_comparison_v2 import execution_row
    from scripts.compare_angular_rate import capture_failure
    arm=None;blocked=False
    try:
        if task_profile is None:
            arm=SimArm(robot,mount_degrees=mount,gravity_compensation=gravity_compensation)
        else:
            from teaching.task_profiles import make_arm,profile
            p=profile(task_profile)
            if (robot,mount,condition,gravity_compensation)!=(p['robot'],p['mount_y_degrees'],'A',True):
                raise ValueError('Worker/profile configuration mismatch')
            arm=make_arm(task_profile)
        if condition is None:
            initial=prepare(arm,Path(folder))
            from teaching.native_prediction import attach_native_governor
            attach_native_governor(arm)
        else:
            from teaching.task_demo import prepare_task
            initial=prepare_task(arm,Path(folder),condition)
        conn.send(dict(kind='ready',actual=initial))
        while True:
            request=conn.recv()
            if request['kind']=='close':break
            if request['kind']=='state':
                spec=mujoco.mjtState.mjSTATE_INTEGRATION;state=np.empty(mujoco.mj_stateSize(arm.model,spec));mujoco.mj_getState(arm.model,arm.data,state,spec)
                conn.send(dict(spec=int(spec),state=state.tolist(),reference_q=arm.configuration.q.tolist()));continue
            if request['kind']!='step' or blocked:raise RuntimeError('Unknown request or blocked worker')
            started=time.perf_counter()
            try:
                arm.set_target(request['p'],request['r']);arm.step(request['dt']);actual=execution_row(arm)
                forces={k:getattr(arm.data,k).tolist() for k in ['qfrc_actuator','qfrc_bias','qfrc_gravcomp','actuator_force']} if task_profile else None
                conn.send(dict(kind='step',index=request['index'],actual=actual,forces=forces,worker_started_s=started,worker_finished_s=time.perf_counter()))
            except Exception as exc:
                blocked=True
                conn.send(dict(kind='blocked',index=request['index'],error=str(exc),detail=capture_failure(arm,exc,request['dt']),worker_started_s=started,worker_finished_s=time.perf_counter()))
    except EOFError:pass
    except Exception:
        if task_profile and arm is not None:
            import json
            spec=mujoco.mjtState.mjSTATE_INTEGRATION
            state=np.empty(mujoco.mj_stateSize(arm.model,spec));mujoco.mj_getState(arm.model,arm.data,state,spec)
            (Path(folder)/'worker_failure_state.json').write_text(json.dumps(dict(spec=int(spec),state=state.tolist(),reference_q=arm.configuration.q.tolist())),encoding='utf8')
        conn.send(dict(kind='error',traceback=traceback.format_exc()))
    finally:conn.close()

class RobotProcess:
    def __init__(self,folder,robot='franka_fr3',mount=0,condition=None,gravity_compensation=False,task_profile=None):
        parent,child=mp.get_context('spawn').Pipe();self.conn=parent
        self.process=mp.get_context('spawn').Process(target=_worker,args=(child,str(folder),robot,mount,condition,gravity_compensation,task_profile),name='isolated-'+robot)
        self.process.start();child.close();self.index=0;self.last_failure=None;self.timings=[]
        try:
            if not self.conn.poll(120):raise TimeoutError('Robot worker preparation timeout')
            row=self.conn.recv()
            if row['kind']!='ready':raise RuntimeError(str(row))
            self.model=mujoco.MjModel.from_binary_path(str(Path(folder)/'model.mjb'));self.data=mujoco.MjData(self.model)
            self._mirror(row['actual'])
        except Exception:self.close();raise

    def _mirror(self,actual):
        self.actual=actual
        # Read-only presentation copy; never the integration state used for checks.
        self.data.qpos[:]=actual['q'];self.data.qvel[:]=actual['qvel'];self.data.ctrl[:]=actual['issued_q'];self.data.time=actual['simulation_s'];mujoco.mj_forward(self.model,self.data)

    def set_target(self,p,r):self.target=(np.asarray(p).tolist(),np.asarray(r).tolist())

    def step(self,dt):
        sent=time.perf_counter();self.conn.send(dict(kind='step',index=self.index,p=self.target[0],r=self.target[1],dt=dt))
        if not self.conn.poll(5):raise TimeoutError('Robot worker step timeout; no claimed safe hold')
        received=self.conn.recv();returned=time.perf_counter()
        if received['kind']=='error':raise RuntimeError(received['traceback'])
        if received['index']!=self.index:raise RuntimeError('Robot response ordering mismatch')
        self.timings.append(dict(index=self.index,sent_s=sent,returned_s=returned,worker_started_s=received['worker_started_s'],worker_finished_s=received['worker_finished_s'],worker_ms=(received['worker_finished_s']-received['worker_started_s'])*1000,roundtrip_ms=(returned-sent)*1000))
        self.index+=1
        if received['kind']=='blocked':self.last_failure=received['detail'];raise RuntimeError(received['error'])
        self.forces=received.get('forces')
        self._mirror(received['actual'])

    def final_state(self):
        self.conn.send(dict(kind='state'))
        if not self.conn.poll(5):raise TimeoutError('Worker final state unavailable')
        return self.conn.recv()

    def close(self):
        if self.process.is_alive():
            try:self.conn.send(dict(kind='close'))
            except (OSError,EOFError):pass
            self.process.join(5)
        if self.process.is_alive():self.process.terminate();self.process.join(5)
        self.conn.close()

