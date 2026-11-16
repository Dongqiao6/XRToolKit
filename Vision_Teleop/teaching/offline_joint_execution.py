"""Direct position-servo references with transactional original bare-arm checks.

No live IK, filtering, physical state reset or unsafe candidate submission.
"""
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
from .basic_comparison import BareSafety
from .rod_scene import RodSafetyError

class DirectJointExecution(BareSafety):
    def __init__(self,arm):
        super().__init__(arm);self.site=self.model.site('attachment_site').id;self.saved=mujoco.MjData(self.model)
        self.previous_reference_velocity=np.zeros(3);self.execution={}
    def cartesian_velocity(self,data):
        jp=np.zeros((3,self.model.nv));jr=np.zeros_like(jp);mujoco.mj_comPos(self.model,data);mujoco.mj_jacSite(self.model,data,jp,jr,self.site)
        return jp@data.qvel,jr@data.qvel
    def step(self,planned,dt=.01):
        if dt<=0 or abs(round(dt/self.model.opt.timestep)*self.model.opt.timestep-dt)>1e-8:raise ValueError('Positive simulation-multiple dt required')
        planned=np.asarray(planned,float);self.begin_step();self.last['committed']=False
        tracking=float(np.max(abs(self.arm.configuration.q-self.arm.data.qpos)))
        self.execution=dict(accepted=False,issued_q=None,previous_reference_q=self.arm.configuration.q.tolist(),tracking_error_rad=tracking)
        if tracking>.2:
            self.last.update(violation='previous_reference_tracking_error',tracking_error_rad=tracking)
            raise RodSafetyError('Simulation cannot track the bounded command; target updates stopped')
        if planned.shape!=(self.model.nq,) or not np.isfinite(planned).all():raise ValueError('Finite complete planned joint reference required')
        previous=self.arm.data.ctrl.copy();self.scratch.qpos[:]=previous;mujoco.mj_kinematics(self.model,self.scratch)
        oldp=self.scratch.site_xpos[self.site].copy();oldr=self.scratch.site_xmat[self.site].reshape(3,3).copy()
        self.scratch.qpos[:]=planned;mujoco.mj_kinematics(self.model,self.scratch);command=self.inspect(self.scratch)
        p=self.scratch.site_xpos[self.site].copy();r=self.scratch.site_xmat[self.site].reshape(3,3).copy();v=(p-oldp)/dt;speed=np.linalg.norm(v);omega=Rotation.from_matrix(r@oldr.T).magnitude()/dt;acc=np.linalg.norm(v-self.previous_reference_velocity)/dt
        qspeed=float(np.max(abs(planned-previous))/dt);ctrlmargin=float(np.min(np.minimum(planned-self.model.actuator_ctrlrange[:,0],self.model.actuator_ctrlrange[:,1]-planned)))
        self.execution=dict(planned_q=planned.tolist(),previous_reference_q=previous.tolist(),tracking_error_rad=tracking,issued_q=None,adjustment_rad=0.,accepted=False,reference_speed_m_s=float(speed),reference_acceleration_m_s2=float(acc),reference_angular_rad_s=float(omega),reference_joint_rad_s=qspeed,horizon_s=2*dt)
        violation=None
        if command['minimum_distance_m']<.005-1e-9 or command['joint_margin_rad']<-1e-9 or ctrlmargin<-1e-9:violation='planned_reference_geometry_or_control_range'
        elif qspeed>.6+1e-7 or speed>.1+1e-7 or omega>.5+1e-7 or acc>.2+1e-7:violation='planned_reference_rate'
        if violation:
            self.last.update(command=command,violation=violation);raise RodSafetyError(violation)
        mujoco.mj_copyData(self.trial,self.model,self.arm.data);self.trial.ctrl[:]=planned
        beforev,_=self.cartesian_velocity(self.trial);peaks=dict(speed_m_s=0.,angular_rad_s=0.,acceleration_m_s2=0.,torque_ratio=0.)
        h=self.model.opt.timestep;microsteps=round(dt/h);first=None
        try:
            for k in range(2*microsteps):
                previousq=self.trial.qpos.copy();mujoco.mj_step(self.model,self.trial);mujoco.mj_kinematics(self.model,self.trial)
                self.scratch.qpos[:]=(previousq+self.trial.qpos)/2;self.scratch.qvel[:]=self.trial.qvel;mujoco.mj_kinematics(self.model,self.scratch)
                self._check(self.scratch);self._check(self.trial)
                currentv,currentw=self.cartesian_velocity(self.trial);a=np.linalg.norm(currentv-beforev)/h;beforev=currentv
                limits=np.max(abs(self.model.jnt_actfrcrange),axis=1);ratio=float(np.max(abs(self.trial.qfrc_actuator)/limits))
                peaks['speed_m_s']=max(peaks['speed_m_s'],float(np.linalg.norm(currentv)));peaks['angular_rad_s']=max(peaks['angular_rad_s'],float(np.linalg.norm(currentw)));peaks['acceleration_m_s2']=max(peaks['acceleration_m_s2'],float(a));peaks['torque_ratio']=max(peaks['torque_ratio'],ratio)
                if not np.isfinite(self.trial.qfrc_actuator).all() or ratio>1+1e-7:raise RodSafetyError('actual_drive_force')
                # Cartesian bounds belong to the planned/issued reference above,
                # as in TaskCommandState/PositionMotion/SmoothTarget. Actual
                # Cartesian rates are diagnostics; original actual joint,
                # collision, range and actuator checks remain hard constraints.
                if k==microsteps-1:
                    mujoco.mj_copyData(self.saved,self.model,self.trial);first=dict(self.last);firstpeaks=peaks.copy()
        except Exception as exc:
            self.last.update(violation='predicted_'+str(exc),rejected_q=self.trial.qpos.tolist(),rejected_qvel=self.trial.qvel.tolist(),rejected_time=float(self.trial.time),cartesian_peaks=peaks,committed=False);raise
        mujoco.mj_copyData(self.arm.data,self.model,self.saved);self.arm.configuration.update(planned);self.previous_reference_velocity=v
        self.last=first|dict(committed=True,cartesian_peaks=firstpeaks,prediction_peaks=peaks);self.execution.update(issued_q=planned.tolist(),accepted=True)
        return self.last
