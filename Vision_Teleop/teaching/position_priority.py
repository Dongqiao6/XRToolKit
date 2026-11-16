"""Two Mink QPs: preserve primary Cartesian translation in the secondary solve.

Specific position-before-orientation interface, not a general HQP framework.
The guarantee is local J dq equality, not finite-motion or global feasibility.
Concept: Escande, Mansard, Wieber, IJRR 2014, doi:10.1177/0278364914521306.
"""
import mink
import mujoco
import numpy as np


class CartesianPosition(mink.Task):
    k=3
    def __init__(self,site,position):
        super().__init__(cost=np.ones(3),gain=1.,lm_damping=1e-3)
        self.site=site;self.position=np.asarray(position,float)

    def compute_error(self,configuration):
        return configuration.data.site(self.site).xpos-self.position

    def compute_jacobian(self,configuration):
        jac=np.zeros((3,configuration.model.nv))
        mujoco.mj_jacSite(configuration.model,configuration.data,jac,None,configuration.model.site(self.site).id)
        return jac


class PreserveTranslation(mink.Task):
    k=3
    def __init__(self,jacobian,displacement):
        super().__init__(cost=np.ones(3),gain=1.)
        self.jacobian=jacobian.copy();self.displacement=displacement.copy()

    def compute_error(self,configuration):return -self.displacement
    def compute_jacobian(self,configuration):return self.jacobian


def solve_position_first(arm,dt):
    target=arm.eef.transform_target_to_world
    primary=CartesianPosition(arm.site,target.translation())
    # DAQP defaults permit a small displacement violation, amplified by 1/dt
    # when reported as velocity. Tighten numerical accuracy, never safety bounds.
    v1=mink.solve_ik(arm.configuration,[primary,arm.posture],dt,'daqp',limits=arm.limits,safety_break=True,primal_tol=1e-9)
    j=primary.compute_jacobian(arm.configuration)
    preserve=PreserveTranslation(j,j@v1*dt)
    # Keep original frame task costs intact for callers and ordinary holds.
    if getattr(arm,'target_axis_only',False):
        from .tool_axis import ToolAxis
        orientation=ToolAxis(arm.site,target.rotation().as_matrix()[:,2])
    else:
        orientation=mink.FrameTask(arm.site,'site',position_cost=0.,orientation_cost=1.,lm_damping=1e-3)
        orientation.set_target(target)
    fallback=False
    try:
        v2=mink.solve_ik(arm.configuration,[orientation,arm.posture],dt,'daqp',limits=arm.limits,constraints=[preserve],safety_break=True,primal_tol=1e-9)
    except mink.NoSolutionFound:
        v2=v1.copy();fallback=True
    residual=float(np.linalg.norm(j@(v2-v1)*dt))
    if residual>1e-7:raise RuntimeError('Secondary IK violated primary translation equality')
    violation=0.
    for limit in arm.limits:
        c=limit.compute_qp_inequalities(arm.configuration,dt)
        if not c.inactive:
            violation=max(violation,float(np.max(c.G@(v2*dt)-c.h)))
    if violation>1e-7:raise RuntimeError('IK violated an original safety inequality')
    arm.priority_diagnostics=dict(linear_translation_preservation_error_m=residual,secondary_fallback=fallback,
                                  commanded_peak_joint_speed_rad_s=float(np.max(np.abs(v2))),max_inequality_violation=violation)
    return v2


class PreservePose(mink.Task):
    k=6
    def __init__(self,jacobian,displacement):
        super().__init__(cost=np.ones(6),gain=1.)
        self.jacobian=jacobian.copy();self.displacement=displacement.copy()

    def compute_error(self,configuration):return -self.displacement
    def compute_jacobian(self,configuration):return self.jacobian


def solve_pose_first(arm,dt,secondary=None):
    """Full end-effector task before configuration preference, local QP equality.

    Position/orientation tradeoffs inside the primary task retain FrameTask units
    and costs; only its relation to posture is hierarchical, not global feasibility.
    """
    v1=mink.solve_ik(arm.configuration,[arm.eef],dt,'daqp',limits=arm.limits,safety_break=True,primal_tol=1e-9)
    j=arm.eef.compute_jacobian(arm.configuration)
    preserve=PreservePose(j,j@v1*dt)
    fallback=False
    try:
        v2=mink.solve_ik(arm.configuration,secondary or [arm.posture],dt,'daqp',limits=arm.limits,
            constraints=[preserve],safety_break=True,primal_tol=1e-9)
    except mink.NoSolutionFound:
        v2=v1.copy();fallback=True
    residual=float(np.linalg.norm(j@(v2-v1)*dt))
    if residual>1e-7:raise RuntimeError('Secondary IK violated primary pose equality')
    violation=0.
    for limit in arm.limits:
        c=limit.compute_qp_inequalities(arm.configuration,dt)
        if not c.inactive:violation=max(violation,float(np.max(c.G@(v2*dt)-c.h)))
    if violation>1e-7:raise RuntimeError('IK violated original safety inequalities')
    arm.priority_diagnostics=dict(linear_pose_preservation_error=residual,secondary_fallback=fallback,
        commanded_peak_joint_speed_rad_s=float(np.max(np.abs(v2))),max_inequality_violation=violation)
    return v2
