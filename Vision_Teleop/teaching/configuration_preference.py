"""Offline body-plane preference below an already solved full tool increment.

Reuses Mink equality constraints and the existing PreservePose task. This is not
a general HQP solver, a replacement for live IK, or a finite-motion guarantee.
The named robot joint anchors define a geometric descriptor, not human anatomy.
"""
import mink
import mujoco
import numpy as np
from .position_priority import PreservePose
from .human_geometry import secondary_capacity

ROBOT_POINTS = {
    'franka_fr3': ('fr3_joint2', 'fr3_joint4', 'fr3_joint6'),
    'ur5e': ('shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint'),
}


def plane_geometry(configuration, joint_names, min_sine=.15):
    """Ordered S/E/W joint anchors in world; refuse collinear geometry."""
    if len(joint_names) != 3 or len(set(joint_names)) != 3:
        raise ValueError('Three explicit distinct robot joint anchors required')
    model, data = configuration.model, configuration.data
    xyz, jac = [], []
    for name in joint_names:
        jid = model.joint(name).id
        point = data.xanchor[jid].copy()
        jp = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jp, None, point, int(model.jnt_bodyid[jid]))
        xyz.append(point); jac.append(jp)
    a, b = xyz[1]-xyz[0], xyz[2]-xyz[1]
    c = np.cross(a, b); norm = np.linalg.norm(c)
    denom = np.linalg.norm(a)*np.linalg.norm(b)
    if not np.isfinite(denom) or denom < 1e-12 or norm/denom < min_sine:
        raise ValueError('Robot arm plane degenerate; no inferred normal')
    n = c/norm
    da, db = jac[1]-jac[0], jac[2]-jac[1]
    dc = np.cross(da.T, b).T + np.cross(a, db.T).T
    jn = (np.eye(3)-np.outer(n, n)) @ dc/norm
    return dict(normal=n, jacobian=jn, points=np.asarray(xyz), bend_sine=float(norm/denom))


class RobotPlaneTask(mink.Task):
    """Signed normal residual n(q)-n_target; fixed explicit world reference.

    Normal alignment may be infeasible, even for 7 DOF. It is only a secondary
    soft request; no SEW-coordinate or anatomical-equivalence claim.
    """
    k = 3

    def __init__(self, joint_names, target_normal_world, *, cost=.001):
        n = np.asarray(target_normal_world, float)
        if n.shape != (3,) or not np.isfinite(n).all() or not np.isclose(np.linalg.norm(n), 1., atol=1e-8, rtol=0):
            raise ValueError('An explicit unit world normal required; no silent normalization')
        if not np.isfinite(cost) or cost <= 0:
            raise ValueError('Positive finite preference cost required')
        super().__init__(cost=np.full(3, cost), gain=1., lm_damping=0.)
        self.joint_names = tuple(joint_names)
        self.target = n.copy()

    def compute_error(self, configuration):
        return plane_geometry(configuration, self.joint_names)['normal'] - self.target

    def compute_jacobian(self, configuration):
        return plane_geometry(configuration, self.joint_names)['jacobian']


def preserve_tool_increment(arm, primary_velocity, dt, secondary_tasks):
    """Return velocity, diagnostics. Caller owns integration and execution checks.

    Freeze J_e dq = J_e dq_baseline at this same q, target and dt. Does not
    promote the baseline's damped, position-first step to a strict pose optimum.
    No nullity means exact unchanged output. Missing preference should be omitted
    explicitly by caller; invalid primary increments are never accepted.
    """
    v0 = np.asarray(primary_velocity, float)
    if (not np.isfinite(dt) or dt <= 0 or v0.shape != (arm.model.nv,)
            or not np.isfinite(v0).all()):
        raise ValueError('Finite primary velocity and positive dt required')
    arm.configuration.check_limits(safety_break=True)

    def violation(v):
        maximum = 0.
        for limit in arm.limits:
            c = limit.compute_qp_inequalities(arm.configuration, dt)
            if not c.inactive:
                maximum = max(maximum, float(np.max(c.G @ (v*dt)-c.h)))
        return maximum

    if violation(v0) > 1e-7:
        raise RuntimeError('Primary increment violates original inequalities')
    j = arm.eef.compute_jacobian(arm.configuration)
    capacity = secondary_capacity(j, arm.model.nv)
    reason = 'secondary_solved'
    if not secondary_tasks:
        v = v0.copy(); reason = 'no_secondary_request'
    elif capacity['local_nullity'] == 0:
        v = v0.copy(); reason = 'no_local_redundancy'
    else:
        preserve = PreservePose(j, j @ (v0*dt))
        try:
            v = mink.solve_ik(arm.configuration, secondary_tasks, dt, 'daqp',
                limits=arm.limits, constraints=[preserve], safety_break=True, primal_tol=1e-9)
        except mink.NoSolutionFound:
            v = v0.copy(); reason = 'secondary_qp_failed_primary_retained'
    delta = j @ ((v-v0)*dt)
    residual = float(np.linalg.norm(delta)); bad = violation(v)
    if not np.isfinite(v).all() or residual > 1e-7 or bad > 1e-7:
        raise RuntimeError('Secondary result violated preserved pose or original inequalities')
    return v, dict(reason=reason,linear_pose_residual=residual,
        linear_translation_residual_m=float(np.linalg.norm(delta[:3])),
        linear_rotation_residual_rad=float(np.linalg.norm(delta[3:])),
        max_inequality_violation=bad,rank=capacity['rank'],nullity=capacity['local_nullity'],
        delta_joint_speed_rad_s=float(np.max(np.abs(v-v0))),
        finite_motion_or_actual_state_guarantee=False,live_deployed=False)
