"""Offline circular relative-swivel objective, below an existing primary step.

Uses the existing named model anchors and original constraints. The reference
depends on q: its derivative is included. No arbitrary 6-DOF redundancy, global
IK solution, comfort claim, or live import. Circular residual has a stationary
antipodal target; this local task is not an exact angle-tracking guarantee.
"""
import mink
import mujoco
import numpy as np
from .configuration_preference import plane_geometry
from .relative_swivel import transport_normal, _pair


def swivel_geometry(configuration, joint_names, anchor_axis, anchor_normal):
    g = plane_geometry(configuration, joint_names)
    p = g['points']; d = p[2]-p[0]; length = np.linalg.norm(d)
    if length < 1e-6:
        raise ValueError('zero_shoulder_wrist_distance')
    u = d/length; u0, n0 = _pair(anchor_axis, anchor_normal)
    ref = transport_normal(u0, n0, u)
    jac = []
    for i in [0, 2]:
        model, data = configuration.model, configuration.data
        jid = model.joint(joint_names[i]).id
        j = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, j, None, p[i], int(model.jnt_bodyid[jid]))
        jac.append(j)
    ju = (np.eye(3)-np.outer(u, u)) @ (jac[1]-jac[0])/length
    v = np.cross(u0, u); dv = np.cross(u0, ju.T).T
    c = float(u0@u); dc = u0@ju
    b = np.cross(v, n0); db = np.cross(dv.T, n0).T
    z = np.cross(v, b)
    dz = np.cross(dv.T, b).T + np.cross(v, db.T).T
    jr = db + dz/(1+c) - np.outer(z, dc)/(1+c)**2
    n, jn = g['normal'], g['jacobian']
    cosine, sine = float(ref@n), float(u@np.cross(ref, n))
    jcos = n@jr + ref@jn
    jsin = np.cross(ref, n)@ju + u@(np.cross(jr.T, n).T + np.cross(ref, jn.T).T)
    jangle = (cosine*jsin-sine*jcos)/(cosine*cosine+sine*sine)
    return dict(axis=u, normal=n, reference_normal=ref,
        angle_rad=float(np.arctan2(sine, cosine)), circular=np.array([cosine, sine]),
        jacobian=np.vstack([jcos, jsin]), angle_jacobian=jangle)


class RobotSwivelTask(mink.Task):
    k = 2

    def __init__(self, joint_names, anchor_axis, anchor_normal, target_angle_rad,
                 *, binding_id, cost=.001):
        u, n = _pair(anchor_axis, anchor_normal)
        if (not binding_id or not np.isfinite(target_angle_rad)
                or abs(target_angle_rad) > np.pi or not np.isfinite(cost) or cost <= 0):
            raise ValueError('Explicit reference binding, principal angle and positive cost required')
        if len(joint_names) != 3 or len(set(joint_names)) != 3:
            raise ValueError('Three distinct robot joint anchors required')
        super().__init__(cost=np.full(2, cost), gain=1., lm_damping=0.)
        self.joint_names = tuple(joint_names)
        self.anchor_axis, self.anchor_normal = u.copy(), n.copy()
        self.binding_id = binding_id
        self.target = np.array([np.cos(target_angle_rad), np.sin(target_angle_rad)])

    def geometry(self, configuration):
        return swivel_geometry(configuration, self.joint_names, self.anchor_axis, self.anchor_normal)

    def compute_error(self, configuration):
        return self.geometry(configuration)['circular']-self.target

    def compute_jacobian(self, configuration):
        return self.geometry(configuration)['jacobian']
