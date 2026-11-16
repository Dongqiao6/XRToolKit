"""A tool-axis task: roll about the axis is explicitly unconstrained.

This only describes the task; weighted IK is not a hard angular guarantee.
The vector residual is singular at the antipode: callers preposition with
the existing full-pose task before enabling this local task.
"""
import mink
import mujoco
import numpy as np


class ToolAxis(mink.Task):
    k = 3

    def __init__(self, site, direction):
        super().__init__(cost=np.ones(3), gain=1., lm_damping=1e-3)
        self.site = site
        direction = np.asarray(direction, float)
        if direction.shape != (3,) or not np.isfinite(direction).all() or not np.isclose(np.linalg.norm(direction), 1.):
            raise ValueError('Unit finite tool direction required')
        self.direction = direction.copy()

    def compute_error(self, configuration):
        return configuration.data.site(self.site).xmat.reshape(3,3)[:,2] - self.direction

    def compute_jacobian(self, configuration):
        axis = configuration.data.site(self.site).xmat.reshape(3,3)[:,2]
        x,y,z = axis
        skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        jac = np.zeros((3, configuration.model.nv))
        mujoco.mj_jacSite(configuration.model, configuration.data, None, jac, configuration.model.site(self.site).id)
        return -skew @ jac
