"""One bounded reference-increment governor for the isolated bare-arm study.

Never changes velocities or the physical state. Predictions use the pinned
simulator and are not a physical braking guarantee. No live entry imports this.
"""
from types import SimpleNamespace

import mujoco
import numpy as np

from .basic_comparison import BareSafety
from .rod_scene import RodSafetyError


class ReferenceGovernor(BareSafety):
    bisections = 12

    def __init__(self, arm):
        super().__init__(arm)
        proxy = SimpleNamespace(model=arm.model, data=mujoco.MjData(arm.model),
            with_gripper=False, site=arm.site, q_indices=arm.q_indices, limits=arm.limits)
        self.probe = BareSafety(proxy)
        self.accepted = mujoco.MjData(arm.model)
        self.execution = {}

    def evaluate(self, q, dt, fraction):
        """Check the issued interval AND one following interval holding its ref."""
        mujoco.mj_copyData(self.probe.arm.data, self.model, self.arm.data)
        first = None
        try:
            self.probe.commit_checked_step(q, dt)
            first = dict(self.probe.last)
            # Save first interval, never commit the look-ahead interval.
            candidate = mujoco.MjData(self.model)
            mujoco.mj_copyData(candidate, self.model, self.probe.arm.data)
            self.probe.commit_checked_step(q, dt)
            second = dict(self.probe.last)
            mujoco.mj_copyData(self.accepted, self.model, candidate)
            self.best_last = first
            ok = True
        except RodSafetyError:
            second = dict(self.probe.last)
            ok = False
        self.execution['attempts'].append(dict(fraction=fraction, accepted=ok,
            issued_interval=first, held_next_interval=second))
        return ok

    def commit_checked_step(self, command_q, dt):
        raw = command_q.copy()
        old = self.arm.data.ctrl.copy()
        previous = raw.copy(); previous[self.arm.q_indices] = old
        site = self.arm.configuration.data.site(self.arm.site)
        self.execution = dict(raw_ik_q=raw.tolist(), raw_ik_p=site.xpos.tolist(),
            raw_ik_r=site.xmat.reshape(3, 3).tolist(), previous_reference_q=previous.tolist(),
            attempts=[], horizon_s=2*dt, bisections=self.bisections)
        if self.evaluate(raw, dt, 1.):
            fraction = 1.
        else:
            if not self.evaluate(previous, dt, 0.):
                self.last = dict(self.probe.last)
                self.last['violation'] = 'governor_hold_reference_not_safe'
                self.last['committed'] = False
                raise RodSafetyError('governor_hold_reference_not_safe')
            lo, hi = 0., 1.
            # Fixed resolution, not repeated tuning; no monotonicity claim.
            # Only a tested-safe candidate may be issued.
            for _ in range(self.bisections):
                mid = (lo+hi)/2
                if self.evaluate(previous+mid*(raw-previous), dt, mid):lo = mid
                else:hi = mid
            fraction = lo
        issued = previous+fraction*(raw-previous)
        self.execution.update(fraction=fraction, issued_q=issued.tolist(),
            max_joint_adjustment_rad=float(np.max(np.abs(issued-raw))), committed=True)
        mujoco.mj_copyData(self.arm.data, self.model, self.accepted)
        # Future IK starts from the last actually issued reference, preventing
        # unissued joint increments accumulating. Actual integrator stays intact.
        self.arm.configuration.update(issued)
        self.last = self.best_last


def attach_reference_governor(arm):
    guard = ReferenceGovernor(arm)
    arm.task_safety = guard
    arm.limits[-1] = guard.limit
    return guard
