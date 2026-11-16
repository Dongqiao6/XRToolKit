"""Task-only time/validity contract and bounded FR3 gravity compensation.

The original task stays the default. No estimator, mapping or filter tuning.
"""
import numpy as np
import mujoco
from .position_motion import PositionMotion
from .replay import SmoothTarget

CONTROL_DT = .01


class TaskCommandState:
    """Each channel uses its own freshness; explicit engagement still needs both.

    One call advances exactly one physical control interval. Source age is wall
    time, never the number of calls. Missing rotation holds the command; missing
    position requests bounded deceleration. Neither fabricates observations.
    """
    def __init__(self, position, rotation, lower, upper, settings):
        self.motion = PositionMotion(position, settings)
        self.target = SmoothTarget(position, rotation, settings)
        self.lower = np.asarray(lower, float)
        self.upper = np.asarray(upper, float)
        self.engaged = False
        # Acceleration is an execution bound, not a tunable side effect of tau.
        self.motion.acceleration = .2

    def update(self, sample):
        active = bool(sample['active'])
        def fresh(channel, age):
            value = sample.get(age)
            return bool(active and sample[channel] and value is not None
                        and np.isfinite(value) and 0 <= value <= 150)
        position_ok = fresh('position_fresh', 'position_age_ms')
        rotation_ok = fresh('pose_fresh', 'orientation_age_ms')
        if position_ok and rotation_ok:
            self.engaged = True
        desired = None
        if self.engaged and position_ok:
            point = np.asarray(sample['raw_p'], float)
            if point.shape != (3,) or not np.isfinite(point).all():
                raise ValueError('Fresh position must contain a finite point')
            desired = np.clip(point, self.lower, self.upper)
        if self.engaged and rotation_ok:
            self.target.update(None, np.asarray(sample['raw_r'], float), CONTROL_DT)
        point = self.motion.update(desired, CONTROL_DT)
        if np.any(point < self.lower-1e-7) or np.any(point > self.upper+1e-7):
            raise RuntimeError('Common command outside original workspace')
        return dict(command_p=point.tolist(), command_r=self.target.rotation.tolist(),
                    engaged=self.engaged)


def next_control_tick(previous, started, finished, period=CONTROL_DT):
    """No compressed starts and no second full idle period after an overrun.

    This is deliberately not phase catch-up. OS lateness/long computation can
    still lose model time, which the existing wall-minus-model log exposes.
    """
    if not np.isfinite([previous, started, finished, period]).all() or period <= 0:
        raise ValueError('Finite clock values and positive period required')
    if started < previous or finished < started:
        raise ValueError('Control clock must be monotonic')
    earliest = started + period
    return max(earliest, finished), finished >= earliest


def compensated_fr3_model(scene):
    """Compile the existing FR3 with native gravity compensation under its caps.

    MuJoCo 3.12.0 body.gravcomp + joint.actgravcomp supplies compensation through
    the joint actuator clamp. Do not use qfrc_applied or zero model gravity.
    Compilation sets internal gravity-compensation flags consistently.
    """
    if mujoco.__version__ != '3.12.0':
        raise RuntimeError('Gravity candidate requires pinned MuJoCo 3.12.0')
    spec = mujoco.MjSpec.from_file(str(scene))
    expected = {f'fr3_joint{i}' for i in range(1, 8)}
    if {j.name for j in spec.joints} != expected:
        raise ValueError('Compensation is validated only for the bare seven-joint FR3')
    for body in spec.bodies:
        if body.name in {f'fr3_link{i}' for i in range(1, 8)}:
            body.gravcomp = 1.
    for joint in spec.joints:
        joint.actgravcomp = True
    model = spec.compile()
    if not np.all(model.jnt_actfrclimited) or not np.all(model.jnt_actgravcomp):
        raise ValueError('Compensation must remain inside original joint force caps')
    return model
