"""Isolated one-factor candidate: remove angular relaxation, keep rate bound.

Not imported by live, preparation or position demo. No target history/queue.
Validity and freeze decisions remain with the original PublishedTargets reader.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def latest_rotation_step(previous, latest, dt, maximum_rate=.5):
    if not np.isfinite(dt) or dt<=0 or not np.isfinite(maximum_rate) or maximum_rate<=0:
        raise ValueError('Positive finite dt and angular rate required')
    previous=np.asarray(previous,float)
    if latest is None:return previous.copy()
    latest=np.asarray(latest,float)
    if latest.shape!=(3,3) or not np.isfinite(latest).all() or not np.allclose(latest.T@latest,np.eye(3),atol=1e-6) or np.linalg.det(latest)<.999:
        raise ValueError('Invalid rotation target')
    delta=Rotation.from_matrix(latest@previous.T).as_rotvec()
    fraction=min(1.,maximum_rate*dt/max(float(np.linalg.norm(delta)),1e-12))
    return Rotation.from_rotvec(delta*fraction).as_matrix()@previous
