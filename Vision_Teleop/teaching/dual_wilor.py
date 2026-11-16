"""Two current WiLoR orientations in one camera frame; no accumulated bridging.

WiLoR's confirmed right-hand MANO rotations use the same canonical hand frame.
OpenCV calibration maps X_right = R_lr X_left + T_lr, hence Q_left = R_lr.T Q_right.
Equal chordal averaging uses SciPy; compatibility does not establish accuracy.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def proper_rotation(value):
    if value is None:
        return None
    r = np.asarray(value, float)
    if (r.shape != (3, 3) or not np.isfinite(r).all()
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-5)
            or abs(np.linalg.det(r) - 1) > 1e-5):
        return None
    return r


def fuse_orientation(left, right, right_age_ms, left_to_right, settings):
    """A right sample may be current/past only; the caller preserves timestamps.

    Missing/invalid/conflicting observations return explicit validity. No held
    rotation is labeled as a measurement. A stale right view cannot veto left.
    """
    extrinsic = proper_rotation(left_to_right)
    if extrinsic is None:
        raise ValueError('Invalid calibrated rotation')
    bound = float(settings['max_disagreement_deg'])
    age_bound = float(settings['max_pair_delta_ms'])
    weights = np.asarray(settings['rotation_weights'], float)
    if not (0 < bound < 180 and 0 < age_bound and np.isfinite(age_bound)
            and weights.shape == (2,) and np.isfinite(weights).all() and (weights > 0).all()):
        raise ValueError('Invalid frozen fusion settings')
    l = proper_rotation(left.get('rotation')) if left and left.get('valid') else None
    fresh = right_age_ms is not None and np.isfinite(right_age_ms) and 0 <= right_age_ms <= age_bound
    r = proper_rotation(right.get('rotation')) if fresh and right and right.get('valid') else None
    if r is not None:
        r = extrinsic.T @ r
    out = dict(valid=False, rotation=None, source='none', disagreement_deg=None,
               right_fresh=bool(fresh), anatomical_accuracy_verified=False)
    if l is None and r is None:
        return {**out, 'reason': 'no_current_valid_orientation'}
    if l is not None and r is not None:
        angle = float(np.degrees(Rotation.from_matrix(l @ r.T).magnitude()))
        out['disagreement_deg'] = angle
        if angle > bound:
            return {**out, 'reason': 'cross_view_conflict'}
        value = Rotation.from_matrix(np.stack((l, r))).mean(weights=weights).as_matrix()
        source = 'dual_rgb'
    else:
        value = l if l is not None else r
        source = 'left_rgb' if l is not None else 'right_rgb'
    return {**out, 'valid': True, 'rotation': value.tolist(), 'source': source,
            'reason': 'current_measurement_not_ground_truth'}
