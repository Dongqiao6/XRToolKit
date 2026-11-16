"""Offline anchor-relative arm swivel; no control entry imports this module.

This local reference convention is related to general SEW parameterization, not
a reproduction of stereographic SEW or an anatomical joint-angle estimate.
Shortest rotation from a fixed shoulder-wrist axis transports the anchor plane.
Its antipode is singular. Angles are circular principal values, never unwrapped
through missing samples. A reference is explicit and never rebound on recovery.
"""
from dataclasses import dataclass
import numpy as np
from .human_geometry import represent, Limits

# Numerical conditioning guard for the reference construction, not a new
# perception acceptance limit. Both anchor and current geometry retain Limits.
MIN_ONE_PLUS_DOT = 1e-6


def _unit(value):
    v = np.asarray(value, float)
    if v.shape != (3,) or not np.isfinite(v).all() or not np.isclose(
            np.linalg.norm(v), 1., atol=1e-8, rtol=0):
        raise ValueError('Explicit finite unit vector required')
    return v


def _pair(axis, normal):
    u, n = _unit(axis), _unit(normal)
    if abs(float(u @ n)) > 1e-8:
        raise ValueError('Plane normal must be perpendicular to shoulder-wrist axis')
    return u, n


def transport_normal(anchor_axis, anchor_normal, current_axis):
    """Shortest-rotation reference; reject antipode instead of choosing an axis."""
    u0, n0 = _pair(anchor_axis, anchor_normal)
    u = _unit(current_axis)
    c = float(np.clip(u0 @ u, -1., 1.))
    if 1. + c <= MIN_ONE_PLUS_DOT:
        raise ValueError('antipodal_reference_axis')
    v = np.cross(u0, u)
    return n0 + np.cross(v, n0) + np.cross(v, np.cross(v, n0))/(1. + c)


@dataclass(frozen=True)
class SwivelAnchor:
    axis: tuple
    normal: tuple
    frame: str
    clock: str
    stamp_s: float
    reference_id: str
    # Immutable provenance tuples: name, source, kind, sample_id, stamp_s.
    sources: tuple

    def __post_init__(self):
        _pair(self.axis, self.normal)
        if (not self.frame or not self.clock or not self.reference_id
                or not np.isfinite(self.stamp_s) or len(self.sources) != 3):
            raise ValueError('Explicit valid anchor and provenance required')


def _geometry(points, *, frame, at_s, clock, limits):
    # No hand pose is synthesized. It is independently handled by the existing
    # primary mapping; neither this descriptor nor its validity changes it.
    return represent(points, None, frame=frame, at_s=at_s, clock=clock, limits=limits)


def bind_swivel(points, *, frame, at_s, reference_id,
                clock='host_perf_counter', limits=Limits()):
    g = _geometry(points, frame=frame, at_s=at_s, clock=clock, limits=limits)
    for name in ['shoulder_wrist_direction', 'arm_plane_normal']:
        if not g[name]['valid']:
            raise ValueError('Invalid anchor: ' + g[name]['reason'])
    sources = tuple((n, points[n].source, points[n].kind, points[n].sample_id,
                     points[n].stamp_s) for n in ['shoulder', 'elbow', 'wrist'])
    return SwivelAnchor(tuple(g['shoulder_wrist_direction']['value']),
        tuple(g['arm_plane_normal']['value']), frame, clock, at_s, reference_id, sources)


def relative_swivel(points, anchor, *, frame, at_s,
                    clock='host_perf_counter', limits=Limits()):
    """Optional request descriptor. Invalid input gives no angle, never a hold.

    Translation and scale cancel only in this auxiliary descriptor. The primary
    position path must still include original hand and torso displacement.
    """
    g = _geometry(points, frame=frame, at_s=at_s, clock=clock, limits=limits)
    out = dict(valid=False, reason=None, angle_rad=None, circular_cos_sin=None,
        frame=frame, clock=clock, evaluation_stamp_s=at_s, inputs=g['inputs'],
        reference_id=anchor.reference_id if isinstance(anchor, SwivelAnchor) else None,
        meaning='relative_elbow_side_preference_not_hand_roll_or_comfort',
        permitted_to_modify_primary_task=False, live_deployed=False)
    if not isinstance(anchor, SwivelAnchor):
        out['reason'] = 'missing_anchor'; return out
    if (frame, clock) != (anchor.frame, anchor.clock):
        out['reason'] = 'anchor_frame_or_clock_mismatch'; return out
    if at_s < anchor.stamp_s:
        out['reason'] = 'before_anchor'; return out
    for name in ['shoulder_wrist_direction', 'arm_plane_normal']:
        if not g[name]['valid']:
            out['reason'] = g[name]['reason']; return out
    u, n = _pair(g['shoulder_wrist_direction']['value'], g['arm_plane_normal']['value'])
    try:
        ref = transport_normal(anchor.axis, anchor.normal, u)
    except ValueError as e:
        out['reason'] = str(e); return out
    sine, cosine = float(u @ np.cross(ref, n)), float(ref @ n)
    angle = float(np.arctan2(sine, cosine))
    out.update(valid=True, reason='computed_from_observed_surface_geometry',
        angle_rad=angle, circular_cos_sin=[cosine, sine],
        principal_branch_near_cut=bool(abs(abs(angle)-np.pi) < 1e-6),
        axis=u.tolist(), normal=n.tolist(), reference_normal=ref.tolist(),
        anchor_axis_dot=float(np.asarray(anchor.axis) @ u),
        anatomical_accuracy_verified=False, unwrapped=False)
    return out


def robot_normal_request(anchor_axis, anchor_normal, current_axis, angle_rad, *, binding_id):
    """Geometric request at the given robot axis, NOT a ready-to-use IK task.

    binding_id identifies the explicitly chosen human/robot reference pair.
    Own robot anchors replace a shared world normal: task semantics differ.
    The target depends on q through the robot S-W axis. A solver must include
    that derivative, not feed this into a constant-target plane Jacobian and
    claim an exact swivel objective. No reachability or redundancy guarantee.
    """
    if not binding_id or not np.isfinite(angle_rad) or abs(angle_rad) > np.pi:
        raise ValueError('Explicit reference binding and principal angle required')
    u = _unit(current_axis)
    ref = transport_normal(anchor_axis, anchor_normal, u)
    normal = np.cos(angle_rad)*ref + np.sin(angle_rad)*np.cross(u, ref)
    return dict(normal=normal.tolist(), axis=u.tolist(), angle_rad=float(angle_rad),
        binding_id=binding_id, target_depends_on_robot_configuration=True,
        ready_for_constant_target_plane_task=False, reachability_verified=False,
        permitted_to_modify_primary_task=False)
