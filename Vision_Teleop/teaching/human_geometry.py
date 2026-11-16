"""Independent, provenance-carrying upper-limb geometry; no live control import.

Coordinates are metres, rotations map canonical hand axes into ``frame``.
Validity means admissible to this calculation, never anatomical accuracy.
Missing/occluded observations must arrive invalid; this module cannot discover
occlusion from a confident 3D coordinate. It never interpolates or holds joints.
"""
from dataclasses import dataclass, replace
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class Observation:
    value: object
    stamp_s: float
    frame: str
    source: str
    kind: str  # triangulated_estimate, depth_surface_proxy, model_orientation, synthetic_test
    valid: bool = True
    reason: str = 'observed'
    sample_id: str = ''
    clock: str = 'host_perf_counter'
    unit: str = 'm'
    physical_reference: str = 'unspecified_not_anatomical_center'


@dataclass(frozen=True)
class ArmLength:
    metres: float
    source: str
    reference_id: str
    kind: str  # independently_measured, estimated_proxy, synthetic_test


@dataclass(frozen=True)
class Limits:
    max_age_s: float = .150
    max_joint_span_s: float = .0333
    min_plane_sine: float = .15
    min_length_m: float = 1e-6

    def __post_init__(self):
        a = np.array([self.max_age_s, self.max_joint_span_s, self.min_plane_sine, self.min_length_m])
        if not np.isfinite(a).all() or (a <= 0).any() or self.min_plane_sine >= 1:
            raise ValueError('Invalid geometry diagnostic limits')


def rotation(value):
    r = np.asarray(value, dtype=float)
    if (r.shape != (3, 3) or not np.isfinite(r).all()
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-5, rtol=0)
            or not np.isclose(np.linalg.det(r), 1., atol=1e-5, rtol=0)):
        raise ValueError('Expected proper rotation, no reflection or repair')
    return r


def _field(value=None, reason='missing', **extra):
    return dict(valid=value is not None, value=value, reason=reason, **extra)


def _read(obs, shape, frame, clock, at_s, limits):
    if obs is None:
        return None, 'missing'
    if not obs.valid:
        return None, obs.reason or 'invalid_observation'
    if not obs.source or not obs.kind or not obs.sample_id:
        return None, 'missing_provenance'
    if obs.kind not in {'triangulated_estimate', 'depth_surface_proxy', 'model_orientation', 'synthetic_test'}:
        return None, 'unsupported_observation_kind'
    if (shape == (3,) and obs.kind == 'model_orientation') or (shape == (3, 3) and obs.kind not in {'model_orientation', 'synthetic_test'}):
        return None, 'observation_kind_mismatch'
    if obs.unit != ('rotation_matrix' if shape == (3, 3) else 'm'):
        return None, 'unit_mismatch'
    if obs.frame != frame or obs.clock != clock:
        return None, 'frame_or_clock_mismatch'
    if not np.isfinite(obs.stamp_s):
        return None, 'invalid_timestamp'
    age = at_s - obs.stamp_s
    if age < -1e-9 or age > limits.max_age_s:
        return None, 'future_or_stale'
    try:
        v = np.asarray(obs.value, dtype=float)
        if v.shape != shape or not np.isfinite(v).all():
            return None, 'invalid_shape_or_nonfinite'
        if shape == (3, 3):
            rotation(v)
    except (ValueError, TypeError):
        return None, 'invalid_value_or_rotation'
    return v, 'observed_not_ground_truth'


def _length(length):
    return (isinstance(length, ArmLength) and np.isfinite(length.metres) and length.metres > 0
            and bool(length.source) and bool(length.reference_id)
            and length.kind in {'independently_measured', 'estimated_proxy', 'synthetic_test'})


def represent(points: dict[str, Observation], hand: Optional[Observation], *, frame: str,
              at_s: float, arm_length: Optional[ArmLength] = None,
              clock='host_perf_counter', limits=Limits()):
    """Each output has independent validity; source timestamps are not resampled.

    Right arm uses shoulder/elbow/wrist. Torso additionally needs left_shoulder,
    left_hip/right_hip. Torso axes x=left→right, y=hip→shoulder (orthogonalized),
    z=x×y; this is a landmark frame, not a measured anatomical forward axis.
    A fixed, documented arm-length reference is required for normalized reach.
    """
    if not frame or not clock or not np.isfinite(at_s):
        raise ValueError('Explicit frame, clock and finite evaluation time required')
    names = ['shoulder', 'elbow', 'wrist', 'left_shoulder', 'left_hip', 'right_hip']
    values, reasons, origins = {}, {}, {}
    for name in names:
        obs = points.get(name)
        values[name], reasons[name] = _read(obs, (3,), frame, clock, at_s, limits)
        origins[name] = dict(valid=values[name] is not None, reason=reasons[name],
                             source=obs.source if obs else None, kind=obs.kind if obs else None,
                             stamp_s=obs.stamp_s if obs and np.isfinite(obs.stamp_s) else None,
                             frame=obs.frame if obs else None, sample_id=obs.sample_id if obs else None)

    def group(required):
        bad = [f'{n}:{reasons[n]}' for n in required if values[n] is None]
        if bad:
            return ';'.join(bad)
        stamps = [points[n].stamp_s for n in required]
        if max(stamps) - min(stamps) > limits.max_joint_span_s:
            return 'joint_capture_time_span'
        return None

    output = dict(schema_version=1, frame=frame, clock=clock, evaluation_stamp_s=at_s,
                  units=dict(position='m', direction='unit_vector', extension='dimensionless'),
                  anatomical_accuracy_verified=False, exposure_sync_verified=False,
                  inputs=origins, shoulder_wrist_direction=_field(), extension=_field(),
                  arm_plane_normal=_field(), torso_reference=_field())
    reason = group(['shoulder', 'wrist'])
    if reason:
        output['shoulder_wrist_direction'] = _field(reason=reason)
        output['extension'] = _field(reason=reason)
    else:
        reach = values['wrist'] - values['shoulder']
        distance = float(np.linalg.norm(reach))
        if distance <= limits.min_length_m:
            output['shoulder_wrist_direction'] = _field(reason='zero_shoulder_wrist_distance')
        else:
            output['shoulder_wrist_direction'] = _field((reach / distance).tolist(), 'computed', reach_m=distance)
        if _length(arm_length):
            output['extension'] = _field(distance / arm_length.metres, 'computed_not_comfort',
                arm_length_m=arm_length.metres, reference_id=arm_length.reference_id,
                reference_kind=arm_length.kind, reference_source=arm_length.source,
                exceeds_reference=bool(distance > arm_length.metres))
        else:
            output['extension'] = _field(reason='missing_or_invalid_fixed_arm_length')
    reason = group(['shoulder', 'elbow', 'wrist'])
    if reason:
        output['arm_plane_normal'] = _field(reason=reason)
    else:
        upper, lower = values['elbow'] - values['shoulder'], values['wrist'] - values['elbow']
        lu, ll = float(np.linalg.norm(upper)), float(np.linalg.norm(lower))
        if min(lu, ll) <= limits.min_length_m:
            output['arm_plane_normal'] = _field(reason='zero_arm_segment')
        else:
            cross = np.cross(upper / lu, lower / ll)
            sine = float(np.linalg.norm(cross))
            output['arm_plane_normal'] = _field(
                (cross / sine).tolist() if sine >= limits.min_plane_sine else None,
                'computed' if sine >= limits.min_plane_sine else 'near_collinear_straight_or_folded',
                bend_sine=sine, min_bend_sine=limits.min_plane_sine,
                observed_segment_lengths_m=[lu, ll])
    reason = group(['shoulder', 'left_shoulder', 'left_hip', 'right_hip'])
    if reason:
        output['torso_reference'] = _field(reason=reason)
    else:
        origin = (values['shoulder'] + values['left_shoulder']) / 2
        x = values['shoulder'] - values['left_shoulder']
        up = origin - (values['left_hip'] + values['right_hip']) / 2
        nx, nu = np.linalg.norm(x), np.linalg.norm(up)
        if min(nx, nu) <= limits.min_length_m:
            output['torso_reference'] = _field(reason='zero_torso_axis')
        else:
            x = x / nx
            y = up / nu - x * np.dot(x, up / nu)
            ny = np.linalg.norm(y)
            if ny < limits.min_plane_sine:
                output['torso_reference'] = _field(reason='collinear_torso_axes')
            else:
                y /= ny
                output['torso_reference'] = _field(dict(origin_m=origin.tolist(),
                    rotation=np.column_stack((x, y, np.cross(x, y))).tolist()), 'landmark_frame_not_anatomical_truth')
    h, reason = _read(hand, (3, 3), frame, clock, at_s, limits)
    output['hand_orientation'] = _field(h.tolist() if h is not None else None, reason,
        stamp_s=hand.stamp_s if hand and np.isfinite(hand.stamp_s) else None,
        source=hand.source if hand else None, sample_id=hand.sample_id if hand else None,
        kind=hand.kind if hand else None, frame=hand.frame if hand else None)
    return output


def mapping_evidence(primary_command, points, *, body_frame, at_s,
                     arm_length=None, limits=Limits()):
    """Attach current geometry without making it depend on a clutch reference.

    This is an offline evidence interface, not a new position/orientation map.
    The original complete command stays authoritative. Instantaneous reach and
    plane, a personally confirmed range, and relative body motion have different
    prerequisites; a failed legacy body anchor does not erase current geometry.
    Body coordinates are never silently converted to the task/world frame.
    """
    from copy import deepcopy
    geometry = represent(points, None, frame=body_frame, at_s=at_s,
                         arm_length=arm_length, limits=limits)
    direction = geometry['shoulder_wrist_direction']
    plane = geometry['arm_plane_normal']
    return dict(primary_command=deepcopy(primary_command), current_body_geometry=geometry,
        usage=dict(
            reach_description=dict(available=direction['valid'], frame=body_frame,
                purpose='Describe observed shoulder-relative reach; no shoulder subtraction from control'),
            normalized_extension=dict(available=geometry['extension']['valid'],
                reason=geometry['extension']['reason'], comfort_measurement=False),
            range_initialization=dict(ready=False, reason='personal_range_not_established_by_current_observation',
                required=['documented_fixed_arm_length_and_endpoint_relation',
                          'explicit_fixed_operating_reference',
                          'personally_confirmed_direction_and_span'],
                downstream_interface='human_activity.fixed_mapping_suggestion', auto_change_scale=False),
            relative_body_motion=dict(current_plane_available=plane['valid'],
                reference_created=False, reason='separate_explicit_body_reference_required',
                automatic_late_binding=False, modifies_primary_requirement=False),
            world_body_direction=dict(ready=False, reason='body_to_task_registration_not_provided')),
        observed_geometry_requires_clutch=False, live_deployed=False,
        independent_orientation_policy='Keep caller original full-orientation command; never derive hand roll from arm plane')


def transform_observation(obs, R_target_source, translation_m, target_frame):
    """Explicit rigid frame change; times and observation validity stay unchanged."""
    r = rotation(R_target_source)
    t = np.asarray(translation_m, float)
    if t.shape != (3,) or not np.isfinite(t).all() or not target_frame:
        raise ValueError('Invalid rigid transform')
    if not obs.valid:
        return replace(obs, frame=target_frame, value=None)
    v = np.asarray(obs.value, float)
    if obs.unit == 'rotation_matrix':
        value = r @ rotation(v)
    elif obs.unit == 'm' and v.shape == (3,) and np.isfinite(v).all():
        value = r @ v + t
    else:
        raise ValueError('Invalid observation for transform')
    return replace(obs, value=value.tolist(), frame=target_frame,
                   source=obs.source + f'; rigid_transform:{obs.frame}->{target_frame}')


def confirmed_range_scale(arm_length, comfortable_span_fraction, task_span_m, *, confirmation_id):
    """Scalar proposal from an explicitly comfortable span, never a motion maximum.

    This is an offline initialization interface, not proof of comfort/reachability.
    Caller must obtain the confirmation; no body hard limit is used as comfort.
    """
    if (not _length(arm_length) or not confirmation_id
            or not np.isfinite([comfortable_span_fraction, task_span_m]).all()
            or not 0 < comfortable_span_fraction <= 1 or task_span_m <= 0):
        raise ValueError('Confirmed comfortable span and documented arm length required')
    human_span = arm_length.metres * comfortable_span_fraction
    return dict(scale=task_span_m / human_span, human_span_m=human_span,
                task_span_m=task_span_m, confirmation_id=confirmation_id,
                arm_length_reference_id=arm_length.reference_id,
                deployable=False, reason='proposal_requires_task_workspace_and_user_validation')


def input_semantics(points, hand, *, control_position=None, arm_length=None):
    """Describe physical references independently of coordinate/time validity.

    A coordinate transform never establishes that two surface observations refer
    to the same physical point. Three landmarks plus a hand rotation do not
    uniquely identify forearm axial rotation or all anatomical wrist angles.
    """
    entries = {}
    for name, obs in {**points, 'independent_hand_orientation': hand,
                      'control_wrist': control_position}.items():
        entries[name] = (dict(present=True, valid=obs.valid, reason=obs.reason,
            physical_reference=obs.physical_reference, source=obs.source, kind=obs.kind,
            frame=obs.frame, stamp_s=obs.stamp_s, clock=obs.clock, unit=obs.unit,
            sample_id=obs.sample_id, anatomical_center_verified=False) if obs else
            dict(present=False, valid=False, reason='missing_observation'))
    return dict(observations=entries,
        fixed_arm_length=(dict(valid=True, metres=arm_length.metres, source=arm_length.source,
            reference_id=arm_length.reference_id, kind=arm_length.kind) if _length(arm_length)
            else dict(valid=False, reason='no_independent_fixed_length')),
        observable_quantities='Directions and planes of admitted proxy points; independent model hand rotation',
        unavailable_anatomical_quantities=['verified_joint_centres','forearm_axial_rotation',
            'complete_wrist_joint_angles','personal_comfort'],
        fixed_lengths_inferred_from_instantaneous_segments=False,
        coordinate_transform_implies_physical_point_equivalence=False,
        opposite_shoulder_and_hips_required_for_local_right_arm=False)


def mapping_request(points, hand, anchor_points, anchor_hand, *, frame, at_s, anchor_s,
                    R_world_input, command_position_anchor, command_rotation_anchor,
                    scale, hand_position, anchor_hand_position, arm_length=None, limits=Limits()):
    """Offline requirement interface BEFORE TaskCommandState, not a controller.

    Range assistance supplies a fixed initialization scale. Position still uses
    the baseline hand-position observation, NOT the body's detected wrist. Delta
    wrist = delta shoulder + delta shoulder-to-wrist, retaining torso motion.
    Once initialized, missing body geometry disables only the optional preference
    and decomposition, not hand input. This is explicit interface policy, not a
    silent fallback or automatic rebind. Clutch caller binds both hand anchors
    to HELD COMMON command, never robot actual pose.
    """
    r = rotation(R_world_input)
    ra = rotation(command_rotation_anchor)
    pa = np.asarray(command_position_anchor, float)
    if pa.shape != (3,) or not np.isfinite(pa).all() or not np.isfinite(scale) or scale <= 0:
        raise ValueError('Invalid command anchor or scale')
    current = represent(points, hand, frame=frame, at_s=at_s, arm_length=arm_length, limits=limits)
    anchor = represent(anchor_points, anchor_hand, frame=frame, at_s=anchor_s, arm_length=arm_length, limits=limits)
    out = dict(frame='task_world', stamp_s=at_s, position=_field(), orientation=_field(),
               human_plane_preference=_field(), control_connected=False,
               anchor_policy='explicit_clutch_to_common_command_not_robot_actual')
    w, wr = _read(hand_position, (3,), frame, 'host_perf_counter', at_s, limits)
    w0, w0r = _read(anchor_hand_position, (3,), frame, 'host_perf_counter', anchor_s, limits)
    if w is not None and w0 is not None:
        out['position'] = _field((pa + scale * r @ (w - w0)).tolist(), 'same_baseline_hand_position_source',
                                sample_id=hand_position.sample_id, source=hand_position.source,
                                source_stamp_s=hand_position.stamp_s,
                                anchor_source_stamp_s=anchor_hand_position.stamp_s,
                                age_ms=1000 * (at_s - hand_position.stamp_s))
        if current['inputs']['shoulder']['valid'] and anchor['inputs']['shoulder']['valid']:
            s, s0 = np.asarray(points['shoulder'].value), np.asarray(anchor_points['shoulder'].value)
            if (abs(points['shoulder'].stamp_s - hand_position.stamp_s) <= limits.max_joint_span_s
                    and abs(anchor_points['shoulder'].stamp_s - anchor_hand_position.stamp_s) <= limits.max_joint_span_s):
                out['position'].update(retained_shoulder_translation_m=(s - s0).tolist(),
                                       reach_change_m=((w - s) - (w0 - s0)).tolist())
    else:
        out['position'] = _field(reason='hand_position_or_anchor_invalid:' + wr + ';' + w0r)
    if current['hand_orientation']['valid'] and anchor['hand_orientation']['valid']:
        relative = np.asarray(hand.value) @ np.asarray(anchor_hand.value).T
        out['orientation'] = _field((r @ relative @ r.T @ ra).tolist(), 'independent_hand_relative_rotation',
                                    source_stamp_s=hand.stamp_s, anchor_source_stamp_s=anchor_hand.stamp_s,
                                    source=hand.source, sample_id=hand.sample_id,
                                    age_ms=1000 * (at_s - hand.stamp_s))
    else:
        out['orientation'] = _field(reason='current_or_anchor_hand_missing')
    plane = current['arm_plane_normal']
    if plane['valid']:
        out['human_plane_preference'] = _field((r @ np.asarray(plane['value'])).tolist(),
            'request_not_ik_constraint', permitted_to_modify_primary_task=False)
    else:
        out['human_plane_preference'] = _field(reason=plane['reason'])
    return out


def secondary_capacity(task_jacobian, model_dof, *, rank_rtol=1e-9):
    """Inspect local 6D differential nullspace; NOT a constrained IK solution.

    No automatic robot fitting or integration. Even nonzero nullity can be
    blocked by joint/collision/torque constraints, and finite motion needs task
    rechecks. Robot-specific shoulder/elbow landmarks must be defined separately.
    """
    j = np.asarray(task_jacobian, float)
    if (not isinstance(model_dof, int) or model_dof < 6 or j.shape != (6, model_dof)
            or not np.isfinite(j).all() or not np.isfinite(rank_rtol) or not 0 < rank_rtol < 1):
        raise ValueError('Full model DOF and matching six-dimensional task Jacobian required')
    _, s, vh = np.linalg.svd(j, full_matrices=True)
    rank = int(np.sum(s > rank_rtol * s[0])) if s[0] else 0
    basis = vh[rank:].T
    return dict(rank=rank, model_dof=model_dof, local_nullity=model_dof-rank,
                nullspace_basis=basis.tolist(),
                primary_full_rank=rank == 6, apply_preference=False,
                reason='no_local_redundancy' if model_dof == rank else 'requires_constrained_task_preserving_solver')
