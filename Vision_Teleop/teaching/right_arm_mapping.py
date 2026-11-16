"""Offline right-arm auxiliary input; never imported by the live demo.

The auxiliary wrist is ALWAYS the right WiLoR wrist plus that camera's depth.
The primary position remains the original LEFT RGB-D wrist. They are distinct
surface proxies, not interchangeable anatomical centres. No per-frame fallback.
"""
from dataclasses import dataclass, replace

import numpy as np

from teaching.human_geometry import Limits, input_semantics, mapping_request, represent, rotation

WRIST_SOURCE = 'WiLoR_projected_wrist_plus_right_aligned_depth_surface'
BODY_SOURCE = 'RTMW_2d_plus_right_aligned_depth_surface'
RIGHT = 'right_camera_optical'
LEFT = 'left_camera_optical'


@dataclass(frozen=True)
class BodyHandRegistration:
    """OpenCV convention: p_right = R_right_left @ p_left + t_right_left_m.

    accepted_for_sdk_body_hand must be backed by a separate review for these
    camera profiles. A successful board fit alone does not establish this.
    """
    R_right_left: object
    t_right_left_m: object
    calibration_sha256: str
    review_reference: str
    accepted_for_sdk_body_hand: bool = False

    def inverse(self):
        r = rotation(self.R_right_left)
        t = np.asarray(self.t_right_left_m, float)
        if (t.shape != (3,) or not np.isfinite(t).all()
                or len(self.calibration_sha256) != 64
                or any(c not in '0123456789abcdef' for c in self.calibration_sha256)
                or not self.review_reference):
            raise ValueError('Explicit calibration hash, review and metre translation required')
        return r.T, -r.T @ t


def right_arm_request(points, hand, *, hand_position, anchor_hand,
                      anchor_hand_position, at_s, anchor_s, R_world_left,
                      command_position_anchor, command_rotation_anchor, scale,
                      registration=None, limits=Limits()):
    """Return unchanged primary requirements plus a separately gated preference.

    Only shoulder/elbow/right wrist enter the auxiliary geometry. No shoulder
    subtraction, inferred length, torso fabrication, interpolation or IK runs.
    Caller binds both hand anchors explicitly to the held COMMON command.
    """
    request = mapping_request({}, hand, {}, anchor_hand, frame=LEFT, at_s=at_s,
        anchor_s=anchor_s, R_world_input=R_world_left,
        command_position_anchor=command_position_anchor,
        command_rotation_anchor=command_rotation_anchor, scale=scale,
        hand_position=hand_position, anchor_hand_position=anchor_hand_position,
        limits=limits)
    checked = {}
    shoulder = points.get('shoulder')
    for name in ('shoulder', 'elbow', 'wrist'):
        obs = points.get(name)
        if obs is None:
            continue
        expected = WRIST_SOURCE if name == 'wrist' else BODY_SOURCE
        reason = None
        if obs.source != expected or obs.kind != 'depth_surface_proxy':
            reason = 'fixed_auxiliary_source_mismatch'
        elif shoulder is not None and obs.sample_id != shoulder.sample_id:
            reason = 'right_body_frame_id_mismatch'
        if reason:
            obs = replace(obs, value=None, valid=False, reason=reason)
        checked[name] = obs
    local = represent(checked, None, frame=RIGHT, at_s=at_s, limits=limits)
    plane = local['arm_plane_normal']
    reason = plane['reason'] if not plane['valid'] else None
    if reason is None and not (request['position']['valid'] and request['orientation']['valid']):
        reason = 'primary_hand_requirement_invalid'
    if reason is None:
        stamps = [checked[n].stamp_s for n in ('shoulder', 'elbow', 'wrist')]
        stamps += [hand.stamp_s, hand_position.stamp_s]
        if max(stamps) - min(stamps) > limits.max_joint_span_s:
            reason = 'body_hand_capture_time_span'
    r_left_right = None
    registration_state = 'missing_registration'
    if registration is not None:
        try:
            r_left_right, _ = registration.inverse()
            registration_state = ('reviewed_for_sdk_body_hand' if
                registration.accepted_for_sdk_body_hand is True else 'registration_review_required')
        except (ValueError, TypeError):
            registration_state = 'invalid_registration'
    if reason is None and registration_state != 'reviewed_for_sdk_body_hand':
        reason = registration_state
    preference = dict(valid=False, value=None, reason=reason,
        permitted_to_modify_primary_task=False, control_connected=False,
        source_frame=RIGHT, frame='task_world')
    if reason is None:
        preference.update(valid=True, reason='request_only_not_ik_constraint',
            value=(rotation(R_world_left) @ r_left_right @ np.asarray(plane['value'])).tolist(),
            source_stamp_s=checked['wrist'].stamp_s,
            sample_id=checked['wrist'].sample_id,
            calibration_sha256=registration.calibration_sha256,
            review_reference=registration.review_reference)
    request['human_plane_preference'] = preference
    return dict(primary_and_preference=request, right_arm_geometry=local,
        input_semantics=input_semantics(checked, hand, control_position=hand_position),
        auxiliary_wrist_source=WRIST_SOURCE,
        primary_position_source=hand_position.source if hand_position else None,
        registration_status=registration_state, live_deployed=False,
        note='Sparse offline interface; no continuous tracking, comfort or accuracy claim')
