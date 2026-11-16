"""Explicit offline admission of a saved control wrist into body geometry.

A control-valid depth point need not be a visible wrist. This adapter requires
an independent wrist-region review and exact same-frame/session provenance.
It neither substitutes for RTMW automatically nor changes primary hand control.
"""
import numpy as np
from .human_geometry import Observation


def reviewed_control_wrist(record, *, session_id, body_session_id, body_frame_id,
                           wrist_visible, review_reason):
    meta = record.get('metadata', {})
    ns = meta.get('host_arrival_monotonic_ns')
    stamp = ns / 1e9 if isinstance(ns, int) and not isinstance(ns, bool) else float('nan')
    frame_id = record.get('frame_id', '')
    common = dict(stamp_s=stamp, frame='left_camera_optical',
                  source='saved_WiLoR_projected_wrist_left_depth_surface',
                  kind='depth_surface_proxy', sample_id=frame_id,
                  physical_reference='reviewed_wrist_region_surface_not_anatomical_center')
    diagnostic = dict(primary_control_modified=False, automatic_fallback=False,
                      anatomical_joint_center_verified=False, review_reason=review_reason,
                      inference_done_s=None)

    def reject(reason):
        return Observation(None, valid=False, reason=reason, **common), {**diagnostic, 'reason': reason}

    if not session_id or session_id != body_session_id:
        return reject('session_mismatch')
    if not body_frame_id or frame_id != body_frame_id or meta.get('frame_id') != frame_id:
        return reject('frame_mismatch')
    if meta.get('camera_role') != 'left' or meta.get('host_clock') != 'perf_counter_ns' or not np.isfinite(stamp):
        return reject('camera_or_clock_mismatch')
    hand = meta.get('hand', {})
    position = hand.get('rgbd_position', {})
    if hand.get('selection', {}).get('reason') != 'selected':
        return reject('hand_selection_invalid')
    if (hand.get('points_source') != 'wilor_projected_keypoints'
            or hand.get('position_definition') != 'wilor_projected_wrist_left_measured_surface'
            or position.get('source') != 'primary_rgbd_wrist_surface_proxy'):
        return reject('unsupported_position_definition')
    if position.get('valid') is not True:
        return reject('control_position_invalid:' + str(position.get('reason')))
    if (position.get('frame_id') != frame_id or position.get('camera_role') != 'left'
            or position.get('host_arrival_monotonic_ns') != ns):
        return reject('depth_provenance_mismatch')
    try:
        points = np.asarray(hand.get('points'), float)
        uv = np.asarray(position.get('pixel_xy'), float)
        xyz = np.asarray(position.get('position_m'), float)
        if (points.shape != (1,21,2) or uv.shape != (2,) or xyz.shape != (3,)
                or not np.isfinite(points).all() or not np.isfinite(xyz).all()
                or not np.array_equal(points[0,0], uv)):
            return reject('invalid_saved_point_or_pixel_correspondence')
    except (TypeError, ValueError):
        return reject('invalid_saved_point_or_pixel_correspondence')
    done = hand.get('inference_done_monotonic_ns')
    if not isinstance(done, int) or isinstance(done, bool) or done < ns:
        return reject('unknown_or_invalid_inference_completion')
    diagnostic['inference_done_s'] = done / 1e9
    diagnostic['completion_is_control_availability'] = False
    if wrist_visible is not True or not review_reason:
        return reject('wrist_correspondence_unverified:' + str(review_reason))
    return Observation(xyz.tolist(), **common), {**diagnostic, 'reason': 'reviewed_surface_proxy_only'}
