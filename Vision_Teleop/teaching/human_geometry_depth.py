"""Offline body-landmark depth surface adapter, not anatomical joint estimation.

Reuses the existing SDK deprojection; explicitly requires visibility review.
No camera access, learned filling, or mixing the board K with SDK intrinsics.
"""
import numpy as np

from teaching.human_geometry import Observation
from teaching.rgbd_position import deproject


def surface_point(joint, metadata, depth, *, visible, visibility_reason, protocol,
                  project=deproject, depth_role='left'):
    if depth_role not in ('left', 'right'):
        raise ValueError('Explicit left or right depth source required')
    stamp = metadata['host_arrival_monotonic_ns'] / 1e9
    common = dict(stamp_s=stamp, frame=f'{depth_role}_camera_optical',
                  source=f'RTMW_2d_plus_{depth_role}_aligned_depth_surface', kind='depth_surface_proxy',
                  sample_id=metadata['frame_id'])
    diagnostic = dict(anatomical_joint_center_verified=False, visible_review=visible,
                      visibility_reason=visibility_reason, pixel=joint.get('pixel'))

    def reject(reason):
        return Observation(None, valid=False, reason=reason, **common), {**diagnostic, 'reason': reason}

    if visible is not True:
        return reject('visibility_rejected:' + visibility_reason)
    if not joint.get('valid_2d'):
        return reject('invalid_2d_detection')
    pixel = np.asarray(joint.get('pixel'), float)
    if pixel.shape != (2,) or not np.isfinite(pixel).all():
        return reject('invalid_pixel')
    profile = metadata.get('aligned_depth')
    if profile is None or depth is None:
        return reject('missing_aligned_depth')
    roles = [metadata[k] for k in ('role', 'camera_role') if k in metadata]
    if not roles or any(role != depth_role for role in roles):
        return reject('depth_camera_role_mismatch')
    if depth.dtype != np.uint16 or depth.shape != (metadata['height'], metadata['width']):
        return reject('depth_shape_or_type_mismatch')
    if profile['timestamp_domain'] != metadata['timestamp_domain']:
        return reject('depth_timestamp_domain_mismatch')
    delta = profile['color_depth_delta_ms']
    if not np.isfinite(delta) or abs(delta) > protocol['color_depth_delta_max_ms']:
        return reject('color_depth_time_difference')
    scale = profile['scale_m']
    if not np.isfinite(scale) or scale <= 0:
        return reject('invalid_depth_scale')
    x, y = np.rint(pixel).astype(int)
    radius = protocol['depth_patch_radius_px']
    if not radius <= x < depth.shape[1] - radius or not radius <= y < depth.shape[0] - radius:
        return reject('outside_depth_support')
    patch = depth[y-radius:y+radius+1, x-radius:x+radius+1].astype(float) * scale
    lo, hi = protocol['depth_range_m']
    keep = (patch >= lo) & (patch <= hi)
    diagnostic['depth_fraction'] = float(keep.mean())
    if not keep[radius, radius] or keep.mean() < protocol['depth_fraction_min']:
        return reject('missing_surface_depth')
    samples = patch[keep]
    median = float(np.median(samples))
    iqr = float(np.percentile(samples, 75) - np.percentile(samples, 25))
    diagnostic.update(depth_m=median, depth_iqr_m=iqr, color_depth_delta_ms=delta)
    if iqr > protocol['depth_iqr_max_m'] or abs(patch[radius, radius] - median) > protocol['depth_iqr_max_m']:
        return reject('mixed_surface_depth')
    value = np.asarray(project(pixel, median, profile), float)
    if value.shape != (3,) or not np.isfinite(value).all():
        return reject('invalid_deprojection')
    return Observation(value.tolist(), **common), {**diagnostic, 'reason': 'surface_proxy_only'}
