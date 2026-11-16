"""Measured wrist-region surface position, not an anatomical joint-center truth."""
import numpy as np


def deproject(pixel, depth_m, profile):
    import pyrealsense2 as rs
    intr = rs.intrinsics()
    for key in ('width','height','fx','fy','ppx','ppy'):
        setattr(intr,key,profile[key])
    model = profile['distortion'].split('.')[-1]
    if not hasattr(rs.distortion,model):
        raise ValueError('Unsupported SDK distortion model')
    intr.model = getattr(rs.distortion,model)
    intr.coeffs = profile['coeffs']
    return rs.rs2_deproject_pixel_to_point(intr,list(map(float,pixel)),float(depth_m))


def wrist_position(frame, hand, settings, project=deproject):
    result=dict(valid=False,position_m=None,source='primary_rgbd_wrist_surface_proxy',
                frame_id=frame.identity,camera_role=frame.metadata['camera_role'],
                host_arrival_monotonic_ns=frame.metadata['host_arrival_monotonic_ns'],
                anatomical_joint_center_verified=False)
    def reject(reason):
        return {**result,'reason':reason}
    p=np.asarray(hand.get('points',[]),float)
    if p.shape!=(1,21,2) or not np.isfinite(p).all():
        return reject('single_hand_required')
    d=frame.depth_image
    profile=frame.metadata.get('aligned_depth')
    if d is None or profile is None:
        return reject('aligned_depth_missing')
    if d.shape!=frame.image.shape[:2] or d.dtype!=np.uint16:
        return reject('depth_color_shape_or_type_mismatch')
    if profile['timestamp_domain'] != frame.metadata['timestamp_domain']:
        return reject('color_depth_timestamp_domain_mismatch')
    if not np.isfinite(profile['scale_m']) or profile['scale_m']<=0:
        return reject('invalid_depth_scale')
    delta=profile['color_depth_delta_ms']
    if not np.isfinite(delta) or abs(delta)>settings['max_color_depth_delta_ms']:
        return reject('color_depth_time_difference')
    x,y=np.rint(p[0,0]).astype(int)
    radius=settings['patch_radius_px']
    if not radius<=x<d.shape[1]-radius or not radius<=y<d.shape[0]-radius:
        return reject('wrist_outside_depth_support')
    patch=d[y-radius:y+radius+1,x-radius:x+radius+1].astype(float)*profile['scale_m']
    valid=(patch>=settings['min_depth_m']) & (patch<=settings['max_depth_m'])
    result['valid_depth_fraction']=float(valid.mean())
    if not valid[radius,radius] or valid.mean()<settings['min_valid_fraction']:
        return reject('depth_hole_or_out_of_range')
    values=patch[valid]
    median=float(np.median(values))
    spread=float(np.percentile(values,90)-np.percentile(values,10))
    result.update(depth_m=median,depth_spread_m=spread,pixel_xy=p[0,0].tolist())
    if spread>settings['max_depth_spread_m'] or abs(patch[radius,radius]-median)>settings['max_center_deviation_m']:
        return reject('depth_edge_or_mixed_surfaces')
    palm_depths=[]
    for j in (5,9,17):
        u,v=np.rint(p[0,j]).astype(int)
        if 0<=u<d.shape[1] and 0<=v<d.shape[0]:
            value=float(d[v,u])*profile['scale_m']
            if settings['min_depth_m']<=value<=settings['max_depth_m']:
                palm_depths.append(value)
    if len(palm_depths)<2:
        return reject('palm_depth_support_missing')
    result['wrist_palm_depth_difference_m']=abs(median-float(np.median(palm_depths)))
    if result['wrist_palm_depth_difference_m']>settings.get('max_wrist_palm_depth_difference_m',.15):
        return reject('wrist_palm_depth_inconsistent')
    try:
        xyz=np.asarray(project(p[0,0],median,profile),float)
    except (ValueError,RuntimeError) as exc:
        return reject('deprojection_failed: '+str(exc))
    if xyz.shape!=(3,) or not np.isfinite(xyz).all():
        return reject('invalid_deprojection')
    return {**result,'valid':True,'position_m':xyz.tolist(),'reason':'measured_surface_proxy'}
