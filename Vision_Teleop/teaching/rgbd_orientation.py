"""Diagnostic palm-surface orientation from measured depth and 2D anatomical labels.

No MediaPipe world landmarks, forearm substitution, temporal filling or stereo gate.
The fitted surface is not an anatomical joint plane. Not yet a robot input.
"""
import cv2
import numpy as np

from .rgbd_position import deproject, wrist_position
from .replay import palm_rotation


def palm_orientation(frame, hand, depth_settings, settings, project=deproject):
    result=dict(valid=False,rotation=None,points=None,source='rgbd_palm_surface_plane',
                anatomical_pose_verified=False,frame_id=frame.identity)
    def reject(reason):
        return {**result,'reason':reason}
    wrist=wrist_position(frame,hand,depth_settings,project)
    if not wrist['valid']:
        return reject('position_support: '+wrist['reason'])
    p=np.asarray(hand['points'],float)[0]
    d=frame.depth_image
    profile=frame.metadata['aligned_depth']
    support=p[[0,5,9,13,17]]
    if (support<0).any() or (support[:,0]>=d.shape[1]).any() or (support[:,1]>=d.shape[0]).any():
        return reject('palm_outside_image')
    lo=np.floor(support.min(axis=0)).astype(int)
    hi=np.ceil(support.max(axis=0)).astype(int)
    hi=np.minimum(hi,[d.shape[1]-1,d.shape[0]-1])
    mask=np.zeros((hi[1]-lo[1]+1,hi[0]-lo[0]+1),np.uint8)
    hull=cv2.convexHull(np.rint(support-lo).astype(np.int32))
    cv2.fillConvexPoly(mask,hull,1)
    margin=settings['border_px']
    mask=cv2.erode(mask,np.ones((2*margin+1,2*margin+1),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0)
    v,u=np.nonzero(mask)
    stride=settings['sample_stride_px']
    selected=(v%stride==0)&(u%stride==0)
    u,v=u[selected]+lo[0],v[selected]+lo[1]
    if len(u)<settings['min_samples']:
        return reject('insufficient_palm_area')
    if len(u)>settings['max_samples']:
        selected=np.linspace(0,len(u)-1,settings['max_samples'],dtype=int)
        u,v=u[selected],v[selected]
    depths=d[v,u].astype(float)*profile['scale_m']
    good=(depths>=depth_settings['min_depth_m'])&(depths<=depth_settings['max_depth_m'])
    result.update(sample_count=len(depths),valid_depth_fraction=float(good.mean()))
    if good.mean()<depth_settings['min_valid_fraction'] or good.sum()<settings['min_samples']:
        return reject('insufficient_palm_depth')
    try:
        cloud=np.asarray([project([x,y],z,profile) for x,y,z in zip(u[good],v[good],depths[good])],float)
    except (ValueError,RuntimeError) as exc:
        return reject('deprojection_failed: '+str(exc))
    if cloud.shape!=(int(good.sum()),3) or not np.isfinite(cloud).all():
        return reject('invalid_point_cloud')
    center=cloud.mean(axis=0)
    _,singular,axes=np.linalg.svd(cloud-center,full_matrices=False)
    normal=axes[-1]
    residual=np.abs((cloud-center)@normal)
    second_span=float(singular[1]/np.sqrt(len(cloud)))
    residual_p95=float(np.percentile(residual,95))
    result.update(plane_residual_p95_m=residual_p95,second_axis_std_m=second_span)
    if second_span<settings['min_second_axis_std_m']:
        return reject('plane_support_nearly_linear')
    if residual_p95>settings['max_plane_residual_p95_m']:
        return reject('palm_not_supported_by_plane')
    surface=[]
    support_mode=settings.get('landmark_support','direct')
    if support_mode not in ('direct','interior'):
        raise ValueError('Unknown landmark support mode')
    result['landmark_support']=support_mode
    for pixel in p[[0,5,9,17]]:
        try:
            ray=np.asarray(project(pixel,1.,profile),float)
        except (ValueError,RuntimeError) as exc:
            return reject('ray_deprojection_failed: '+str(exc))
        if ray.shape!=(3,) or not np.isfinite(ray).all() or np.linalg.norm(ray)<1e-9:
            return reject('invalid_palm_ray')
        denom=float(normal@ray)
        if abs(denom)/np.linalg.norm(ray)<settings['min_ray_plane_cosine']:
            return reject('palm_edge_on')
        point=ray*float(normal@center)/denom
        if not depth_settings['min_depth_m']<=point[2]<=depth_settings['max_depth_m']:
            return reject('plane_intersection_out_of_depth_range')
        # The interior variant validates the measured plane at an interior ray.
        # Original semantic rays still define orientation, not substitute joints.
        support_pixel=pixel if support_mode=='direct' else (pixel+support.mean(axis=0))/2
        support_ray=np.asarray(project(support_pixel,1.,profile),float)
        support_denom=float(normal@support_ray)
        if abs(support_denom)/np.linalg.norm(support_ray)<settings['min_ray_plane_cosine']:
            return reject('palm_edge_on')
        support_point=support_ray*float(normal@center)/support_denom
        x,y=np.rint(support_pixel).astype(int)
        radius=depth_settings['patch_radius_px']
        if not radius<=x<d.shape[1]-radius or not radius<=y<d.shape[0]-radius:
            return reject('landmark_outside_depth_support')
        patch=d[y-radius:y+radius+1,x-radius:x+radius+1].astype(float)*profile['scale_m']
        valid=(patch>=depth_settings['min_depth_m'])&(patch<=depth_settings['max_depth_m'])
        if not valid[radius,radius] or valid.mean()<depth_settings['min_valid_fraction']:
            return reject('landmark_depth_missing')
        if abs(float(np.median(patch[valid]))-support_point[2])>settings['max_landmark_plane_depth_difference_m']:
            return reject('landmark_depth_plane_conflict')
        if support_mode=='interior':
            # Never ignore contradictory measured depth at the semantic ray.
            ox,oy=np.rint(pixel).astype(int)
            if not 0<=ox<d.shape[1] or not 0<=oy<d.shape[0]:
                return reject('landmark_outside_depth_support')
            measured=float(d[oy,ox])*profile['scale_m']
            if depth_settings['min_depth_m']<=measured<=depth_settings['max_depth_m'] and abs(measured-point[2])>settings['max_landmark_plane_depth_difference_m']:
                return reject('landmark_depth_plane_conflict')
        surface.append(point)
    surface=np.asarray(surface)
    rotation=palm_rotation(surface)
    if rotation is None:
        return reject('degenerate_semantic_palm_axes')
    return {**result,'valid':True,'rotation':rotation.tolist(),'points':surface.tolist(),
            'reason':'measured_surface_orientation_not_anatomical_accuracy',
            'point_source':'2d_landmark_rays_intersect_measured_depth_plane'}
