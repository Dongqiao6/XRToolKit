"""Offline evidence checks for the existing frozen position maps.

No fitting, sensing, robot execution or online policy. A map's inverse is used
to ask whether independently specified world positions fit a declared model
domain, never to count inverse/forward agreement as task performance.
"""
import itertools
import numpy as np
from .human_activity import build_activity_domain, domain_coverage


def relative_error_radius(current_m, anchor_m):
    """Worst-case error of h(t)-h(bind); missing bounds remain unknown.

    Both supplied radii must bound the corresponding errors. Constant common
    bias can cancel, but that correlation cannot be assumed from two radii.
    """
    if current_m is None or anchor_m is None:
        return None
    values=np.asarray([current_m,anchor_m],float)
    if not np.isfinite(values).all() or np.any(values<0):
        raise ValueError('Finite nonnegative error radii required')
    return float(values.sum())


def world_region_bound(mapping, world_center_m, world_radius_m, input_error_m, *,
                       world_anchor_m, unit_m, cells_per_axis=8):
    """Cover the SAME world ball with fixed grid boxes, reusing existing bounds.

    Every intersecting box is retained, including portions outside the ball.
    Inverse Lipschitz L bounds its preimage by a ball about F^-1(box center).
    The forward derivative bound over that ball plus input error yields a
    finite-error bound. Max over boxes covers the whole world ball, unlike a
    test only at inverse endpoints. Conservative float arithmetic, not interval
    certification. Subdivision count is fixed before evaluation, not until PASS.
    """
    center=np.asarray(world_center_m,float);anchor=np.asarray(world_anchor_m,float)
    if center.shape!=(3,) or anchor.shape!=(3,) or not np.isfinite([center,anchor]).all() or mapping.dimension!=3:
        raise ValueError('Finite 3D world center and anchor required')
    if not np.isfinite([world_radius_m,unit_m]).all() or world_radius_m<0 or unit_m<=0:
        raise ValueError('Finite nonnegative world radius and positive unit required')
    if input_error_m is None:
        return dict(status='unknown_input_error_bound',finite_error_upper_m=None)
    if not np.isfinite(input_error_m) or input_error_m<0 or not isinstance(cells_per_axis,int) or not 1<=cells_per_axis<=16:
        raise ValueError('Finite error radius and bounded integer grid required')
    r=world_radius_m/unit_m;n=cells_per_axis;half=r/n
    offsets=np.array(list(itertools.product((np.arange(n)+.5)*2*half-r,repeat=3)))
    closest=np.maximum(np.abs(offsets)-half,0)
    offsets=offsets[np.linalg.norm(closest,axis=1)<=r+1e-12]
    y=(center-anchor)/unit_m+offsets;inverse=mapping.inverse(y)
    inverse_lip=1/mapping.scale*np.prod([1/(1-layer.q) for layer in mapping.layers])
    inverse_cover=inverse_lip*np.sqrt(3)*half
    gain=mapping.ball_gain_bound(inverse,inverse_cover+input_error_m/unit_m)
    upper=float(np.max(gain)*input_error_m)
    return dict(status='model_float_upper_bound',finite_error_upper_m=upper,
        world_radius_m=float(world_radius_m),input_error_m=float(input_error_m),
        inverse_lipschitz=float(inverse_lip),retained_boxes=len(offsets),cells_per_axis=n,
        max_box_preimage_radius_m=float(inverse_cover*unit_m),
        scope='Entire stated world ball and all bounded input perturbations; conditional model bound, not observed accuracy')


def inspect_model_task(mapping, domain_spec, world_points_m, *, human_anchor_m,
                       world_anchor_m, unit_m, endpoint_relation):
    """Conditional position feasibility only; full pose/robot claims stay unknown.

    This minimal audit currently accepts only the named synthetic common axes
    used by the existing Gao experiment. Physical deployment is explicitly
    rejected rather than silently identifying the measured surface with wrist.
    """
    domain=build_activity_domain(domain_spec)
    result=dict(status='insufficient_model_input',position_domain=None,
        personal_comfort_supported=False,full_pose_supported=None,
        robot_execution_supported=None,complete_task_supported=None,
        inverse_roundtrip_is_task_evidence=False)
    if domain_spec.get('scope')!='synthetic' or endpoint_relation!='synthetic_same_point_assumption':
        result.update(status='physical_control_to_model_relation_not_established')
        return result
    if domain['status']!='ready':
        result.update(domain=domain);return result
    points=np.asarray(world_points_m,float);h=np.asarray(human_anchor_m,float);w=np.asarray(world_anchor_m,float)
    if points.ndim!=2 or points.shape[1]!=3 or not 1<=len(points)<=256 or h.shape!=(3,) or w.shape!=(3,) or not all(np.isfinite(x).all() for x in [points,h,w]) or not np.isfinite(unit_m) or unit_m<=0:
        raise ValueError('Finite independent task and explicit anchors/units required')
    inverse=mapping.inverse((points-w)/unit_m);human=h+unit_m*inverse
    coverage=domain_coverage(domain,human,frame=domain['frame'],path_kind='polyline')
    jac=mapping.forward(inverse,True)[1];sv=np.linalg.svd(jac,compute_uv=False)
    result.update(status='conditional_model_position_checked',position_domain=coverage,
        required_model_points_m=human.tolist(),world_points_m=points.tolist(),
        singular_min=float(sv[:,-1].min()),singular_max=float(sv[:,0].max()),
        condition_max=float((sv[:,0]/sv[:,-1]).max()),
        human_polyline_length_m=float(np.linalg.norm(np.diff(human,axis=0),axis=1).sum()),
        domain_scope='Explicit inverse sample polyline only; not continuous inverse world curve or anatomical posture',
        orientation_scope='Existing independent hand rotation preserved; no shoulder/elbow/wrist orientation admissibility model')
    return result
