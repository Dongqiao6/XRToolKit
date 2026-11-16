"""Small offline two-link position domain and fixed similarity initialization.

No sensing, comfort score, runtime gain adjustment, robot solver or new package.
The endpoint is an explicitly named MODEL wrist, not a depth surface by default.
The directional cone constrains shoulder-to-wrist direction, NOT shoulder joint
angles. Shoulder rotations and elbow swivel are otherwise free in this model.
"""
import numpy as np

from teaching.human_geometry import ArmLength, confirmed_range_scale, rotation

PARAMS = ('upper_arm_m', 'forearm_m', 'elbow_flexion_rad', 'extension_fraction',
          'direction_axis', 'direction_half_angle_rad')
KINDS = {'synthetic', 'model_assumption', 'literature_inspired',
         'independently_measured', 'personal_confirmed'}
NUMERIC_TOL = 1e-10  # Algebraic roundoff only, not a perception/comfort threshold.


def _provenance(p):
    return isinstance(p, dict) and p.get('kind') in KINDS and bool(p.get('source_id'))


def _vec(x):
    a = np.asarray(x, float)
    if a.shape != (3,) or not np.isfinite(a).all():
        raise ValueError('finite three-vector required')
    return a


def build_activity_domain(spec):
    """A radial shell intersected with a convex circular cone (half-angle <=90°).

    Radius follows two rigid links and elbow flexion beta (zero = straight).
    An optional extension interval further clips radius. It is an existential
    endpoint set, not proof of the actual operator's posture or comfort.
    """
    spec = spec or {}
    missing = [k for k in ('frame','endpoint_reference','scope','fixed_reference') if not spec.get(k)]
    params = spec.get('parameters') or {}
    for k in PARAMS:
        if k not in params: missing.append('parameters.'+k)
        elif not isinstance(params[k],dict) or 'value' not in params[k] or not _provenance(params[k]):
            missing.append('parameters.'+k+'.value_or_provenance')
    ref = spec.get('fixed_reference') or {}
    for k in ('origin_m','R_frame_domain','stamp_s','clock','source_id','kind'):
        if ref.get(k) is None: missing.append('fixed_reference.'+k)
    out = dict(status='insufficient_information', missing=missing, specification=spec,
        comfort_validated=False, robot_feasibility_checked=False,
        anatomical_angles_observed=False, shoulder_policy='fixed_initial_reference_never_runtime_subtraction',
        endpoint_existence_does_not_identify_actual_posture=True)
    if missing: return out
    try:
        if spec['scope'] not in ('synthetic','model_only','personal_parameters'):
            raise ValueError('explicit domain scope required')
        if not _provenance(ref) or not ref['clock'] or not np.isfinite(ref['stamp_s']):
            raise ValueError('fixed reference provenance/time required')
        if spec['scope']=='synthetic' and any(p['kind']!='synthetic' for p in [ref,*params.values()]):
            raise ValueError('synthetic domain must not disguise real parameter provenance')
        if spec['scope']!='synthetic' and any(p['kind']=='synthetic' for p in [ref,*params.values()]):
            raise ValueError('synthetic parameters cannot complete a real domain')
        a,b = [float(params[k]['value']) for k in ('upper_arm_m','forearm_m')]
        beta = np.asarray(params['elbow_flexion_rad']['value'],float)
        extension = np.asarray(params['extension_fraction']['value'],float)
        if not np.isfinite([a,b]).all() or min(a,b)<=0: raise ValueError('positive fixed lengths required')
        if beta.shape!=(2,) or not np.isfinite(beta).all() or not 0<=beta[0]<=beta[1]<=np.pi:
            raise ValueError('ordered elbow flexion interval in [0,pi] required')
        if extension.shape!=(2,) or not np.isfinite(extension).all() or not 0<=extension[0]<=extension[1]<=1:
            raise ValueError('ordered extension interval in [0,1] required')
        axis = _vec(params['direction_axis']['value'])
        if not np.isclose(np.linalg.norm(axis),1.,atol=1e-10,rtol=0):
            raise ValueError('explicit unit direction axis required')
        angle = float(params['direction_half_angle_rad']['value'])
        if not np.isfinite(angle) or not 0<=angle<=np.pi/2:
            raise ValueError('this bounded model supports only convex direction cones <= pi/2')
        origin = _vec(ref['origin_m']); R = rotation(ref['R_frame_domain'])
        lo = max(np.sqrt(max(0,a*a+b*b+2*a*b*np.cos(beta[1]))),(a+b)*extension[0])
        hi = min(np.sqrt(max(0,a*a+b*b+2*a*b*np.cos(beta[0]))),(a+b)*extension[1])
        out.update(status='ready' if lo<=hi else 'empty_domain', radius_m=[float(lo),float(hi)],
            frame=spec['frame'],endpoint_reference=spec['endpoint_reference'],
            origin_m=origin.tolist(),R_frame_domain=R.tolist(),axis=axis.tolist(),half_angle_rad=angle,
            length_sum_m=a+b,model='two_link_radial_shell_intersect_direction_cone',
            omitted_constraints=['shoulder_anatomical_joint_limits','elbow_swivel_limits',
                'forearm_axial_rotation','wrist_angles','body_collision','load','fatigue','robot_constraints'])
    except (ValueError,TypeError,KeyError) as exc:
        out.update(status='invalid_parameters',error=str(exc))
    return out


def domain_coverage(domain, points, *, frame, path_kind='points'):
    """Exact shell/cone checks for named points or every point of a polyline.

    Cone is convex: endpoints suffice there. Shell is not: analytically minimize
    radius on every line segment. This does NOT certify arbitrary paths between
    point tasks, boxes, body configurations, time requirements or robot paths.
    """
    if domain['status']!='ready': return dict(status=domain['status'],covered=False)
    if frame!=domain['frame']: return dict(status='frame_mismatch',covered=False)
    if path_kind not in ('points','polyline'): return dict(status='unsupported_task_set',covered=False)
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=3 or not 1<=len(p)<=256 or not np.isfinite(p).all():
        return dict(status='invalid_points',covered=False)
    local=(p-np.array(domain['origin_m']))@np.array(domain['R_frame_domain'])
    radius=np.linalg.norm(local,axis=1);lo,hi=domain['radius_m']
    z=local@np.array(domain['axis']);c=np.cos(domain['half_angle_rad'])
    cone=z+NUMERIC_TOL>=c*radius
    inside=cone & (radius+NUMERIC_TOL>=lo) & (radius<=hi+NUMERIC_TOL)
    segments=[]
    if path_kind=='polyline':
        for i,(a,b) in enumerate(zip(local[:-1],local[1:])):
            delta=b-a;den=float(delta@delta)
            u=float(np.clip(-a@delta/den,0,1)) if den else 0.
            minimum=float(np.linalg.norm(a+u*delta))
            good=bool(inside[i] and inside[i+1] and minimum+NUMERIC_TOL>=lo)
            segments.append(dict(segment=i,minimum_radius_m=minimum,minimum_at_fraction=u,covered=good))
    return dict(status='checked',covered=bool(np.all(inside) and all(x['covered'] for x in segments)),
        point_inside=inside.tolist(),radii_m=radius.tolist(),segments=segments,
        coverage_scope='entire_explicit_polyline_position_only' if path_kind=='polyline' else 'listed_points_only',
        actual_human_posture_checked=False,orientation_or_robot_checked=False)


def _intervals(constraints, lo, hi):
    """Closed feasible intervals of quadratic inequalities on bounded inverse gain.

    Roots partition the interval into cells with constant sign. Keep isolated
    feasible roots as well; never interpolate across an infeasible inner hole.
    """
    roots=[lo,hi]
    for coeff in constraints:
        coeff=np.trim_zeros(np.array(coeff,float),'b')
        if len(coeff)<2: continue
        for r in np.polynomial.polynomial.polyroots(coeff):
            if abs(r.imag)<=1e-10 and lo<r.real<hi: roots.append(float(r.real))
    roots=sorted(set(roots))
    good=lambda x: all(np.polynomial.polynomial.polyval(x,q)<=NUMERIC_TOL for q in constraints)
    pieces=[(x,x) for x in roots if good(x)]
    for a,b in zip(roots[:-1],roots[1:]):
        if good((a+b)/2): pieces.append((a,b))
    merged=[]
    for a,b in sorted(pieces):
        if merged and a<=merged[-1][1]: merged[-1][1]=max(b,merged[-1][1])
        else: merged.append([a,b])
    return merged


def fixed_mapping_suggestion(domain_spec, task, mapping):
    """Offline scalar candidates + pointwise intervals + separate path checks.

    p_world = task_anchor + g*A*(p_control-human_anchor), g>0.
    Requires explicit physical endpoint/control reference binding, not just R/t.
    A confirmed span supplies a nominal scale through confirmed_range_scale;
    the geometric coverage checks here may reject that nominal scale.
    """
    if ((mapping or {}).get('endpoint_binding') or {}).get('relation') == 'uncertain_reference':
        return _uncertain_mapping_suggestion(domain_spec, task, mapping)
    domain=build_activity_domain(domain_spec)
    out=dict(status='insufficient_information',domain=domain,missing=list(domain['missing']),
        deployable=False,personal_comfort_recommendation=False,optimality_claim=False,
        orientation_policy='existing_independent_relative_rotation_not_scaled',
        runtime_gain_change=False,robot_feasibility_checked=False)
    task=task or {};mapping=mapping or {}
    for k in ('frame','path_kind','points_m','provenance'):
        if task.get(k) is None: out['missing'].append('task.'+k)
    for k in ('input_frame','task_frame','human_anchor_m','task_anchor_m','R_world_input',
              'control_point_reference','endpoint_binding','scale_range','provenance'):
        if mapping.get(k) is None: out['missing'].append('mapping.'+k)
    binding=mapping.get('endpoint_binding') or {}
    for k in ('model_reference','control_reference','relation','validated','kind','source_id'):
        if binding.get(k) is None: out['missing'].append('mapping.endpoint_binding.'+k)
    if out['missing']: return out
    if domain['status']!='ready': out['status']=domain['status'];return out
    try:
        if not _provenance(task['provenance']) or not _provenance(mapping['provenance']):
            raise ValueError('task and mapping provenance required')
        synthetic=domain_spec['scope']=='synthetic'
        if any((p['kind']=='synthetic')!=synthetic for p in (task['provenance'],mapping['provenance'])):
            raise ValueError('synthetic and real parameter scopes cannot be mixed')
        if (binding['validated'] is not True or binding['relation']!='same_physical_reference'
                or binding['model_reference']!=domain['endpoint_reference']
                or binding['control_reference']!=mapping['control_point_reference']
                or binding['kind']!=('synthetic' if synthetic else 'validated_reference_relation')
                or not binding['source_id']):
            out.update(status='physical_reference_binding_required',
                missing=['trusted_model_endpoint_to_control_point_relation']);return out
        if task['frame']!=mapping['task_frame'] or domain['frame']!=mapping['input_frame']:
            raise ValueError('explicit task/input coordinate binding mismatch')
        if task['path_kind'] not in ('points','polyline'):
            raise ValueError('only explicit points/polyline supported; no bounding-box coverage inference')
        A=rotation(mapping['R_world_input']);h=_vec(mapping['human_anchor_m']);w=_vec(mapping['task_anchor_m'])
        pts=np.asarray(task['points_m'],float)
        if pts.ndim!=2 or pts.shape[1]!=3 or not 1<=len(pts)<=256 or not np.isfinite(pts).all():
            raise ValueError('1..256 finite task points required')
        bounds=np.asarray(mapping['scale_range'],float)
        if bounds.shape!=(2,) or not np.isfinite(bounds).all() or not 0<bounds[0]<=bounds[1]:
            raise ValueError('explicit positive bounded scale range required')
        out['anchor_coverage']=domain_coverage(domain,[h],frame=domain['frame'])
        out['initial_anchor_inside_domain']=out['anchor_coverage']['covered']
        # The task anchor is NOT silently inserted into a point task/path. An
        # outside initialization is reported and prevents a personal recommendation.
        local_R=np.array(domain['R_frame_domain']);origin=np.array(domain['origin_m'])
        c=(h-origin)@local_R;directions=(pts-w)@A@local_R
        axis=np.array(domain['axis']);cos2=np.cos(domain['half_angle_rad'])**2
        rmin,rmax=domain['radius_m'];constraints=[]
        for v in directions:
            q=np.array([c@c,2*c@v,v@v]);zc,zv=c@axis,v@axis
            constraints += [q-np.array([rmax*rmax,0,0]),np.array([rmin*rmin,0,0])-q,
                np.array([-zc,-zv,0]),cos2*q-np.array([zc*zc,2*zc*zv,zv*zv])]
        inv=_intervals(constraints,1/bounds[1],1/bounds[0])
        scales=sorted([[1/b,1/a] for a,b in inv])
        out.update(pointwise_scale_intervals=scales,
            interval_scope='listed_task_points_only_within_supplied_scale_range',
            interval_numeric_tolerance=NUMERIC_TOL,
            gain_feasibility_assumed_monotone=False,mapping_provenance=mapping,
            task_provenance=task,nominal_span_proposal=None)
        candidates=list(mapping.get('candidate_scales',[]))
        span=mapping.get('confirmed_span')
        if span is not None:
            if not _provenance(span) or span['kind']!=('synthetic' if synthetic else 'personal_confirmed'):
                raise ValueError('explicit personal span confirmation or synthetic fixture required')
            length=ArmLength(domain['length_sum_m'],'explicit_fixed_segments',
                '+'.join(domain_spec['parameters'][n]['source_id'] for n in ('upper_arm_m','forearm_m')),
                'synthetic_test' if synthetic else 'estimated_proxy')
            proposal=confirmed_range_scale(length,span['fraction'],span['task_span_m'],confirmation_id=span['source_id'])
            out['nominal_span_proposal']=proposal;candidates.insert(0,proposal['scale'])
        # One deterministic midpoint per connected pointwise interval, not a scan
        # or optimum search; supplied candidates are preserved even when rejected.
        candidates += [(a+b)/2 for a,b in scales]
        if len(candidates)>32: raise ValueError('bounded candidate budget exceeded (32); return a smaller explicit task')
        if any(not np.isfinite(g) or g<=0 for g in candidates):raise ValueError('positive finite candidate scale required')
        records=[]
        for g in dict.fromkeys(float(x) for x in candidates):
            preimage=h+(pts-w)@A/g
            check=domain_coverage(domain,preimage,frame=domain['frame'],path_kind=task['path_kind'])
            in_range=bool(bounds[0]<=g<=bounds[1])
            rec=dict(scale=g,within_scale_range=in_range,coverage=check,human_positions_m=preimage.tolist(),
                position_noise_gain=g,task_displacement_to_human_gain=1/g,
                acceptable_for_stated_geometry=bool(in_range and check['covered']),
                comfort_verified=False,robot_verified=False)
            records.append(rec)
        out['candidates']=records
        passed=[r['scale'] for r in records if r['acceptable_for_stated_geometry']]
        out.update(status=('no_pointwise_scale_in_supplied_range' if not scales else
            'geometric_candidates_available' if passed else 'no_checked_candidate_covers_path'),
            geometric_candidate_scales=passed,
            no_single_scale_proven=not bool(scales),
            continuous_path_scale_intervals_computed=False,
            note='Position set only; no human posture, comfort, time or robot feasibility guarantee. Fixed shoulder is initialization geometry, never live subtraction.')
    except (ValueError,TypeError,KeyError) as exc:
        out.update(status='invalid_parameters',error=str(exc))
    return out


def _radial_intersection(domain, length_intervals):
    """Intersection of radial shells over an independent two-length rectangle.

    Direction, elbow and extension ranges are fixed MODEL restrictions, not
    measured human limits. Max convex norm occurs at corners; min occurs at
    corners or stationary points on edges (or the equal-link pi degeneracy).
    """
    a, b = length_intervals
    params = domain['specification']['parameters']
    beta = params['elbow_flexion_rad']['value']
    extension = params['extension_fraction']['value']
    corners = [(x, y) for x in a for y in b]
    norm = lambda x, y, c: np.sqrt(max(0., x*x+y*y+2*c*x*y))
    lower = max(max(norm(x, y, np.cos(beta[1])) for x, y in corners),
                (a[1]+b[1])*extension[0])
    c = np.cos(beta[0])
    minima = corners + [(x, float(np.clip(-c*x, *b))) for x in a]
    minima += [(float(np.clip(-c*y, *a)), y) for y in b]
    upper = min(min(norm(x, y, c) for x, y in minima), (a[0]+b[0])*extension[1])
    return [float(lower), float(upper)]


def _tube_coverage(domain, points, radius, radial, path_kind):
    """Conservative ball tube around points/a full polyline in a shell/cone.

    radius is a worst-case Minkowski envelope of explicitly supplied sets;
    it is NOT a causal decomposition or a statistically inferred confidence.
    Failure of this sufficient test is indeterminate, never global infeasibility.
    """
    local = (points-np.array(domain['origin_m'])) @ np.array(domain['R_frame_domain'])
    norms = np.linalg.norm(local, axis=1)
    minimum, maximum = float(min(norms)), float(max(norms))
    if path_kind == 'polyline':
        for a, b in zip(local[:-1], local[1:]):
            v = b-a; den = v@v
            u = float(np.clip(-a@v/den, 0, 1)) if den else 0.
            minimum = min(minimum, float(np.linalg.norm(a+u*v)))
    axis = np.array(domain['axis']); z = local@axis
    perp = np.linalg.norm(local-z[:, None]*axis, axis=1)
    theta = domain['half_angle_rad']
    # This clearance is concave along each segment, hence endpoints suffice.
    cone_margin = float(min(np.sin(theta)*z-np.cos(theta)*perp))-radius
    inner_margin = minimum-radius-radial[0] if radial[0] > 0 else None
    outer_margin = radial[1]-maximum-radius
    covered = (radial[0] <= radial[1] and cone_margin >= -NUMERIC_TOL
               and outer_margin >= -NUMERIC_TOL
               and (inner_margin is None or inner_margin >= -NUMERIC_TOL))
    return dict(status='conditional_covered' if covered else 'indeterminate', covered=bool(covered),
        inner_margin_m=inner_margin, outer_margin_m=float(outer_margin), cone_margin_m=cone_margin,
        centerline_minimum_radius_m=minimum, centerline_maximum_radius_m=maximum,
        uncertainty_envelope_radius_m=radius,
        coverage_scope='entire_explicit_polyline_position_only' if path_kind=='polyline' else 'listed_points_only',
        failure_does_not_prove_infeasibility=True)


def _uncertain_mapping_suggestion(domain_spec, task, mapping):
    """Existing relative mapping, uncertain model endpoint, explicit candidates.

    c=m+b => m_req=m0+A.T*(pW-w0)/g - (b-b0). No correction is
    applied to control. Only zero-centered isotropic uncertainty sets supported.
    Missing bounds never become zero. No gain scan/interval optimality claim.
    """
    domain = build_activity_domain(domain_spec)
    out = dict(status='insufficient_information', missing=list(domain['missing']), domain=domain,
        deployable=False, personal_comfort_recommendation=False, robot_feasibility_checked=False,
        orientation_policy='existing_independent_relative_rotation_not_scaled', runtime_gain_change=False,
        no_single_scale_proven=False, uncertainty_branch=True, geometric_candidate_scales=[])
    task = task or {}; binding = mapping['endpoint_binding']
    u = binding.get('uncertainty') or {}
    for prefix, obj, keys in (
        ('task', task, ('frame','path_kind','points_m','provenance')),
        ('mapping', mapping, ('input_frame','task_frame','human_anchor_m','task_anchor_m','R_world_input',
                             'control_point_reference','scale_range','candidate_scales','provenance')),
        ('endpoint_binding', binding, ('model_reference','control_reference','source_id')),
        ('uncertainty', u, ('frame','anchor_event_id','valid_motion_scope','source_ids','evidence_level',
            'model_anchor_m','model_anchor_radius_m','shoulder_radius_m','delta_b_radius_m',
            'length_intervals_m','fixed_model_restrictions_acknowledged'))):
        for key in keys:
            if obj.get(key) is None: out['missing'].append(prefix+'.'+key)
    if out['missing']: return out
    if domain['status'] != 'ready': out['status'] = domain['status']; return out
    try:
        levels = {'model_assumption_sensitivity_only', 'finite_record_empirical_envelope_only',
                  'independently_supported_bound_with_scope'}
        if u['evidence_level'] not in levels: raise ValueError('explicit uncertainty evidence level required')
        if not isinstance(u['source_ids'], list) or not u['source_ids'] or not all(isinstance(x,str) and x.strip() for x in u['source_ids']):
            raise ValueError('nonempty uncertainty source references required')
        if not all(isinstance(x,str) and x.strip() for x in
                   (u['anchor_event_id'], u['valid_motion_scope'], binding['source_id'])):
            raise ValueError('anchor event, scope and binding source required')
        if u['fixed_model_restrictions_acknowledged'] is not True:
            raise ValueError('direction, elbow and extension are conditional fixed model restrictions')
        supported = u['evidence_level'] == 'independently_supported_bound_with_scope'
        if supported and (not isinstance(u.get('support_review_id'),str) or not u['support_review_id'].strip()):
            out['missing'].append('uncertainty.support_review_id'); return out
        synthetic = domain_spec['scope'] == 'synthetic'
        if synthetic and supported: raise ValueError('synthetic inputs cannot certify a physical bound')
        for p in (task['provenance'], mapping['provenance']):
            if not _provenance(p) or (p['kind']=='synthetic') != synthetic:
                raise ValueError('consistent task and mapping provenance required')
        if (binding['model_reference'] != domain['endpoint_reference'] or
                binding['control_reference'] != mapping['control_point_reference']):
            raise ValueError('endpoint references mismatch')
        if not (u['frame'] == domain['frame'] == mapping['input_frame'] and task['frame'] == mapping['task_frame']):
            raise ValueError('explicit uncertainty/task/input frame mismatch')
        if task['path_kind'] not in ('points','polyline'): raise ValueError('explicit points/polyline required')
        A = rotation(mapping['R_world_input']); m0 = _vec(u['model_anchor_m'])
        _vec(mapping['human_anchor_m'])  # original control anchor retained, NOT substituted for m0
        w0 = _vec(mapping['task_anchor_m'])
        pts = np.asarray(task['points_m'],float)
        if pts.ndim != 2 or pts.shape[1] != 3 or not 1 <= len(pts) <= 256 or not np.isfinite(pts).all():
            raise ValueError('1..256 finite task points required')
        radii = np.asarray([u[k] for k in ('model_anchor_radius_m','shoulder_radius_m','delta_b_radius_m')],float)
        if radii.shape != (3,) or not np.isfinite(radii).all() or np.any(radii < 0):
            raise ValueError('explicit nonnegative uncertainty radii required')
        lengths = np.asarray(u['length_intervals_m'],float)
        nominal = np.array([domain_spec['parameters'][k]['value'] for k in ('upper_arm_m','forearm_m')])
        if (lengths.shape != (2,2) or not np.isfinite(lengths).all() or np.any(lengths <= 0)
                or np.any(lengths[:,0] > nominal) or np.any(lengths[:,1] < nominal)):
            raise ValueError('ordered positive segment intervals containing nominal dimensions required')
        bounds = np.asarray(mapping['scale_range'],float); gains = np.asarray(mapping['candidate_scales'],float)
        if bounds.shape != (2,) or not np.isfinite(bounds).all() or not 0 < bounds[0] <= bounds[1]:
            raise ValueError('explicit positive scale range required')
        if gains.ndim != 1 or not 1 <= len(gains) <= 32 or not np.isfinite(gains).all() or np.any(gains <= 0):
            raise ValueError('1..32 explicit positive candidates required; no uncertainty gain search')
        radial = _radial_intersection(domain, lengths)
        radius = float(radii.sum())
        anchor = _tube_coverage(domain, m0[None,:], float(radii[:2].sum()), radial, 'points')
        records = []
        for g in dict.fromkeys(float(x) for x in gains):
            centers = m0+(pts-w0)@A/g
            coverage = _tube_coverage(domain, centers, radius, radial, task['path_kind'])
            in_range = bool(bounds[0] <= g <= bounds[1])
            qualifies = bool(in_range and anchor['covered'] and coverage['covered'])
            records.append(dict(scale=g, within_scale_range=in_range, model_centerline_m=centers.tolist(),
                coverage=coverage, conditional_geometry_covered=qualifies,
                supported_bound_coverage=bool(supported and qualifies),
                acceptable_for_stated_geometry=bool(supported and qualifies),
                comfort_verified=False, robot_verified=False, position_noise_gain=g))
        out.update(status='conditional_bound_check_complete' if supported else 'sensitivity_only',
            evidence_level=u['evidence_level'], source_evidence_independently_verified_by_software=False,
            uncertainty_contract=u, mapping_provenance=mapping, task_provenance=task,
            anchor_coverage=anchor, radial_intersection_m=radial, candidates=records,
            conditional_candidate_scales=[r['scale'] for r in records if r['conditional_geometry_covered']],
            geometric_candidate_scales=[r['scale'] for r in records if r['supported_bound_coverage']],
            continuous_path_scale_intervals_computed=False,
            note='Conditional fixed position model only. Supplied evidence is recorded, not authenticated. No personal comfort, sensor accuracy, posture continuity, orientation, time or robot guarantee.')
    except (ValueError, TypeError, KeyError) as exc:
        out.update(status='invalid_parameters', error=str(exc))
    return out
