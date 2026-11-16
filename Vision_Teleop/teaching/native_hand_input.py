"""Validated official hand detections and one fixed wrist/depth definition."""
from pathlib import Path
import hashlib,json
import numpy as np
from .dual_wilor import proper_rotation
# Same frozen smaller-envelope overlap as pose_candidate; keep this pure module
# usable in the GPU environment, which intentionally has no MuJoCo dependency.
CONFLICT_OVERLAP=.5

def configuration(root):
    config=json.loads((Path(root)/'config/wilor_native_input.json').read_text(encoding='utf8'))
    if config.get('inference_backend','eager') not in ('eager','compiled_graph'):
        raise ValueError('Unknown native inference backend')
    assets=Path(root)/config['detector_assets']
    if config['confidence']!=.3 or config['crop_padding']!=2 or config['fallback_source'] is not None:
        raise ValueError('Native hand input requires the frozen author settings and no fallback')
    if hashlib.sha256((assets/'detector.pt').read_bytes()).hexdigest()!=config['detector_sha256']:
        raise ValueError('Unrecognized official detector')
    if not (assets/'deps'/f"ultralytics-{config['ultralytics_version']}.dist-info/METADATA").exists():
        raise ValueError('Pinned detector dependencies missing')
    return config,assets

def select_detection(boxes):
    """No score winner or held-box recovery; retain opposite-label ambiguity."""
    b=np.asarray(boxes,float)
    out=dict(reason='no_hand',index=None,detected_count=len(b),boxes=b.tolist(),region_conflicts=[])
    if b.size==0:return out
    if b.ndim!=2 or b.shape[1]!=6 or not np.isfinite(b).all() or np.any(b[:,2:4]<=b[:,:2]) or np.any((b[:,4]<0)|(b[:,4]>1)) or not np.isin(b[:,5],[0,1]).all():
        return {**out,'reason':'invalid_detections'}
    right=np.flatnonzero(b[:,5]==1)
    if len(right)!=1:return {**out,'reason':'no_right_hand' if len(right)==0 else 'ambiguous_right_hands'}
    for i in right:
        for j in np.flatnonzero(b[:,5]==0):
            a,c=b[i,:4],b[j,:4];area=min(np.prod(a[2:]-a[:2]),np.prod(c[2:]-c[:2]))
            overlap=float(np.prod(np.maximum(0,np.minimum(a[2:],c[2:])-np.maximum(a[:2],c[:2])))/area)
            if overlap>=CONFLICT_OVERLAP:out['region_conflicts'].append(dict(indices=[int(i),int(j)],overlap_smaller=overlap))
    if out['region_conflicts']:return {**out,'reason':'overlapping_handedness_conflict'}
    return {**out,'reason':'selected','index':int(right[0])}

def hand_observation(prediction,selection):
    """Projected wrist is a surface locator, never an anatomical/bottle truth."""
    out=dict(points=[],handedness=[],selection=selection,points_source='wilor_projected_keypoints',
             position_definition='wilor_projected_wrist_left_measured_surface',object_tracking=False,
             anatomical_joint_center_verified=False,handedness_verified=False)
    if selection['reason']!='selected':return out
    points=np.asarray(prediction.get('pixels') if prediction else None,float)
    if not prediction or proper_rotation(prediction.get('rotation')) is None or points.shape!=(21,2) or not np.isfinite(points).all():
        return {**out,'selection':{**selection,'reason':'invalid_wilor_reconstruction','index':None}}
    return {**out,'points':[points.tolist()],'handedness':[dict(label='Right',score=selection['boxes'][selection['index']][4])]}
