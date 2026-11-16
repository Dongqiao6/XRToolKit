"""Offline right-arm association to an independently selected same-frame hand.

No largest-person fallback, temporal identity claim, or live control integration.
Distances compare two projected wrist estimates, not anatomical ground truth.
"""
import numpy as np


def bind_right_arm(points, scores, hand, *, body_frame_id, hand_frame_id,
                   max_distance_px, margin_px, score_min):
    result=dict(valid=False,index=None,reason='missing_hand',distances_px=[],live_deployed=False)
    if not body_frame_id or body_frame_id!=hand_frame_id:
        return {**result,'reason':'frame_mismatch'}
    if hand.get('selection',{}).get('reason')!='selected':return result
    h=np.asarray(hand.get('points'),float)
    p=np.asarray(points,float);s=np.asarray(scores,float)
    if h.shape!=(1,21,2) or not np.isfinite(h).all():return result
    if p.ndim!=3 or p.shape[1]<11 or p.shape[2]!=2 or s.shape!=p.shape[:2]:
        return {**result,'reason':'invalid_body_shape'}
    if not (np.isfinite([max_distance_px,margin_px,score_min]).all() and max_distance_px>0 and margin_px>0 and 0<=score_min<=1):
        raise ValueError('Explicit finite association limits required')
    candidates=[]
    for i,(body,conf) in enumerate(zip(p,s)):
        if not np.isfinite(body[10]).all() or not np.isfinite(conf[10]) or conf[10]<score_min:
            result['distances_px'].append(None);continue
        distance=float(np.linalg.norm(body[10]-h[0,0]));result['distances_px'].append(distance)
        candidates.append((distance,i))
    candidates.sort()
    if not candidates or candidates[0][0]>max_distance_px:return {**result,'reason':'no_wrist_match'}
    if len(candidates)>1 and candidates[1][0]-candidates[0][0]<margin_px:
        return {**result,'reason':'ambiguous_wrist_match'}
    return {**result,'valid':True,'index':candidates[0][1],'reason':'same_frame_right_hand_association'}
