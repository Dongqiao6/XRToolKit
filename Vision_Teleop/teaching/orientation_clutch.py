"""Explicit palm rebind, with atomic position rebind for the task demo."""
import numpy as np


class OrientationClutch:
    def __init__(self, world_from_camera, position_scale=None):
        self.A=np.asarray(world_from_camera,float).copy()
        self.hand=None;self.anchor=None;self.binding_id=0
        self.scale=position_scale
        self.point=None;self.position_anchor=None

    def bind(self, pose, held_command, now, *, point=None, position_stamp=None, held_position=None):
        # Called only for an explicit start/resume, never on ordinary recovery.
        if not pose or not pose.get('valid') or pose.get('rotation') is None:
            return None
        stamp=pose.get('stamp')
        if stamp is None or not np.isfinite(stamp) or not 0<=now-stamp<=.15:
            return None
        hand=np.asarray(pose['rotation'],float);anchor=np.asarray(held_command,float)
        for rotation in [hand,anchor]:
            if rotation.shape!=(3,3) or not np.isfinite(rotation).all() or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6) or np.linalg.det(rotation)<.999:
                raise ValueError('Invalid clutch rotation; no binding committed')
        if self.scale is not None:
            if point is None or held_position is None or position_stamp is None:
                return None
            if not np.isfinite(position_stamp) or not 0<=now-position_stamp<=.15:
                return None
            point=np.asarray(point,float);held_position=np.asarray(held_position,float)
            for value in [point,held_position]:
                if value.shape!=(3,) or not np.isfinite(value).all():
                    raise ValueError('Invalid clutch position; no binding committed')
            # Commit both references only after both streams pass validation.
            self.point=point.copy();self.position_anchor=held_position.copy()
        self.hand=hand.copy();self.anchor=anchor.copy();self.binding_id+=1
        binding=dict(binding_id=self.binding_id,hand_reference_rotation=self.hand.tolist(),
                    command_anchor_rotation=self.anchor.tolist(),orientation_source_stamp_s=stamp,
                    bound_at_s=now,source_age_ms=(now-stamp)*1000)
        if self.scale is not None:
            binding.update(hand_reference_position_m=self.point.tolist(),
                           command_anchor_position_m=self.position_anchor.tolist(),
                           position_source_stamp_s=position_stamp,
                           position_source_age_ms=(now-position_stamp)*1000)
        return binding

    def map_position(self, point):
        if self.point is None or point is None:return None
        return self.position_anchor+self.scale*(self.A@(np.asarray(point,float)-self.point))

    def map(self, hand):
        if self.hand is None or hand is None:return None
        hand=np.asarray(hand,float)
        # Exact held target on the binding sample, including after many resumes.
        if np.array_equal(hand,self.hand):return self.anchor.copy()
        return self.A@(hand@self.hand.T)@self.A.T@self.anchor
