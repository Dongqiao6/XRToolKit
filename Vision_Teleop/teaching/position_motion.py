"""Acceleration-bounded position command for the recorded position-only replay."""
import numpy as np


class PositionMotion:
    def __init__(self, position, settings):
        self.position=np.asarray(position,float).copy()
        self.velocity=np.zeros(3)
        self.speed=float(settings['max_target_speed_m_s'])
        self.tau=float(settings['smoothing_time_s'])
        if not np.isfinite([self.speed,self.tau]).all() or min(self.speed,self.tau)<=0:
            raise ValueError('Positive speed and smoothing time required')
        # Derived from unchanged speed / smoothing time, not tuned to this video.
        self.acceleration=self.speed/self.tau

    def update(self, position, dt):
        if not np.isfinite(dt) or dt<=0:raise ValueError('Positive finite dt required')
        desired=np.zeros(3)
        if position is not None:
            error=np.asarray(position,float)-self.position
            if error.shape!=(3,) or not np.isfinite(error).all():raise ValueError('Finite position required')
            desired=error/self.tau
            desired*=min(1.,self.speed/max(np.linalg.norm(desired),1e-12))
        delta=desired-self.velocity
        self.velocity+=delta*min(1.,self.acceleration*dt/max(np.linalg.norm(delta),1e-12))
        self.position+=self.velocity*dt
        return self.position.copy()
