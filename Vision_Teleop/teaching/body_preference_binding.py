"""Offline optional body preference attached to committed primary clutch events.

Human reference is common across robots; each robot binds only its own model
reference. No event is inferred from tracking recovery or a comfort C-key mark.
The primary command passes through unchanged. This module is not imported live.
"""
from copy import deepcopy
from dataclasses import dataclass, asdict
import numpy as np
from .human_geometry import Limits
from .relative_swivel import bind_swivel, relative_swivel
from .configuration_preference import plane_geometry, ROBOT_POINTS
from .robot_swivel import RobotSwivelTask


@dataclass(frozen=True)
class BodyPacket:
    session_id: str
    available_at_s: float  # Availability in the SAME clock, not just capture time.
    points: dict


class BodyPreferenceBinding:
    def __init__(self, session_id, frame, clock='host_perf_counter', limits=Limits()):
        if not session_id or not frame or not clock:
            raise ValueError('Explicit session, coordinate frame and clock required')
        self.session_id, self.frame, self.clock = session_id, frame, clock
        self.limits = limits
        self.anchor = None
        self.binding_id = None
        self.segment_id = 0
        self.last_time = -np.inf
        self.state = 'waiting_for_committed_binding'

    def _time(self, stamp):
        if not np.isfinite(stamp) or stamp < self.last_time:
            raise ValueError('Nonfinite or out-of-order event/read time')

    def _packet_reason(self, packet, at_s):
        if packet is None: return 'body_packet_missing'
        if packet.session_id != self.session_id: return 'body_session_mismatch'
        if not np.isfinite(packet.available_at_s): return 'body_availability_unknown'
        if packet.available_at_s > at_s: return 'body_not_yet_available'
        if any(np.isfinite(p.stamp_s) and p.stamp_s > packet.available_at_s
               for p in packet.points.values()): return 'availability_before_capture'
        return None

    def event(self, event, packet=None):
        """Accept raw committed pose_clutch_bound / user_pause events only.

        Dispatch at their recorded event time, before the next control read.
        A missing body at a committed start/resume disables it for this segment;
        hand control proceeds independently, without waiting for body recovery.
        """
        kind = event.get('type')
        if kind not in {'pose_clutch_bound', 'user_pause'}:
            raise ValueError('Not a committed primary binding or pause event')
        now = event['bound_at_s'] if kind == 'pose_clutch_bound' else event['clock_s']
        self._time(now)
        if kind == 'pose_clutch_bound':
            segment = event['binding_id']
            if (not isinstance(segment, int) or isinstance(segment, bool)
                    or segment <= self.segment_id or event.get('cause') not in {'start','resume'}):
                raise ValueError('Unique increasing committed binding and explicit cause required')
        self.last_time = now
        self.anchor = None
        if kind == 'user_pause':
            self.state = 'paused'
            return dict(type=kind, stamp_s=now, body_bound=False, reason='paused', binding_id=self.binding_id)
        self.segment_id = segment
        self.binding_id = f'{self.session_id}:primary_binding:{segment}'
        reason = self._packet_reason(packet, now)
        if reason is None:
            try:
                self.anchor = bind_swivel(packet.points, frame=self.frame, at_s=now,
                    clock=self.clock, limits=self.limits, reference_id=self.binding_id)
            except ValueError as error:
                reason = str(error)
        self.state = 'bound' if self.anchor is not None else 'body_unbound_for_segment'
        return dict(type=kind, stamp_s=now, body_bound=self.anchor is not None,
            reason=reason or 'explicit_body_reference_bound', binding_id=self.binding_id,
            primary_binding_id=segment, session_id=self.session_id,
            anchor=asdict(self.anchor) if self.anchor else None)

    def augment(self, primary_command, packet, *, at_s):
        """Pass the saved common command through; only attach optional metadata."""
        self._time(at_s); self.last_time = at_s
        reason = None
        if not primary_command.get('engaged', False): reason = 'primary_not_engaged'
        elif self.anchor is None: reason = self.state
        else: reason = self._packet_reason(packet, at_s)
        request = dict(valid=False, angle_rad=None, reason=reason, binding_id=self.binding_id,
            at_s=at_s, session_id=self.session_id, permitted_to_modify_primary_task=False)
        if reason is None:
            request.update(relative_swivel(packet.points, self.anchor, frame=self.frame,
                at_s=at_s, clock=self.clock, limits=self.limits))
        return dict(primary_command=deepcopy(primary_command), body_request=request)


class RobotPreferenceBinding:
    """Bind the robot reference q at the SAME logical committed event.

    Robot state chooses its own geometric reference, never a different human
    request or primary command. This does not synchronize physical exposure or
    promise that an executed robot matches its reference q.
    """
    def __init__(self, robot_name):
        self.joint_names = ROBOT_POINTS[robot_name]
        self.reference = None
        self.last_event_s = -np.inf
        self.seen_bindings = set()

    def event(self, common_binding, configuration, *, reference_stamp_s):
        stamp = common_binding['stamp_s']
        if (not np.isfinite(reference_stamp_s) or reference_stamp_s != stamp
                or stamp < self.last_event_s):
            raise ValueError('Robot reference must belong to this logical binding event')
        token = common_binding.get('binding_id')
        if common_binding['type'] == 'pose_clutch_bound':
            if token in self.seen_bindings:
                raise ValueError('Robot reference cannot rebind the same primary event')
            self.seen_bindings.add(token)
        self.last_event_s = stamp
        self.reference = None
        if not common_binding['body_bound']:
            return dict(valid=False, reason=common_binding['reason'])
        try:
            g = plane_geometry(configuration, self.joint_names)
            axis = g['points'][2]-g['points'][0]; length = np.linalg.norm(axis)
            if length < 1e-6: raise ValueError('zero_shoulder_wrist_distance')
        except ValueError as error:
            return dict(valid=False, reason=str(error))
        self.reference = dict(binding_id=common_binding['binding_id'], stamp_s=stamp,
            axis=(axis/length).tolist(), normal=g['normal'].tolist(),
            reference_q=configuration.q.tolist(), model_dof=configuration.model.nv,
            joint_names=list(self.joint_names))
        return dict(valid=True, reason='robot_reference_bound', **deepcopy(self.reference))

    def task(self, request, configuration):
        if not request['valid']: return None, dict(reason=request['reason'])
        if self.reference is None: return None, dict(reason='robot_reference_missing')
        if request['binding_id'] != self.reference['binding_id']:
            return None, dict(reason='binding_mismatch')
        if request['at_s'] < self.reference['stamp_s']:
            return None, dict(reason='request_before_robot_reference')
        r = self.reference
        task = RobotSwivelTask(self.joint_names, r['axis'], r['normal'], request['angle_rad'],
            binding_id=r['binding_id'])  # Existing fixed .001 mechanism cost, no tuning.
        try:
            task.geometry(configuration)
        except ValueError as error:
            return None, dict(reason='robot_geometry_invalid:'+str(error))
        return task, dict(reason='optional_body_task', binding_id=r['binding_id'],
                          angle_rad=request['angle_rad'])
