"""Explicit candidate-only position mapping and world-fixed clutch.

No body observations, personal limb lengths or comfort model are inferred here.
The original independent orientation clutch and command dynamics are retained.
"""
from pathlib import Path
import json
import numpy as np
from .orientation_clutch import OrientationClutch
from .diffeomorphic_position import PositionMap

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'results/position_mapping/20261009_gao_01/d140_tol04'
ORIGIN = np.array([.45, 0., 1.50])
UNIT = .1


def load_map(condition):
    if condition not in ('A', 'C'):
        raise ValueError('Only fixed A and frozen C are trial conditions')
    return PositionMap.restore(json.loads((SOURCE/f'{condition}.json').read_text())['map'])


class WorldFieldClutch(OrientationClutch):
    """Translate the hand reference in the domain of ONE fixed world field.

    u0 = f^-1((held_world - origin)/unit)
    p = origin + unit*f(u0 + A*(hand - new_hand_reference)/unit)
    This intentionally is not held_world + unit*f(hand_displacement/unit).
    """
    def __init__(self, world_from_camera, mapping, *, origin=ORIGIN, unit=UNIT):
        super().__init__(world_from_camera, position_scale=1.)
        self.mapping = mapping
        self.origin = np.asarray(origin, float).copy()
        self.unit = float(unit)
        if mapping.dimension != 3 or self.origin.shape != (3,) or not np.isfinite(self.origin).all() or not np.isfinite(unit) or unit <= 0:
            raise ValueError('Finite 3D field and positive length unit required')
        if self.A.shape != (3, 3) or not np.allclose(self.A.T@self.A, np.eye(3), atol=1e-6) or np.linalg.det(self.A) < .999:
            raise ValueError('Proper fixed world-from-camera rotation required')
        self.u_anchor = None
        self.last_rejection = None

    def bind(self, pose, held_command, now, *, point=None, position_stamp=None, held_position=None):
        # Compute and check inverse BEFORE the parent's atomic position/R commit.
        # Failure leaves all previous references intact and the request pending.
        if held_position is None:
            return None
        try:
            held = np.asarray(held_position, float)
            if held.shape != (3,) or not np.isfinite(held).all():
                raise ValueError('Invalid held world position')
            u = self.mapping.inverse((held-self.origin)/self.unit)
            residual = float(np.linalg.norm(self.origin+self.unit*self.mapping.forward(u)-held))
            if not np.isfinite(u).all() or residual > 1e-8:
                raise ValueError('Inverse failed its forward residual check')
        except (ValueError, RuntimeError, FloatingPointError) as exc:
            self.last_rejection = str(exc)
            return None
        binding = super().bind(pose, held_command, now, point=point,
                              position_stamp=position_stamp, held_position=held_position)
        if binding is None:
            self.last_rejection = 'Waiting for fresh position and complete orientation'
            return None
        self.u_anchor = u.copy()
        self.last_rejection = None
        binding.update(field_domain_anchor=u.tolist(), world_field_origin_m=self.origin.tolist(),
                       field_unit_m=self.unit, inverse_residual_m=residual,
                       physical_domain_recentered=True,
                       comfort_coverage='not inferred; re-evaluate confirmed hand samples after each binding')
        return binding

    def map_position(self, point):
        if point is None or self.point is None:
            return None
        point = np.asarray(point, float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError('Missing/invalid hand cannot be mapped')
        if np.array_equal(point, self.point):
            return self.position_anchor.copy()
        return self.origin+self.unit*self.mapping.forward(self.u_anchor+self.A@(point-self.point)/self.unit)

    def hand_for_world(self, world):
        """Inverse diagnostic / synthetic driver, never a fabricated observation."""
        if self.point is None:
            raise ValueError('Explicit binding required')
        u = self.mapping.inverse((np.asarray(world, float)-self.origin)/self.unit)
        return self.point+self.unit*(self.A.T@(u-self.u_anchor))


def trial_protocol():
    from .task_demo import GOAL_R
    return dict(schema=2, robot='fr3-default', task_condition='A', mapping_order=['A', 'C'],
        field_origin_m=ORIGIN.tolist(), field_unit_m=UNIT,
        goals=[dict(position_m=(ORIGIN+[x, 0, 0]).tolist(), rotation=GOAL_R.tolist()) for x in [.14, -.14]],
        fine_world_radius_m=.010,
        completion=dict(position_m=.020, full_rotation_deg=2., fresh_hold_s=.5),
        model_perturbation_m=.003, model_output_budget_m=.004,
        distinction='10mm world fine region, 20mm/2deg/0.5s completion and 4mm MODEL perturbation budget are different quantities',
        familiarization_s=20., formal_s=70., initialization_max_s=60.,
        advance='N records current target and advances even if criterion not met; never repeat until pass',
        finish='Both targets must meet unchanged completion rule and at least one explicit formal rebind for completed status; retain incomplete/timeout/exit',
        fixed_gain=1.4, gain_basis='Existing frozen 280mm model representative, selected before human trial; not personalized or claimed optimal',
        initialization='Comfortable start pose; Space requests an atomic fresh position/full-orientation binding. No mandatory range samples; personal range remains unmeasured.',
        input='Original left control-point position and independent full hand orientation; bottle only assists grip',
        settings='Original task command bounds/filter/freshness, gravity, actuators, collision and joint checks',
        comparison='Descriptive A then C, equal familiarization; order/learning confounded; no comfort or accuracy claim from model inputs')


def orientation_guidance(actual, target):
    """Display-only swing, then signed twist that exactly gives target rotation.

    Uses ROBOT actual/target geometry, not inferred human correction. At opposite
    tool axes the swing direction is nonunique: do not invent a turn direction.
    """
    from scipy.spatial.transform import Rotation
    actual, target = np.asarray(actual, float), np.asarray(target, float)
    for r in [actual, target]:
        if r.shape != (3, 3) or not np.isfinite(r).all() or not np.allclose(r.T@r, np.eye(3), atol=1e-6) or np.linalg.det(r) < .999:
            raise ValueError('Proper actual and target rotations required')
    a, b = actual[:, 2], target[:, 2]
    cross = np.cross(a, b); sine = float(np.linalg.norm(cross)); cosine = float(np.clip(a@b, -1, 1))
    angle = float(np.arctan2(sine, cosine))
    if cosine < -1+1e-8:
        return dict(defined=False, reason='opposite_axes', tilt_deg=180., twist_deg=None)
    vector = cross/sine*angle if sine > 1e-12 else np.zeros(3)
    front = Rotation.from_rotvec(vector).apply(actual[:, 0])
    twist = float(np.arctan2(b@np.cross(front, target[:, 0]), front@target[:, 0]))
    return dict(defined=True, tilt_deg=float(np.degrees(angle)), twist_deg=float(np.degrees(twist)),
                swing_rotvec_world=vector.tolist(), aligned_front_world=front.tolist(), axis_world=b.tolist())


class TrialSession:
    """Existing session hook; never pauses model integration during initialization."""
    def __init__(self, folder, condition):
        self.folder = Path(folder); self.condition = condition; self.protocol = trial_protocol()
        self.stage = 'ready'; self.epoch = None; self.start = None; self.formal_start = None
        self.now = 0.; self.latest = None; self.confirmed = []; self.events = []; self.rows = []
        self.bindings = []; self.clutch = None; self.target_index = 0
        self.stable_since = None; self.met = False; self.targets = []; self.end = None
        self.shared = None; self.note = ''; self.formal_bindings = 0
        self.target_started_at = None; self.first_met_at = None; self.hold_s = 0.
        self.last_task_clock = None; self.n_armed = True; self.last_n_clock = None
        self.display_rows = []; self.display_clock = None; self.finalized = False

    @property
    def execution_enabled(self):
        # Waiting is part of the physical time axis; gate user start, not stepping.
        return True

    def observe(self, frames, pose, point, now, index):
        self.now = now
        if self.epoch is None: self.epoch = now
        left = frames.get('left')
        stamp = left.time_ms/1000 if left is not None else None
        valid = bool(point is not None and stamp is not None and 0 <= now-stamp <= .15
                     and pose and pose.get('valid') and pose.get('stamp') is not None and 0 <= now-pose['stamp'] <= .15)
        self.latest = dict(index=index, now=now, point=None if point is None else np.asarray(point).tolist(),
                           position_stamp=stamp, pose_stamp=pose.get('stamp') if pose else None,
                           position_frame_id=getattr(left, 'identity', None), ready=valid)

    def bound(self, binding):
        self.bindings.append(binding)
        if self.start is None:
            self.start = binding['bound_at_s']; self.stage = 'familiarize'
        elif self.stage == 'formal':
            self.formal_bindings += 1
        # Re-evaluate measured sample locations under the NEW domain shift.
        binding['confirmed_samples_world_m'] = [self.clutch.map_position(x['point']).tolist() for x in self.confirmed]
        binding['required_hand_positions_camera_m'] = [self.clutch.hand_for_world(g['position_m']).tolist() for g in self.protocol['goals']]
        binding['personal_volume_coverage'] = 'unknown: personal input range is not measured by this task entry'

    def key(self, key, shared, events, now):
        self.shared = shared
        if self.end:
            return key == 32
        is_n = key in (ord('n'), ord('N'))
        if not is_n and self.last_n_clock is not None and now-self.last_n_clock >= .3:
            self.n_armed = True
        if key in (ord('n'), ord('N')) and self.stage == 'formal':
            armed = self.n_armed; self.last_n_clock = now; self.n_armed = False
            if not armed: return True  # One held key must not submit two goals.
            self.close_attempt(now, 'operator_advance')
            if self.target_index == len(self.protocol['goals'])-1:
                self.end = 'completed' if all(t['criterion_met'] for t in self.targets) and self.formal_bindings else 'user_finished_incomplete'
            else:
                self.target_index += 1; self.stable_since = None; self.met = False; self.first_met_at = None; self.hold_s = 0.
                self.target_started_at = now
                self.note = '请做一次空格暂停—回舒适位置—空格继续，再对准新目标'
            return True
        return False

    def close_attempt(self, now, ending):
        if any(t['index'] == self.target_index for t in self.targets): return
        row = self.rows[-1] if self.rows and self.rows[-1]['stage'] == 'formal' and self.rows[-1]['target_index'] == self.target_index else None
        self.targets.append(dict(index=self.target_index, started_at=self.target_started_at, finished_at=now,
            criterion_met=self.met, first_met_at=self.first_met_at,
            completion_time_s=None if self.first_met_at is None else self.first_met_at-self.target_started_at,
            attempt_duration_s=None if self.target_started_at is None else now-self.target_started_at,
            ending=ending, last=row))

    def tick(self, now):
        self.now = now
        if self.epoch is None: self.epoch = now
        if self.shared and self.shared.get('robot_error'):
            self.end = 'execution_protection'
        if self.start is None and now-self.epoch >= self.protocol['initialization_max_s']:
            self.end = 'initialization_timeout'
        if self.start is not None and self.formal_start is None and now-self.start >= self.protocol['familiarization_s']:
            self.stage = 'formal'; self.formal_start = now; self.stable_since = None; self.met = False
            self.target_started_at = now
            self.events.append(dict(type='formal_begin', clock_s=now))
        if self.formal_start is not None and now-self.formal_start >= self.protocol['formal_s']:
            self.end = self.end or 'task_timeout'

    def expired(self, now):
        return self.end is not None

    def goal(self):
        from .task_demo import START_R
        return self.protocol['goals'][self.target_index] if self.stage == 'formal' else dict(position_m=ORIGIN.tolist(), rotation=START_R.tolist())

    def task_row(self, actual, command, sample, condition):
        from .task_demo import pose_errors
        g = self.goal(); crit = self.protocol['completion']
        err = pose_errors(actual['actual_p'], actual['actual_r'], g['position_m'], g['rotation'], 'A')
        fresh = bool(sample['active'] and sample['position_fresh'] and sample['pose_fresh'])
        inside = bool(err['position_mm'] <= crit['position_m']*1000 and err['full_pose_deg'] <= crit['full_rotation_deg'])
        if self.last_task_clock is not None and self.now-self.last_task_clock > .15:
            self.stable_since = None  # No interpolation of unobserved long gaps.
        self.last_task_clock = self.now
        if self.stage == 'formal' and fresh and inside:
            if self.stable_since is None: self.stable_since = self.now
            self.hold_s = self.now-self.stable_since
            if self.hold_s+1e-12 >= crit['fresh_hold_s']:
                if not self.met: self.first_met_at = self.now
                self.met = True
        else: self.stable_since = None; self.hold_s = 0.
        def error(p, r, target_p, target_r):
            e = pose_errors(p, r, target_p if target_p is not None else ORIGIN,
                            target_r if target_r is not None else np.eye(3), 'A')
            if target_p is None: e['position_mm'] = None
            if target_r is None:
                for key in ['axis_deg', 'full_pose_deg', 'roll_diagnostic_deg']: e[key] = None
            return e
        rp = sample.get('raw_p') if sample['position_fresh'] else None
        rr = sample.get('raw_r') if sample['pose_fresh'] else None
        row = dict(clock_s=self.now, stage=self.stage, target_index=self.target_index, world_goal=g,
            active=sample['active'], position_fresh=sample['position_fresh'], pose_fresh=sample['pose_fresh'],
            final_reference=err, inside_diagnostic_reference=inside, fresh_hold_evidence=fresh and inside,
            criterion_met=self.met, inside_fine_world_region=err['position_mm'] <= self.protocol['fine_world_radius_m']*1000,
            fresh_hold_s=self.hold_s, first_met_at=self.first_met_at,
            guidance=orientation_guidance(actual['actual_r'], g['rotation']),
            raw_to_actual=error(actual['actual_p'], actual['actual_r'], rp, rr),
            requirement_to_command=error(command['command_p'], command['command_r'], rp, rr),
            command_to_actual=error(actual['actual_p'], actual['actual_r'], command['command_p'], command['command_r']),
            requirement_to_goal=error(g['position_m'], g['rotation'], rp, rr))
        self.rows.append(row)
        return row

    def publish(self, now, frames, composite):
        pass  # Decorator stores the exact displayed task status; raw/video remain original.

    def finish(self):
        if self.finalized: return
        self.finalized = True
        self.folder.mkdir(parents=True, exist_ok=True)
        if self.stage == 'formal': self.close_attempt(self.now, self.end or 'user_exit_or_execution_stop')
        summary = dict(condition=self.condition, protocol=self.protocol, status=self.end or 'user_exit_or_execution_stop',
            confirmed_samples=self.confirmed, bindings=self.bindings, formal_rebindings=self.formal_bindings,
            target_results=self.targets, unfinished_target_indices=list(range(len(self.targets), len(self.protocol['goals']))),
            unattempted_target_indices=[i for i in range(len(self.protocol['goals'])) if not any(t['index']==i for t in self.targets)],
            unsatisfied_target_indices=[i for i in range(len(self.protocol['goals'])) if not any(t['index']==i and t['criterion_met'] for t in self.targets)],
            task_targets_completed=sum(t['criterion_met'] for t in self.targets),
            task_complete=len(self.targets)==len(self.protocol['goals']) and all(t['criterion_met'] for t in self.targets),
            clutch_protocol_complete=self.formal_bindings>0,
            personal_range_measured=False,
            total_source_s=None if self.epoch is None else self.now-self.epoch,
            engaged_source_s=None if self.start is None else self.now-self.start,
            note='Preparation, protection and end reason also retained in original runner logs; no unobserved body data inferred')
        if len(self.confirmed) == 3:
            summary['confirmed_endpoint_distance_m'] = float(np.linalg.norm(np.array(self.confirmed[2]['point'])-self.confirmed[1]['point']))
            summary['confirmed_range_scope'] = 'Operator-confirmed point samples only; neither fixed limb length nor certified 3D comfort volume'
        (self.folder/'mapping_trial.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf8')
        for name, rows in [('mapping_task', self.rows), ('mapping_events', self.events), ('mapping_display', self.display_rows)]:
            (self.folder/f'{name}.jsonl').write_text(''.join(json.dumps(x, ensure_ascii=False)+'\n' for x in rows), encoding='utf8')
