"""Fixed approved rod/column assembly and task-specific collision coverage."""
import itertools
import xml.etree.ElementTree as ET

import mink
import mujoco
import numpy as np


def compile_wrist2_candidate(spec):
    """Opt-in UR5e task model: compile the audited upstream mesh hulls.

    Keep the original capsule sizes/assets for attribution, but deactivate their
    collision flags. Enabling meshes before compile builds the contact BVH.
    Explicit inertials, flange/TCP, all other geoms and thresholds stay intact.
    """
    body = spec.body('wrist_2_link')
    if body is None:
        raise ValueError('Wrist2 candidate requires the UR5e task model')
    meshes = [g for g in body.geoms if g.type == mujoco.mjtGeom.mjGEOM_MESH]
    capsules = [g for g in body.geoms if g.type == mujoco.mjtGeom.mjGEOM_CAPSULE]
    if len(meshes) != 3 or len(capsules) != 2:
        raise ValueError('Upstream UR5e wrist2 geometry layout changed')
    for g in capsules:
        g.contype = 0
        g.conaffinity = 0
    for g in meshes:
        g.contype = 1
        g.conaffinity = 1


def add_held_rod(hand, task):
    tool=task['tool']; offset=tool['tcp_flange_offset_m']
    body=ET.SubElement(hand,'body',name='held_rod',pos=f'0 0 {offset}')
    center=(tool['back_m']+tool['front_m'])/2
    half=(tool['front_m']-tool['back_m'])/2
    ET.SubElement(body,'geom',name='rod',type='cylinder',pos=f'0 0 {center}',
                  size=f"{tool['radius_m']} {half}",mass=str(tool['mass_kg']),
                  contype='1',conaffinity='1',rgba='.9 .55 .12 1')
    ET.SubElement(body,'site',name='rod_tip',pos=f"0 0 {tool['front_m']}",size='.003',rgba='1 0 0 1')
    for sign in (-1,1):
        block=ET.SubElement(hand,'body',name=f'insert_{sign}',
                            pos=f"0 {sign*tool['insert_center_abs_y_m']} {offset+tool['insert_center_z_from_tcp_m']}")
        ET.SubElement(block,'geom',name=f'insert_{sign}',type='box',
                      size=' '.join(map(str,tool['insert_half_size_m'])),
                      mass=str(tool['insert_mass_kg']),contype='1',conaffinity='1',rgba='.25 .25 .3 1')


def add_task_scene(spec, task):
    lo=np.array(task['obstacle_min_m']);hi=np.array(task['obstacle_max_m'])
    spec.worldbody.add_geom(name='rod_task_column',type=mujoco.mjtGeom.mjGEOM_BOX,
        pos=(lo+hi)/2,size=(hi-lo)/2,rgba=[.7,.18,.16,1],contype=1,conaffinity=1)
    for name,lo,hi,color in [
        ('gate',task['gate_min_m'],task['gate_max_m'],[.2,.8,.2,.12]),
        ('turn',task['turn_min_m'],task['turn_max_m'],[.2,.4,1,.12]),
        ('start',np.array(task['start_m'])-task['start_half_extent_m'],
         np.array(task['start_m'])+task['start_half_extent_m'],[1,.8,.1,.12])]:
        lo,hi=np.asarray(lo),np.asarray(hi)
        spec.worldbody.add_geom(name='rod_task_'+name,type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=(lo+hi)/2,size=(hi-lo)/2,rgba=color,contype=0,conaffinity=0)
    goal=np.asarray(task['goal_tip_m']);xlo,xhi=task['corridor_x_m']
    spec.worldbody.add_geom(name='rod_task_goal',type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[(xlo+xhi)/2,goal[1],goal[2]],quat=[2**-.5,0,2**-.5,0],
        size=[task['corridor_radius_m'],(xhi-xlo)/2,0],rgba=[.15,.8,.9,.14],contype=0,conaffinity=0)
    spec.worldbody.add_site(name='rod_task_tip_target',pos=goal,size=[.005,0,0],rgba=[0,1,1,1])


class ExplicitPairs(mink.CollisionAvoidanceLimit):
    """Explicit audited pairs bypass same-weld/parent-child environment omissions."""
    def _construct_geom_id_pairs(self,pairs):
        return sorted(set(tuple(sorted((int(a),int(b)))) for left,right in pairs
                          for a,b in itertools.product(left,right) if a!=b))

    def compute_qp_inequalities(self, configuration, dt):
        # Pinned Mink 1.3.0 (ab45779): collision limit returns a velocity bound
        # gain*(distance-minimum)/dt, but build_ik optimizes displacement dq.
        # Convert that bound to displacement, like the upstream VelocityLimit.
        # Keep this adapter task-local; do not edit the pinned dependency or
        # silently rewrite the preserved fixed/follow baseline behavior.
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('Positive finite collision timestep required')
        constraint=super().compute_qp_inequalities(configuration,dt)
        if constraint.inactive:return constraint
        from mink.limits import Constraint
        return Constraint(G=constraint.G,h=constraint.h*dt)


class RodSafetyError(RuntimeError):
    pass


class RodSafety:
    def __init__(self,arm,task):
        self.arm=arm;self.model=arm.model;self.task=task
        model=self.model
        physical=[i for i in range(model.ngeom) if model.geom_contype[i] or model.geom_conaffinity[i]]
        names={i:model.geom(i).name or f'geom_{i}' for i in physical}
        self.names=names
        column=model.geom('rod_task_column').id;floor=model.geom('floor').id
        tool=[i for i in physical if names[i].startswith('tool_') or
              model.body(int(model.geom_bodyid[i])).name.startswith('tool_')]
        robot=[i for i in physical if i not in tool+[column,floor]]
        pairs=set(arm.limits[-1].geom_id_pairs)
        mandatory=set()
        for env in (column,floor):
            for moving in tool+robot:
                # A base mounted in free space is not a designed contact with this column/floor.
                mandatory.add(tuple(sorted((env,moving))))
        mount_body=int(model.site('attachment_site').bodyid.item())
        added_tool=[model.geom(n).id for n in ('tool_rod','tool_insert_-1','tool_insert_1')]
        for a in tool:
            for b in robot:
                # Existing palm/flange mechanical mounting is intentional. Long rod
                # and new inserts still get checked against the terminal link itself.
                if int(model.geom_bodyid[b])==mount_body and a not in added_tool:
                    continue
                mandatory.add(tuple(sorted((a,b))))
        pairs.update(mandatory)
        self.pairs=sorted(pairs)
        self.pair_array=np.asarray(self.pairs,dtype=int)
        self.radii=model.geom_rbound[self.pair_array]
        self.limit=ExplicitPairs(model,[([a],[b]) for a,b in self.pairs],
            minimum_distance_from_collisions=task['criteria']['clearance_m'],
            collision_detection_distance=task['criteria']['detection_distance_m'])
        self.scratch=mujoco.MjData(model)
        self.trial=mujoco.MjData(model)
        self.audit=dict(pair_count=len(pairs),mandatory_pair_count=len(mandatory),
            mandatory_pairs=[[names[a],names[b]] for a,b in sorted(mandatory)],
            fixed_assembly_exclusion='Rigid rod/inserts/hand internal assembly; existing palm-flange mounting only. Rod/inserts vs terminal link are checked.',
            check='Tentative physics substeps plus interpolated joint midpoints; commit only if all pass. Discrete sweep samples, not a continuous-time proof.',
            distance_query_cap_m=.03)
        self.begin_step()

    def begin_step(self):
        self.last=dict(minimum_distance_m=.03,nearest_pair=None,joint_margin_rad=None,
                       max_actual_joint_speed_rad_s=0.,samples=0,violation=None)

    def inspect(self,data):
        cap=self.task['criteria']['detection_distance_m']
        distances=[]
        # Same conservative bounding spheres as the underlying limit; planes remain.
        separation=np.linalg.norm(data.geom_xpos[self.pair_array[:,0]]-data.geom_xpos[self.pair_array[:,1]],axis=1)
        keep=(self.radii<=0).any(axis=1) | (separation<=self.radii.sum(axis=1)+cap)
        for a,b in self.pair_array[keep]:
            d=float(mujoco.mj_geomDistance(self.model,data,a,b,cap,None))
            distances.append((d,a,b))
        nearest=min(distances,default=(cap,None,None))
        ids=self.model.actuator_trnid[:,0];q=data.qpos[self.arm.q_indices]
        ranges=self.model.jnt_range[ids]
        margin=float(np.min(np.minimum(q-ranges[:,0],ranges[:,1]-q)))
        return dict(minimum_distance_m=nearest[0],nearest_pair=None if nearest[1] is None else
                    [self.names[nearest[1]],self.names[nearest[2]]],joint_margin_rad=margin)

    def check_initial(self):
        row=self.inspect(self.arm.data)
        if row['minimum_distance_m']<self.task['criteria']['clearance_m']-1e-9 or row['joint_margin_rad']<-1e-9:
            self.last.update(row,violation='initial_configuration')
            raise RodSafetyError(f'Initial assembly/configuration unsafe; no simulation step: {row}')

    def check_substep(self,old_q,data=None):
        data=self.arm.data if data is None else data
        self.last['max_actual_joint_speed_rad_s']=max(self.last['max_actual_joint_speed_rad_s'],
            float(np.max(np.abs(data.qvel))))
        self.scratch.qpos[:]=(old_q+data.qpos)/2
        mujoco.mj_forward(self.model,self.scratch)
        for sample in (self.scratch,data):
            row=self.inspect(sample);self.last['samples']+=1
            if row['minimum_distance_m']<self.last['minimum_distance_m']:
                self.last.update(minimum_distance_m=row['minimum_distance_m'],nearest_pair=row['nearest_pair'])
            prior=self.last['joint_margin_rad']
            self.last['joint_margin_rad']=min(prior if prior is not None else float('inf'),row['joint_margin_rad'])
            if row['minimum_distance_m']<self.task['criteria']['clearance_m']-1e-9:
                self.last['violation']='clearance';raise RodSafetyError(f'Actual clearance violation: {self.last}')
            if row['joint_margin_rad'] < -1e-9:
                self.last['violation']='joint_position';raise RodSafetyError(f'Actual joint range violation: {self.last}')

    def commit_checked_step(self, command_q, dt):
        """Simulation-only transaction: same current state/command, no lookahead input.

        The physics trial uses a full data copy (including integrator/solver state).
        A rejected step leaves the live simulated state and time untouched. This
        is a stop/rejection, not a braking policy for a physical robot.
        """
        self.begin_step()
        mujoco.mj_copyData(self.trial,self.model,self.arm.data)
        self.trial.ctrl[:]=command_q[self.arm.q_indices]
        try:
            for _ in range(round(dt/self.model.opt.timestep)):
                old=self.trial.qpos.copy()
                mujoco.mj_step(self.model,self.trial)
                self.check_substep(old,self.trial)
        except RodSafetyError as exc:
            rejected=dict(self.last)
            actual=self.inspect(self.arm.data)
            self.last=dict(**actual,max_actual_joint_speed_rad_s=float(np.max(np.abs(self.arm.data.qvel))),
                samples=0,violation='predicted_'+str(rejected['violation']),rejected_step=rejected,
                committed=False)
            raise RodSafetyError(f'Simulation step rejected before commit; state unchanged: {self.last}') from exc
        mujoco.mj_copyData(self.arm.data,self.model,self.trial)
        self.last['committed']=True
