"""Three explicit task configurations, sharing the existing input and task loop."""
import json
import mujoco
import numpy as np
from .robot import ROOT,SimArm
from .task_control import compensated_fr3_model
from .ur5e_gravity import make_compensated_ur5e,FIELDS

CONFIG=ROOT/'config/task_profiles.json'


def profile(name):
    config=json.loads(CONFIG.read_text(encoding='utf8'))
    if config['schema_version']!=1 or config['condition']!='A' or config['base_position_world_m']!=[0,0,1] or config['tool_transform']!='model_attachment_site_identity_extra_transform':
        raise ValueError('Unsupported task configuration; no implicit transform changes')
    p=config['profiles'][name].copy()
    allowed={'fr3-default':('franka_fr3',0,'recorded_fr3_native_bounded'),
             'ur5e-default':('ur5e',0,'verified_ur5e_native_bounded'),
             'fr3-y45':('franka_fr3',45,'fr3_y45_native_bounded')}
    if (p['robot'],p['mount_y_degrees'],p['gravity'])!=allowed[name]:
        raise ValueError('Configuration outside explicitly qualified robot/mount combinations')
    return p


def status(name):
    p=profile(name);path=ROOT/p['evidence']
    evidence=json.loads(path.read_text(encoding='utf8')) if path.exists() else None
    blocked=evidence is not None and evidence['status']!='input_completed'
    report=path.parent.parent/'report.json'
    limits=None
    if report.exists() and evidence is not None:
        data=json.loads(report.read_text(encoding='utf8'))
        matching=next((r for r in data.get('conditions',[]) if r['robot']==evidence['robot']),None)
        if matching is not None:limits=matching.get('safety')
    return dict(profile=name,**p,evidence_status=None if evidence is None else evidence['status'],
        live_allowed=evidence is not None and not blocked,
        known_execution_limits=limits,
        known_blocker=None if not blocked else dict(phase=evidence.get('phase'),error=evidence.get('error'),failed_source_row=evidence.get('failed_source_row'),unexecuted_steps=evidence.get('unexecuted_steps')),
        notice='未进行本配置的新现场体验；完整离线执行不等于对准通过' if name!='fr3-default' else '沿用已认可FR3输入与控制链路')


def make_arm(name):
    p=profile(name)
    if name=='fr3-default':return SimArm('franka_fr3',gravity_compensation=True)
    if name=='ur5e-default':return make_compensated_ur5e()
    # Keep SimArm's default-only guard. This bounded factory qualifies only the
    # bare original FR3 at world-Y45, never arbitrary robots or mounted tools.
    arm=SimArm(p['robot'],mount_degrees=p['mount_y_degrees'])
    reference=compensated_fr3_model(ROOT/'external/menagerie/franka_fr3/scene.xml')
    m=arm.model
    if (m.nq,m.nv,m.nu)!=(7,7,7) or np.any(m.body_gravcomp) or np.any(m.jnt_actgravcomp):
        raise ValueError('Unexpected FR3 layout or preexisting compensation')
    if not np.array_equal(m.jnt_actfrcrange,np.array([[-87,87]]*4+[[-12,12]]*3)):
        raise ValueError('Original FR3 total torque limits required')
    for field in FIELDS:getattr(m,field)[:]=getattr(reference,field)
    m.flg_gravcomp=reference.flg_gravcomp
    # Gravity stays [0,0,-9.81] in world coordinates. MuJoCo computes loads from
    # this model's already tilted base and current joint state, not default loads.
    mujoco.mj_forward(m,arm.data)
    arm.configuration.update(arm.data.qpos)
    return arm
