"""Opt-in, default bare UR5e gravity compensation for offline task replay.

No live import. Original position servos and their force clamps remain intact;
an additional joint clamp bounds feedback + native gravity compensation by the
same original per-actuator limit. Exactly one unit-gear actuator per joint.
"""
import mujoco
import numpy as np
from .robot import ROOT, SimArm

FIELDS = ('body_gravcomp', 'jnt_actgravcomp', 'jnt_actfrclimited', 'jnt_actfrcrange')
BODIES = ('shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link')


def compiled_reference():
    if mujoco.__version__ != '3.12.0':
        raise RuntimeError('Requires pinned MuJoCo 3.12.0')
    scene=ROOT/'external/menagerie/universal_robots_ur5e/scene.xml'
    spec=mujoco.MjSpec.from_file(str(scene));original=spec.compile()
    if original.nq!=6 or original.nv!=6 or original.nu!=6:
        raise ValueError('Only the original bare six-joint UR5e is supported')
    if np.any(original.body_gravcomp) or np.any(original.jnt_actgravcomp):
        raise ValueError('Unexpected preexisting compensation; refusing to double it')
    if not np.array_equal(original.actuator_trnid[:,0],np.arange(6)) or not np.array_equal(original.actuator_gear,np.tile([1,0,0,0,0,0],(6,1))):
        raise ValueError('Requires original one-to-one unit joint transmission')
    if not np.all(original.actuator_forcelimited):
        raise ValueError('Original actuator force caps required')
    if not np.array_equal(original.actuator_forcerange,np.array([[-150,150]]*3+[[-28,28]]*3)):
        raise ValueError('Original UR5e caps changed')
    for body in spec.bodies:
        if body.name in BODIES:body.gravcomp=1.
    for joint in spec.joints:
        j=original.joint(joint.name).id
        joint.actgravcomp=True
        joint.actfrclimited=mujoco.mjtLimited.mjLIMITED_TRUE
        joint.actfrcrange=original.actuator_forcerange[j]
    return spec.compile()


def make_compensated_ur5e():
    """Configure the newly created, not-yet-stepped model; never reset a run.

    Retain all SimArm model-bound tasks and limits. Only copy the four physical
    fields and flag from a compiled specification, as allowed by MuJoCo's model
    changes contract. The model is shared by actual and prediction data.
    """
    arm=SimArm('ur5e');compiled=compiled_reference();m=arm.model
    for field in FIELDS:getattr(m,field)[:]=getattr(compiled,field)
    m.flg_gravcomp=compiled.flg_gravcomp
    if not m.flg_gravcomp or not np.all(m.jnt_actgravcomp) or not np.all(m.jnt_actfrclimited):
        raise RuntimeError('Incomplete native compensation configuration')
    mujoco.mj_forward(m,arm.data)
    arm.configuration.update(arm.data.qpos)
    return arm
