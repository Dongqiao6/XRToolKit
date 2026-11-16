"""Fixed point/direction/front teaching task; visual markers have no collision shape."""
from pathlib import Path
import json,time
import mujoco,numpy as np
from scipy.spatial.transform import Rotation
from .replay import SmoothTarget
from .native_prediction import attach_native_governor

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/task_teaching/20260917_requirements_01'
START=np.array([.45,0.,1.50]);START_R=np.diag([1.,-1.,-1.])
GOAL=np.array([.42,.10,1.48])
GOAL_R=Rotation.from_euler('y',20,degrees=True).as_matrix()@START_R@Rotation.from_euler('z',30,degrees=True).as_matrix()
SETTINGS=dict(max_target_speed_m_s=.1,max_target_angular_speed_rad_s=.5,smoothing_time_s=.5)

def axis_error(a,b):
    u=np.asarray(a)[:,2];v=np.asarray(b)[:,2]
    return float(np.degrees(np.arctan2(np.linalg.norm(np.cross(u,v)),np.dot(u,v))))

def pose_errors(p,r,target_p,target_r,condition):
    from scripts.run_basic_comparison import angle
    r=np.asarray(r);target_r=np.asarray(target_r)
    # Swing first aligns target working axis to actual, then signed twist about
    # that axis measures residual front rotation. Undefined at the antipode.
    a=target_r[:,2];b=r[:,2];cross=np.cross(a,b);s=float(np.linalg.norm(cross));dot=float(np.dot(a,b))
    twist=None
    if dot>-1+1e-8:
        aligned=target_r[:,0] if s<1e-12 else Rotation.from_rotvec(cross/s*np.arctan2(s,dot)).apply(target_r[:,0])
        twist=float(np.degrees(np.arctan2(np.dot(b,np.cross(aligned,r[:,0])),np.dot(aligned,r[:,0]))))
    return dict(position_mm=float(np.linalg.norm(np.asarray(p)-target_p)*1000),axis_deg=axis_error(r,target_r),
        full_pose_deg=angle(r,target_r) if condition=='A' else None,roll_diagnostic_deg=twist,
        roll_is_task_requirement=condition=='A')

def task_row(actual,command,sample,condition):
    raw_p=sample.get('raw_p');raw_r=sample.get('raw_r')
    measured=pose_errors(actual['actual_p'],actual['actual_r'],raw_p if raw_p is not None else START,raw_r if raw_r is not None else START_R,condition)
    if not sample.get('position_fresh'):measured['position_mm']=None
    if not sample.get('pose_fresh'):
        for k in ['axis_deg','full_pose_deg','roll_diagnostic_deg']:measured[k]=None
    goal=pose_errors(actual['actual_p'],actual['actual_r'],GOAL,GOAL_R,condition)
    entered=goal['position_mm']<=20 and goal['axis_deg']<=2 and (condition=='B' or goal['full_pose_deg']<=2)
    return dict(condition=condition,raw_to_actual=measured,
        command_to_actual=pose_errors(actual['actual_p'],actual['actual_r'],command['command_p'],command['command_r'],condition),
        final_reference=goal,inside_diagnostic_reference=bool(entered),
        fresh_hold_evidence=bool(entered and sample.get('position_fresh') and sample.get('pose_fresh')))

def prepare_task(arm,folder,condition):
    """Same nominal home and bounded preparation rule across A/B; no teleport."""
    from scripts.run_basic_comparison import dump
    from scripts.run_basic_comparison_v2 import execution_row
    from scripts.compare_angular_rate import capture_failure
    if condition not in ['A','B']:raise ValueError('Select A or B before running')
    folder=Path(folder);guard=attach_native_governor(arm)
    # Preparation uses the existing weighted pose/axis tasks for both conditions.
    # Main teaching then uses the original local position-preserving second QP.
    arm.position_priority=False;arm.axis_only=condition=='B';arm.target_axis_only=condition=='B'
    p,r=arm.pose();target=SmoothTarget(p,r,SETTINGS);stable=0.;started=time.perf_counter();i=None
    mujoco.mj_saveModel(arm.model,str(folder/'model.mjb'),None)
    dump(folder/'collision_pairs.json',guard.audit)
    dump(folder/'home.json',dict(q=arm.data.qpos.tolist(),p=p.tolist(),r=r.tolist()))
    try:
        guard.check_initial()
        with (folder/'preparation.jsonl').open('w') as f:
            for i in range(3000):
                if time.perf_counter()-started>120:raise TimeoutError('Fixed preparation120s wall budget')
                target.update(START,START_R,.01);arm.set_target(target.position,target.rotation);arm.step(.01)
                row=execution_row(arm);err=pose_errors(row['actual_p'],row['actual_r'],START,START_R,condition)
                good=err['position_mm']<=20 and err['axis_deg']<=2 and (condition=='B' or err['full_pose_deg']<=2)
                stable=stable+.01 if good else 0.
                f.write(json.dumps(dict(index=i,preparation_error=err,stable_s=stable,**row))+'\n')
                if stable+1e-9>=.5:break
            else:raise TimeoutError('Preparation reference not held within30s model time')
    except Exception as exc:
        dump(folder/'preparation_failure.json',dict(stage='preparation',step=i,condition=condition,detail=capture_failure(arm,exc,.01)))
        raise
    arm.posture.set_target_from_configuration(arm.configuration)
    arm.axis_only=False;arm.position_priority=True
    row=execution_row(arm);dump(folder/'initial.json',row)
    dump(folder/'preparation_summary.json',dict(condition=condition,steps=i+1,wall_s=time.perf_counter()-started,criteria='20mm,2deg required rotation,0.5s preparation reference; no task acceptance claim',final_error=err))
    return row

def markers(scene,actual_p,actual_r,condition):
    """Only renderer geoms; no model, collision, dynamics or IK edits."""
    def geom(kind,size,p,r,color):
        if scene.ngeom>=scene.maxgeom:raise RuntimeError('Render marker capacity exhausted')
        g=scene.geoms[scene.ngeom];scene.ngeom+=1
        mujoco.mjv_initGeom(g,kind,np.asarray(size,float),np.asarray(p,float),np.asarray(r,float).ravel(),np.asarray(color,np.float32))
        return g
    def arrow(p,v,color):
        g=geom(mujoco.mjtGeom.mjGEOM_ARROW,[0,0,0],[0,0,0],np.eye(3),color)
        mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_ARROW,.006,np.asarray(p),np.asarray(p)+np.asarray(v)*.10)
    def tool(p,r,color,front):
        p=np.asarray(p);r=np.asarray(r)
        geom(mujoco.mjtGeom.mjGEOM_SPHERE,[.009]*3,p,np.eye(3),color)
        arrow(p,r[:,2],color)
        geom(mujoco.mjtGeom.mjGEOM_BOX,[.023,.009,.0015],p+.06*r[:,2]+.024*r[:,0],r,front)
    geom(mujoco.mjtGeom.mjGEOM_SPHERE,[.012]*3,START,np.eye(3),[.8,.8,.8,.35])
    tool(GOAL,GOAL_R,[.15,1,.4,.8],[.5,1,.6,.65] if condition=='A' else [.5,.5,.5,.25])
    tool(actual_p,actual_r,[.1,.65,1,1],[1,.55,.1,1] if condition=='A' else [.7,.7,.7,.5])
