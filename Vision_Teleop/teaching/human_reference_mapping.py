"""Finite human-model reference selection for the existing four robot tasks.

Fixed similarity A and orientation-only Gao B share the same final human
position layout. Scores are engineering joint geometry, never comfort.
"""
import time
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
from .human_arm_model import HumanArm,left_log_inverse
from .paired_pose_mapping import PairedPoseMap,fit_paired_pose

def skew(v):
    x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])

def exp_jacobian(v):
    a=np.linalg.norm(v);k=skew(v)
    if a<1e-5:return np.eye(3)+(.5-a*a/24)*k+(1/6-a*a/120)*k@k
    return np.eye(3)+(1-np.cos(a))/a**2*k+(a-np.sin(a))/a**3*k@k

def geometry(q,arm):
    # q rows: base0,turned0,base1,turned1,...; actual visit starts base0.
    order=np.array([0,2,3,4,5,6,7,0,1]);sequence=q[order];d=(sequence-arm.reference)/arm.span
    changes=np.diff(sequence,axis=0)/arm.span
    value=float(np.mean(np.sum(d*d,axis=1))+.2*np.mean(np.sum(changes*changes,axis=1)))
    grad=np.zeros_like(q)
    for i,index in enumerate(order):grad[index]+=2*d[i]/arm.span/len(order)
    for i,change in enumerate(changes):
        term=.4*change/arm.span/len(changes);grad[order[i+1]]+=term;grad[order[i]]-=term
    return value,grad

def initial_mapping(params,arm,case):
    ref=arm.fk(arm.reference);center=ref['position']+params[:3];axes=Rotation.from_rotvec(params[3:6]).as_matrix();scale=np.exp(params[6])
    gref=np.asarray(case['robot_rotations'][0])@ref['rotation'].T
    g=Rotation.from_rotvec(params[7:10]).as_matrix()@gref
    return center,axes,scale,g

def to_map(params,arm,case):
    center,axes,scale,g=initial_mapping(params,arm,case)
    # Normalized input anchored to the original model reference hand point;
    # output anchored to unchanged world task0.
    href=arm.fk(arm.reference)['position']
    return PairedPoseMap(dict(scale=float(scale),axes=axes.tolist(),offset=(-scale*axes@(center-href)/.1).tolist(),orientation=g.tolist()))

def targets(case):
    delta=Rotation.from_euler('z',20,degrees=True).as_matrix()
    p=np.repeat(np.asarray(case['robot_positions_m']),2,axis=0)
    r=np.array([x for R in np.asarray(case['robot_rotations']) for x in [R,R@delta]])
    return p,r

def constraints_A(x,arm,case,derivative=False):
    params=x[:10];q=x[10:].reshape(8,7);center,A,g,G=initial_mapping(params,arm,case);wp,wr=targets(case)
    residual=[];jac=np.zeros((48,66)) if derivative else None
    ja=exp_jacobian(params[3:6]);jg=exp_jacobian(params[7:10]);world0=np.asarray(case['robot_positions_m'][0])
    for i,pose in enumerate(q):
        f=arm.fk(pose,derivative);u=A@(f['position']-center);p=world0+g*u;E=G@f['rotation']@wr[i].T;v=Rotation.from_matrix(E).as_rotvec()
        residual.extend(np.r_[20*(p-wp[i]),v])
        if derivative:
            row=6*i;col=10+7*i;j=f['jacobian'];loginv=left_log_inverse(v)
            jac[row:row+3,:3]=-20*g*A;jac[row:row+3,3:6]=-20*g*skew(u)@ja;jac[row:row+3,6]=20*g*u
            jac[row+3:row+6,7:10]=loginv@jg
            jac[row:row+3,col:col+7]=20*g*A@j[:3];jac[row+3:row+6,col:col+7]=loginv@G@j[3:]
    return jac if derivative else np.asarray(residual)

def constraints_B(x,arm,positions,derivative=False):
    q=x.reshape(8,7);delta=Rotation.from_euler('z',20,degrees=True).as_matrix();residual=[];jac=np.zeros((36,56)) if derivative else None
    for i,p in enumerate(positions):
        before=arm.fk(q[2*i],derivative);after=arm.fk(q[2*i+1],derivative)
        E=after['rotation']@(before['rotation']@delta).T;v=Rotation.from_matrix(E).as_rotvec()
        residual.extend(np.r_[20*(before['position']-p),20*(after['position']-p),v])
        if derivative:
            row=9*i;col=14*i;loginv=left_log_inverse(v)
            jac[row:row+3,col:col+7]=20*before['jacobian'][:3];jac[row+3:row+6,col+7:col+14]=20*after['jacobian'][:3]
            jac[row+6:row+9,col:col+7]=-loginv@E@before['jacobian'][3:];jac[row+6:row+9,col+7:col+14]=loginv@after['jacobian'][3:]
    return jac if derivative else np.asarray(residual)

def select_references(arm,case,settings):
    logs=[];bestA=None;start=time.perf_counter();wp,wr=targets(case)
    pbounds=[(-.08,.08)]*3+[(-np.pi/3,np.pi/3)]*3+[(np.log(.75),np.log(2.))]+[(-np.pi/4,np.pi/4)]*3
    boundsA=pbounds+list(zip(np.tile(arm.lower,8),np.tile(arm.upper,8)))
    class Budget(RuntimeError):pass
    def optimize(label,seed,bounds,constraints,offset,started):
        trace=[];best=None;calls=0
        def objective(x):
            nonlocal best,calls
            if time.perf_counter()-started>settings['wall_budget_per_method_s']:raise Budget('Frozen method wall budget')
            calls+=1;value,_=geometry(x[offset:].reshape(8,7),arm);res=float(np.max(np.abs(constraints(x))))
            feasible=res<=settings['constraint_tolerance']
            if feasible and (best is None or value<best['score']):best=dict(x=x.copy(),score=value,constraint_max=res)
            trace.append(dict(call=calls,score=value,constraint_max=res,feasible=feasible))
            return value
        def gradient(x):
            _,g=geometry(x[offset:].reshape(8,7),arm);return np.r_[np.zeros(offset),g.ravel()]
        try:
            result=minimize(objective,seed,jac=gradient,bounds=bounds,method='SLSQP',constraints=[dict(type='eq',fun=constraints,jac=lambda x:constraints(x,True))],
                options=dict(maxiter=settings['iterations_per_start'],ftol=1e-10,disp=False))
            objective(result.x);status=dict(success=bool(result.success),message=str(result.message),nit=result.nit)
        except Budget as exc:status=dict(success=False,message=str(exc))
        logs.append(dict(label=label,status=status,calls=calls,trace=trace,best=None if best is None else dict(score=best['score'],constraint_max=best['constraint_max'],x=best['x'].tolist())))
        return best
    for index,(scale,zdeg) in enumerate(settings['A_initializations']):
        if time.perf_counter()-start>settings['wall_budget_per_method_s']:break
        params=np.r_[np.zeros(3),[0,0,np.deg2rad(zdeg)],np.log(scale),np.zeros(3)];center,A,g,G=initial_mapping(params,arm,case)
        previous=arm.reference.copy();poses=[]
        for pp,rr in zip(wp,wr):
            result=arm.ik(center+A.T@(pp-wp[0])/g,G.T@rr,previous,max_nfev=60)
            previous=result['q'];poses.append(previous.copy())
        seed=np.r_[params,np.array(poses).ravel()]
        found=optimize(f'A_start{index}',seed,boundsA,lambda x,derivative=False:constraints_A(x,arm,case,derivative),10,start)
        if found and (bestA is None or found['score']<bestA['score']):bestA=found
    if bestA is None:return dict(status='A_not_found_within_budget',optimization=logs)
    params=bestA['x'][:10];qA=bestA['x'][10:].reshape(8,7);mapA=to_map(params,arm,case);href=arm.fk(arm.reference)['position']
    positions=mapA.inverse((np.asarray(case['robot_positions_m'])-wp[0])/.1,np.asarray(case['robot_rotations']))[0]*.1+href
    startB=time.perf_counter();bestB=None;boundsB=list(zip(np.tile(arm.lower,8),np.tile(arm.upper,8)))
    seeds=[qA.ravel(),np.tile(arm.reference,8),(.5*qA+.5*arm.reference).ravel()]
    for i,seed in enumerate(seeds):
        if time.perf_counter()-startB>settings['wall_budget_per_method_s']:break
        found=optimize(f'B_start{i}',seed,boundsB,lambda x,derivative=False:constraints_B(x,arm,positions,derivative),0,startB)
        if found and (bestB is None or found['score']<bestB['score']):bestB=found
    out=dict(status='selected' if bestB else 'B_not_found_within_budget',optimization=logs,
        human_anchor_m=href.tolist(),world_anchor_m=wp[0].tolist(),reference_positions_m=positions.tolist(),
        A=dict(params=params.tolist(),q=qA.tolist(),score=bestA['score'],map=mapA.specification()))
    if bestB is not None:
        qB=bestB['x'].reshape(8,7);hr=np.array([arm.fk(q)['rotation'] for q in qB[::2]])
        # Exact A position pairs must produce no nonlinear translation layers.
        mapB,trace=fit_paired_pose((positions-href)/.1,(np.asarray(case['robot_positions_m'])-wp[0])/.1,hr,np.asarray(case['robot_rotations']),**settings['gao_fit'])
        assert all(layer.translation is None for layer in mapB.layers)
        assert np.allclose(mapB.scale*mapB.axes,mapA.scale*mapA.axes,atol=1e-10) and np.allclose(mapB.offset,mapA.offset,atol=1e-10)
        out['B']=dict(q=qB.tolist(),score=bestB['score'],map=mapB.specification(),fit_trace=trace,reference_rotations=hr.tolist())
    return out

def continuous_human_input(arm,mapping,task_rows,human_anchor,world_anchor,settings,initial_q):
    wp=np.array([r['independent_task_position_m'] for r in task_rows]);wr=np.array([r['independent_task_rotation'] for r in task_rows])
    desiredp,desiredr=mapping.inverse((wp-world_anchor)/.1,wr);desiredp=.1*desiredp+human_anchor
    nodes=sorted(set(range(0,len(task_rows),5))|{len(task_rows)-1}|{i for i in range(1,len(task_rows)) if task_rows[i]['phase']!=task_rows[i-1]['phase']})
    # Each design's known first reference is its explicit model initial pose.
    # Subsequent IK has the identical task-only continuation strategy; design
    # poses are never substituted for the actually reconstructed trajectory.
    previous=np.asarray(initial_q).copy();solved=[];failure=None
    for n,i in enumerate(nodes):
        if n and np.linalg.norm(desiredp[i]-desiredp[nodes[n-1]])<1e-12 and np.linalg.norm(desiredr[i]-desiredr[nodes[n-1]])<1e-12:
            result=solved[-1]['solve'].copy();q=previous.copy()
        else:
            result=arm.ik(desiredp[i],desiredr[i],previous,max_nfev=settings['ik_max_nfev'],max_change=None if n==0 else settings['maximum_step_rad'])
            q=result['q']
        stored={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in result.items()}
        solved.append(dict(index=i,solve=stored,q=np.asarray(q).tolist()))
        if not result['found']:
            failure=dict(index=i,source_time_s=task_rows[i]['source_time_s'],reason='Current bounded continuous human IK not found; not physiological impossibility',solve=stored);break
        previous=np.asarray(q).copy()
    if failure:return dict(status='human_ik_failed',nodes=solved,failure=failure,unexecuted_from=solved[-1]['index'])
    qnodes=np.array([x['q'] for x in solved]);q=np.column_stack([np.interp(np.arange(len(task_rows)),nodes,qnodes[:,j]) for j in range(7)])
    actualp=[];actualr=[];landmarks=[]
    for pose in q:
        f=arm.fk(pose,landmarks=True);actualp.append(f['position']);actualr.append(f['rotation']);landmarks.append(np.array(list(f['landmarks'].values())))
    actualp=np.array(actualp);actualr=np.array(actualr);mappedp,mappedr=mapping.forward((actualp-human_anchor)/.1,actualr)
    return dict(status='complete',nodes=solved,q=q,human_position=actualp,human_rotation=actualr,landmarks=np.array(landmarks),
        desired_human_position=desiredp,desired_human_rotation=desiredr,mapped_position=.1*mappedp+world_anchor,mapped_rotation=mappedr)
