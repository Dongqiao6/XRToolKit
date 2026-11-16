"""Isolated finite offline representation / joint-path study. No live imports.

Generated poses never become observations. Fixed source time, no retiming.
"""
from pathlib import Path
import sys,time
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation,Slerp
from scipy.interpolate import CubicHermiteSpline
from scipy.optimize import least_squares,lsq_linear
from scipy.sparse import lil_matrix,csr_matrix
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'external/offline26_deps'))

def logjac(v):
    a=np.linalg.norm(v);x,y,z=v;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    c=1/12 if a<1e-5 else (1-a/2/np.tan(a/2))/(a*a)
    return np.eye(3)-.5*K+c*K@K

def continuous_wxyz(R):
    q=Rotation.from_matrix(R).as_quat()[:,[3,0,1,2]]
    for i in range(1,len(q)):
        if np.dot(q[i],q[i-1])<0:q[i]*=-1
    return q

def complete_observations(obs):
    t=obs['time'];mp=obs['position_valid'];mr=obs['rotation_valid']
    if mp.sum()<2 or mr.sum()<2 or not mp[0] or not mp[-1] or not mr[0] or not mr[-1]:raise ValueError('Need valid bounding observations; no endpoint invention')
    p=np.column_stack([np.interp(t,t[mp],obs['position'][mp,j]) for j in range(3)])
    r=Slerp(t[mr],Rotation.from_matrix(obs['rotation'][mr]))(t).as_matrix()
    return p,r

def pose_at(t,p,r,s):
    pos=np.column_stack([np.interp(s,t,p[:,j]) for j in range(3)])
    return pos,Slerp(t,Rotation.from_matrix(r))(s).as_matrix()

def representation(obs,kind,settings):
    p,r=complete_observations(obs);t=obs['time']
    if kind=='se3':return p,r,dict(kind=kind,generated=True,method='Linear metres and shortest-path SO3 SLERP; shared linear/SLERP completion first')
    from movement_primitives.dmp import CartesianDMP
    # Phase boundaries are common metadata. Fit each phase independently using
    # only the shared completed sequence, never the independent task targets.
    outputp=np.empty_like(p);outputr=np.empty_like(r);fits=[]
    changes=np.r_[0,np.flatnonzero(obs['phase'][1:]!=obs['phase'][:-1])+1,len(t)]
    for start,end in zip(changes[:-1],changes[1:]):
        times=t[start:end]-t[start];duration=float(times[-1]);y=np.c_[p[start:end],continuous_wxyz(r[start:end])]
        model=CartesianDMP(execution_time=duration,dt=.01,n_weights_per_dim=settings['dmp_weights'],int_dt=.01)
        model.imitate(times,y,regularization_coefficient=settings['dmp_regularization'],allow_final_velocity=False)
        tt,yy=model.open_loop(run_t=duration,step_function='rk4',quaternion_step_function='python')
        quaternion=yy[:,3:7];norm=np.linalg.norm(quaternion,axis=1);assert np.isfinite(yy).all() and norm.min()>.99
        rr=Rotation.from_quat((quaternion/norm[:,None])[:,[1,2,3,0]]).as_matrix()
        pp,rot=pose_at(tt,yy[:,:3],rr,np.clip(times,tt[0],tt[-1]));outputp[start:end]=pp;outputr[start:end]=rot
        fits.append(dict(start=int(start),end=int(end),execution_time=duration,weights_position=model.forcing_term_pos.weights_.tolist(),weights_rotation=model.forcing_term_rot.weights_.tolist(),minimum_quaternion_norm=float(norm.min()),maximum_quaternion_norm=float(norm.max())))
    return outputp,outputr,dict(kind=kind,generated=True,quaternion='wxyz, unit, sign continuous',fits=fits)

def joint_curve(t,q):
    slopes=np.empty_like(q);slopes[0]=0.;slopes[-1]=0.
    # Three-point derivative on the ACTUAL nonuniform timestamps. The plain
    # two-neighbor secant is only centered for equal h; using it at .01/.5s
    # phase seams creates artificial endpoint velocities and acceleration.
    h0=(t[1:-1]-t[:-2])[:,None];h1=(t[2:]-t[1:-1])[:,None]
    slopes[1:-1]=-h1/(h0*(h0+h1))*q[:-2]+(h1-h0)/(h0*h1)*q[1:-1]+h0/(h1*(h0+h1))*q[2:]
    return CubicHermiteSpline(t,q,slopes)

class Kinematics:
    def __init__(self,arm):
        self.model=arm.model;self.data=mujoco.MjData(arm.model);self.site=self.model.site('attachment_site').id;self.n=self.model.nv
        self.low=np.maximum(self.model.jnt_range[:,0],self.model.actuator_ctrlrange[:,0]);self.high=np.minimum(self.model.jnt_range[:,1],self.model.actuator_ctrlrange[:,1])
        from .basic_comparison import BareSafety
        self.guard=BareSafety(arm)
    def at(self,q,jac=False):
        self.data.qpos[:]=q;mujoco.mj_kinematics(self.model,self.data);mujoco.mj_comPos(self.model,self.data)
        p=self.data.site_xpos[self.site].copy();r=self.data.site_xmat[self.site].reshape(3,3).copy()
        if not jac:return p,r
        jp=np.zeros((3,self.n));jr=np.zeros_like(jp);mujoco.mj_jacSite(self.model,self.data,jp,jr,self.site)
        return p,r,jp,jr
    def distance(self):return self.guard.inspect(self.data)
    def minimum_distance_gradient(self):
        cap=.03;small=(cap,None,None,None)
        for a,b in self.guard.pairs:
            # Same conservative sphere filtering as the original guard.
            ra=self.model.geom_rbound[a];rb=self.model.geom_rbound[b]
            if ra>0 and rb>0 and np.linalg.norm(self.data.geom_xpos[a]-self.data.geom_xpos[b])>ra+rb+cap:continue
            line=np.zeros(6);distance=mujoco.mj_geomDistance(self.model,self.data,a,b,cap,line)
            if distance<small[0]:small=(distance,a,b,line)
        d,a,b,line=small;grad=np.zeros(self.n)
        if a is not None and d<.008:
            direction=line[3:]-line[:3];length=np.linalg.norm(direction)
            if length>1e-12:
                ja=np.zeros((3,self.n));jb=np.zeros_like(ja)
                mujoco.mj_jac(self.model,self.data,ja,None,line[:3],int(self.model.geom_bodyid[a]));mujoco.mj_jac(self.model,self.data,jb,None,line[3:],int(self.model.geom_bodyid[b]));grad=direction/length@(jb-ja)
        return d,grad
    def ik(self,p,r,start,budget=60):
        q=np.clip(start,self.low,self.high);calls=0
        for iteration in range(budget):
            pos,rot,jp,jr=self.at(q,True);v=Rotation.from_matrix(rot@r.T).as_rotvec();f=np.r_[20*(pos-p),v];calls+=1
            if np.linalg.norm(pos-p)<1e-5 and np.linalg.norm(v)<1e-4:break
            J=np.vstack([20*jp,logjac(v)@jr]);lo=np.maximum(self.low-q,-.15);hi=np.minimum(self.high-q,.15)
            dq=lsq_linear(np.vstack([J,.001*np.eye(self.n)]),np.r_[-f,np.zeros(self.n)],bounds=(lo,hi),method='bvls',tol=1e-10,max_iter=40).x
            accepted=False
            for fraction in [1.,.5,.25,.125,.0625]:
                pp,rr=self.at(q+fraction*dq);trial=np.r_[20*(pp-p),Rotation.from_matrix(rr@r.T).as_rotvec()]
                if trial@trial<f@f:q=q+fraction*dq;accepted=True;break
            if not accepted:break
        pp,rr=self.at(q);e=np.linalg.norm(pp-p);a=Rotation.from_matrix(rr@r.T).magnitude()
        return q,dict(position_m=float(e),rotation_rad=float(a),calls=calls,found=bool(e<1e-4 and a<.002))

def build_initial(kin,t,obs,prepared_reference):
    p,r=complete_observations(obs);pp,rr=pose_at(obs['time'],p,r,t);q=[];previous=prepared_reference.copy();audit=[];began=time.perf_counter()
    for i in range(len(t)):
        if time.perf_counter()-began>120:raise TimeoutError('Initialization120s fixed budget')
        if t[i]<=1.:value=previous.copy();info=dict(held_prepared_reference=True)
        elif i and np.linalg.norm(pp[i]-pp[i-1])<1e-12 and np.linalg.norm(rr[i]-rr[i-1])<1e-12:value=previous.copy();info=dict(reused_static_initialization=True)
        else:value,info=kin.ik(pp[i],rr[i],previous)
        q.append(value);audit.append(info);previous=value
    return np.array(q),audit

class PathProblem:
    """Same sparse nonlinear least-squares objective for both modification sets.

    Bounds are hard. Other constraints have a frozen conservative penalty target
    and ORIGINAL hard postvalidation. Never call penalized fitting a proof.
    """
    def __init__(self,kin,t,initial,obs,prior,task,mutable,settings):
        self.kin=kin;self.t=t;self.initial=initial.copy();self.mutable=np.array(mutable);self.d=kin.n;self.N=len(t);self.cfg=settings
        self.column={int(i):k*self.d for k,i in enumerate(np.flatnonzero(mutable))};self.width=len(self.column)*self.d
        self.obs=obs;self.prior=prior;self.task=task;self.rows=[];self.began=None;self.iterations=0;self.trace=[];self.best=None
    def unpack(self,x):
        q=self.initial.copy();q[self.mutable]=x.reshape(-1,self.d);return q
    def evaluate(self,x,jac=False):
        if self.began is not None and time.perf_counter()-self.began>120:raise TimeoutError('Frozen optimization120s budget')
        q=self.unpack(x);poses=[];distances=[]
        for v in q:
            poses.append(self.kin.at(v,True));distances.append(self.kin.minimum_distance_gradient())
        residual=[];row_indices=[];col_indices=[];values=[]
        def add(value,blocks):
            value=np.atleast_1d(value);base=len(residual);residual.extend(value)
            if jac:
                for node,block in blocks.items():
                    if node not in self.column:continue
                    block=np.atleast_2d(block);a,b=np.nonzero(block);row_indices.extend(base+a);col_indices.extend(self.column[node]+b);values.extend(block[a,b])
        def pose_res(node,pp,rr,wp,wr):
            pos,rot,jp,jr=poses[node]
            if wp:add(wp*(pos-pp)/.02,{node:wp*jp/.02})
            if wr:
                v=Rotation.from_matrix(rot@rr.T).as_rotvec();add(wr*v/np.deg2rad(2),{node:wr*logjac(v)@jr/np.deg2rad(2)})
        for i in range(self.N):
            if self.obs is not None:pose_res(i,self.obs['p'][i],self.obs['r'][i],float(self.obs['mp'][i]),float(self.obs['mr'][i]))
            if self.prior is not None:pose_res(i,self.prior[0][i],self.prior[1][i],.5,.5)
            if self.task['mask'][i]:pose_res(i,self.task['p'][i],self.task['r'][i],10.,10.)
            distance,gradient=distances[i];add([100*min(0.,distance-.006)/.005],{i:(100/.005*gradient if distance<.006 else np.zeros(self.d))[None,:]})
        for i in range(1,self.N):
            dt=self.t[i]-self.t[i-1];I=np.eye(self.d);vel=(q[i]-q[i-1])/dt
            add(.05*vel/.6,{i:.05*I/(.6*dt),i-1:-.05*I/(.6*dt)})
            excess=np.maximum(0,abs(vel)-.54);D=np.diag((abs(vel)>.54)*np.sign(vel))*100/(.6*dt)
            add(100*excess/.6,{i:D,i-1:-D})
            pos,rot,jp,jr=poses[i];p0,r0,jp0,jr0=poses[i-1];dv=(pos-p0)/dt;speed=np.linalg.norm(dv)
            g=np.zeros(3) if speed<1e-12 else dv/speed
            add([100*max(0.,speed-.09)/.1],{i:100/.1*g@jp/dt if speed>.09 else np.zeros(self.d),i-1:-100/.1*g@jp0/dt if speed>.09 else np.zeros(self.d)})
            E=rot@r0.T;v=Rotation.from_matrix(E).as_rotvec();omega=np.linalg.norm(v)/dt;g=np.zeros(3) if np.linalg.norm(v)<1e-12 else v/np.linalg.norm(v)
            J=logjac(v)
            add([100*max(0.,omega-.45)/.5],{i:100/.5*g@J@jr/dt if omega>.45 else np.zeros(self.d),i-1:-100/.5*g@J@E@jr0/dt if omega>.45 else np.zeros(self.d)})
        for i in range(1,self.N-1):
            h0=self.t[i]-self.t[i-1];h1=self.t[i+1]-self.t[i];coeff=np.array([1/h0,-1/h0-1/h1,1/h1])*2/(h0+h1)
            acc=sum(c*q[j] for c,j in zip(coeff,[i-1,i,i+1]));add(.02*acc,{j:.02*c*np.eye(self.d) for c,j in zip(coeff,[i-1,i,i+1])})
            ap=sum(c*poses[j][0] for c,j in zip(coeff,[i-1,i,i+1]));mag=np.linalg.norm(ap);g=ap/mag if mag>1e-12 else np.zeros(3)
            add([100*max(0.,mag-.18)/.2],{j:100/.2*c*g@poses[j][2] if mag>.18 else np.zeros(self.d) for c,j in zip(coeff,[i-1,i,i+1])})
        vector=np.array(residual)
        if jac:return csr_matrix((values,(row_indices,col_indices)),shape=(len(vector),self.width))
        self.iterations+=1;cost=float(vector@vector);self.trace.append(dict(evaluation=self.iterations,cost=cost,wall_s=0 if self.began is None else time.perf_counter()-self.began))
        if self.best is None or cost<self.best[0]:self.best=(cost,x.copy())
        return vector
    def solve(self):
        self.began=time.perf_counter();x=self.initial[self.mutable].ravel();low=np.tile(self.kin.low,sum(self.mutable));high=np.tile(self.kin.high,sum(self.mutable));status=None
        try:
            result=least_squares(lambda x:self.evaluate(x),x,jac=lambda x:self.evaluate(x,True),bounds=(low,high),method='trf',tr_solver='lsmr',
                max_nfev=300,ftol=1e-6,xtol=1e-6,gtol=1e-6,tr_options=dict(maxiter=100,atol=1e-6,btol=1e-6))
            status=dict(success=bool(result.success),status=int(result.status),message=str(result.message),nfev=result.nfev,optimality=float(result.optimality));x=result.x
        except TimeoutError as exc:
            status=dict(success=False,status='budget_exhausted',message=str(exc));x=x if self.best is None else self.best[1]
        return self.unpack(x),dict(**status,trace=self.trace,wall_s=time.perf_counter()-self.began,iterations_upper_bound=self.iterations)
