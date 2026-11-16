"""MoBL author-converted upper limb: seven independent kinematic coordinates.

No muscle control or dynamics. Source joint axes, ranges, reference transforms
and polynomial shoulder/wrist couplings remain intact. MuJoCo FK does not solve
equalities: we explicitly expand them, then chain the full Jacobian.
Model assets carry their own non-commercial research license and attribution.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares,lsq_linear

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'results/human_reference/20261010_model_mapping_01/sources/o2m/models/converted/MoBL_ARMS_model_for_mujoco_converted/MoBL_ARMS_model_for_mujoco_converted.xml'
COORDINATES=('elv_angle','shoulder_elv','shoulder_rot','elbow_flexion','pro_sup','deviation','flexion')
# Explicit engineering reference, not a measured/most comfortable human pose.
REFERENCE_DEG=np.array([30.,45.,-20.,90.,0.,0.,0.])

def kinematic_xml(source=SOURCE,visual=False):
    root=ET.parse(source).getroot()
    # All body/joint/inertial/reference transforms and joint equalities stay.
    # No upstream converter or muscle code is executed.
    for tag in ['actuator','tendon','contact']:
        for node in root.findall(tag):root.remove(node)
    if not visual:
        for parent in root.iter():
            for child in list(parent):
                if child.tag in ['geom','asset']:parent.remove(child)
    else:
        for mesh in root.findall('asset/mesh'):
            mesh.set('file',str((Path(source).parent/mesh.get('file')).resolve()))
    root.insert(0,ET.Comment('KINEMATICS ONLY derivative: removed actuators/tendons/contact; optional meshes removed. Original chain, ranges and couplings retained. See original model license and attribution in sources.'))
    return ET.tostring(root,encoding='unicode')

def left_log_inverse(v):
    v=np.asarray(v);a=np.linalg.norm(v);x,y,z=v
    k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    c=1/12 if a<1e-5 else (1-(a/2)/np.tan(a/2))/(a*a)
    return np.eye(3)-.5*k+c*k@k

class HumanArm:
    frame='mobl_model_world'
    control_point='origin_of_original_hand_body'
    def __init__(self,source=SOURCE,visual=False):
        self.xml=kinematic_xml(source,visual);self.model=mujoco.MjModel.from_xml_string(self.xml);self.data=mujoco.MjData(self.model)
        self.ids=np.array([mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,n) for n in COORDINATES])
        assert np.all(self.ids>=0) and self.model.nq==self.model.nv==21,'Expected original21scalar coordinates including7 independent'
        self.address=self.model.jnt_qposadr[self.ids];self.lower,self.upper=self.model.jnt_range[self.ids].T.copy();self.span=self.upper-self.lower
        self.reference=np.radians(REFERENCE_DEG);self.hand=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,'hand')
        self.couplings=[]
        for i in range(self.model.neq):
            if self.model.eq_type[i]==mujoco.mjtEq.mjEQ_JOINT and self.model.eq_active0[i]:
                a=int(self.model.eq_obj1id[i]);b=int(self.model.eq_obj2id[i]);self.couplings.append((int(self.model.jnt_qposadr[a]),-1 if b<0 else int(self.model.jnt_qposadr[b]),self.model.eq_data[i,:5].copy()))
        assert len(self.couplings)==14 and len(self.address)==7
        self.landmark_ids={n:mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,n) for n in ['thorax','clavicle','scapula','humerus','ulna','radius','proximal_row','hand']}

    def expand(self,q):
        q=np.asarray(q,float)
        if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('Seven finite joint coordinates in radians required')
        if np.any(q<self.lower-1e-8) or np.any(q>self.upper+1e-8):raise ValueError('Human coordinates outside original model limits')
        full=self.model.qpos0.copy();full[self.address]=q;lift=np.zeros((self.model.nv,7));lift[self.address]=np.eye(7)
        known=set(self.address);pending=list(self.couplings)
        while pending:
            progressed=False
            for item in list(pending):
                a,b,c=item
                if b>=0 and b not in known:continue
                x=0. if b<0 else full[b]-self.model.qpos0[b]
                full[a]=self.model.qpos0[a]+np.polynomial.polynomial.polyval(x,c)
                lift[a]=0 if b<0 else np.polynomial.polynomial.polyval(x,np.arange(1,5)*c[1:])*lift[b]
                known.add(a);pending.remove(item);progressed=True
            if not progressed:raise ValueError('Unresolved or cyclic source coupling')
        return full,lift

    def fk(self,q,jacobian=False,landmarks=False):
        full,lift=self.expand(q);self.data.qpos[:]=full
        mujoco.mj_kinematics(self.model,self.data);mujoco.mj_comPos(self.model,self.data)
        p=self.data.xpos[self.hand].copy();r=self.data.xmat[self.hand].reshape(3,3).copy();out=dict(position=p,rotation=r,q_full=full)
        if jacobian:
            jp=np.zeros((3,self.model.nv));jr=np.zeros_like(jp);mujoco.mj_jacBody(self.model,self.data,jp,jr,self.hand)
            out['jacobian']=np.vstack([jp,jr])@lift
        if landmarks:out['landmarks']={n:self.data.xpos[i].copy() for n,i in self.landmark_ids.items()}
        return out

    def ik_trf(self,p,r,start,*,max_nfev=80,max_change=None):
        p=np.asarray(p,float);r=np.asarray(r,float);start=np.asarray(start,float)
        if p.shape!=(3,) or r.shape!=(3,3) or not np.isfinite(p).all() or not np.isfinite(r).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-8) or np.linalg.det(r)<.999999:
            raise ValueError('Explicit finite metre point and proper full orientation required')
        low=self.lower.copy();high=self.upper.copy()
        if max_change is not None:low=np.maximum(low,start-max_change);high=np.minimum(high,start+max_change)
        def fun(q):
            f=self.fk(q);return np.r_[20*(f['position']-p),Rotation.from_matrix(f['rotation']@r.T).as_rotvec()]
        def jac(q):
            f=self.fk(q,True);v=Rotation.from_matrix(f['rotation']@r.T).as_rotvec()
            return np.vstack([20*f['jacobian'][:3],left_log_inverse(v)@f['jacobian'][3:]])
        result=least_squares(fun,np.clip(start,low+1e-12,high-1e-12),jac=jac,bounds=(low,high),max_nfev=max_nfev,ftol=1e-10,xtol=1e-10,gtol=1e-10)
        f=self.fk(result.x);pe=float(np.linalg.norm(f['position']-p));re=float(Rotation.from_matrix(f['rotation']@r.T).magnitude())
        return dict(q=result.x,position_error_m=pe,rotation_error_rad=re,within_original_limits=bool(np.all(result.x>=self.lower)&np.all(result.x<=self.upper)),
            solver_success=bool(result.success),nfev=result.nfev,message=result.message,
            found=bool(pe<=1e-4 and re<=np.deg2rad(.1)),acceptance='Numerical reconstruction1e-4m/.1deg, distinct from unchanged robot20mm/2deg task criterion')

    def ik(self,p,r,start,*,max_nfev=80,max_change=None):
        """Bounded damped minimum-increment Newton, same full-pose task.

        The six-row/seven-variable task is underdetermined. A positive
        1e-6 step damping selects a minimum increment, not a posture objective.
        Source bounds and the continuation box are hard bounds in each BVLS
        subproblem; fixed12-step backtracking must reduce the ORIGINAL task
        cost. No reference-posture residual or relaxed acceptance is added.
        """
        p=np.asarray(p,float);r=np.asarray(r,float);start=np.asarray(start,float)
        if p.shape!=(3,) or r.shape!=(3,3) or start.shape!=(7,) or not np.isfinite(np.r_[p,start]).all() or not np.isfinite(r).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-8) or np.linalg.det(r)<.999999:
            raise ValueError('Explicit finite metre point, seven-coordinate start and proper full orientation required')
        low=self.lower.copy();high=self.upper.copy()
        if max_change is not None:
            if not np.isfinite(max_change) or max_change<=0:raise ValueError('Positive finite continuation bound required')
            low=np.maximum(low,start-max_change);high=np.minimum(high,start+max_change)
        if np.any(low>=high):raise ValueError('Empty coordinate interval')
        q=np.clip(start,low,high);f=self.fk(q,True);nfev=1;iterations=0;history=[];message='Frozen function evaluation budget exhausted'
        def residual(f):return np.r_[20*(f['position']-p),Rotation.from_matrix(f['rotation']@r.T).as_rotvec()]
        error=residual(f)
        while nfev<max_nfev:
            # Use original tight optimizer convergence, then report unchanged
            # reconstruction acceptance separately; avoid a .1deg deadband.
            if np.linalg.norm(error)<1e-9:message='Task residual converged';break
            J=np.vstack([20*f['jacobian'][:3],left_log_inverse(error[3:])@f['jacobian'][3:]])
            linear=lsq_linear(np.vstack([J,.001*np.eye(7)]),np.r_[-error,np.zeros(7)],bounds=(low-q,high-q),method='bvls',tol=1e-12,max_iter=50)
            delta=linear.x;iterations+=1;oldcost=float(error@error);accepted=False
            for backtrack in range(12):
                if nfev>=max_nfev:break
                candidate=np.clip(q+(2.**-backtrack)*delta,low,high);nextf=self.fk(candidate,True);nfev+=1;nexterror=residual(nextf);cost=float(nexterror@nexterror)
                if cost<oldcost:
                    q=candidate;f=nextf;error=nexterror;accepted=True
                    history.append(dict(iteration=iterations,nfev=nfev,cost=cost,step_norm_rad=float(np.linalg.norm((2.**-backtrack)*delta)),backtracks=backtrack,linear_status=int(linear.status)));break
            if not accepted:message='No decreasing bounded step within fixed backtracking budget';break
        pe=float(np.linalg.norm(f['position']-p));re=float(Rotation.from_matrix(f['rotation']@r.T).magnitude());found=bool(pe<=1e-4 and re<=np.deg2rad(.1))
        return dict(q=q,position_error_m=pe,rotation_error_rad=re,within_original_limits=bool(np.all(q>=self.lower)&np.all(q<=self.upper)),
            solver_success=bool(np.linalg.norm(error)<1e-9),nfev=nfev,message=message,found=found,iterations=iterations,trace=history,
            solver='bounded_damped_minimum_increment_v1',damping=1e-6,
            acceptance='Numerical reconstruction1e-4m/.1deg, distinct from unchanged robot20mm/2deg task criterion')
