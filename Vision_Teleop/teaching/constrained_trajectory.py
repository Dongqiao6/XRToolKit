"""Same-curve constraints for the bounded offline study, using SciPy trust-constr.

No live mapping or new controller. Collocation and dense checks share the 5ms
finite-difference convention. Sampling is not a continuous safety proof.
"""
import time
import numpy as np
from scipy.optimize import minimize, least_squares, Bounds, NonlinearConstraint, LinearConstraint
from scipy.sparse import csr_matrix, kron, eye, block_diag, vstack
from .offline_trajectory import joint_curve, logjac, PathProblem
from scipy.spatial.transform import Rotation

class FidelityProblem(PathProblem):
    """Same fidelity/task/smoothness terms, with constraints removed from cost.

    Constraint terms belong to trust-constr inequalities, not a competing soft
    cost. This is an explicit baseline revision, never retroactive old results.
    """
    def evaluate(self,x,jac=False):
        q=self.unpack(x);poses=[self.kin.at(v,True) for v in q]
        values=[];rows=[];cols=[];entries=[]
        def add(value,blocks):
            base=len(values);values.extend(np.atleast_1d(value))
            if jac:
                for node,block in blocks.items():
                    if node not in self.column:continue
                    block=np.atleast_2d(block);a,b=np.nonzero(block);rows.extend(base+a);cols.extend(self.column[node]+b);entries.extend(block[a,b])
        def pose(i,p,r,wp,wr):
            pp,rr,jp,jr=poses[i]
            if wp:add(wp*(pp-p)/.02,{i:wp*jp/.02})
            if wr:
                v=Rotation.from_matrix(rr@r.T).as_rotvec();add(wr*v/np.deg2rad(2),{i:wr*logjac(v)@jr/np.deg2rad(2)})
        for i in range(self.N):
            if self.obs is not None:pose(i,self.obs['p'][i],self.obs['r'][i],float(self.obs['mp'][i]),float(self.obs['mr'][i]))
            if self.prior is not None:pose(i,self.prior[0][i],self.prior[1][i],.5,.5)
            if self.task['mask'][i]:pose(i,self.task['p'][i],self.task['r'][i],10.,10.)
        for i in range(1,self.N):
            dt=self.t[i]-self.t[i-1];D=.05*np.eye(self.d)/(.6*dt);add(.05*(q[i]-q[i-1])/(.6*dt),{i:D,i-1:-D})
        for i in range(1,self.N-1):
            h0=self.t[i]-self.t[i-1];h1=self.t[i+1]-self.t[i];coeff=np.array([1/h0,-1/h0-1/h1,1/h1])*2/(h0+h1)
            add(.02*sum(c*q[j] for c,j in zip(coeff,[i-1,i,i+1])),{j:.02*c*np.eye(self.d) for c,j in zip(coeff,[i-1,i,i+1])})
        if jac:return csr_matrix((entries,(rows,cols)),shape=(len(values),self.width))
        return np.array(values)

class CurveConstraints:
    def __init__(self,problem,indices):
        self.problem=problem;self.kin=problem.kin;self.d=problem.d;self.width=problem.width
        self.indices=np.unique(indices).astype(int);self.h=.005
        basis=joint_curve(problem.t,np.eye(len(problem.t)))
        affected=np.zeros(len(self.indices),bool)
        for offset in [-1,0,1]:
            S=basis(np.clip(self.indices+offset,0,35398)*self.h)
            affected|=np.any(abs(S[:,problem.mutable])>1e-14,axis=1)
        self.indices=self.indices[affected]
        # Include actual neighboring dense-check samples, not node secants.
        self.all_indices=np.unique(np.clip(np.r_[self.indices-1,self.indices,self.indices+1],0,35398))
        self.t=self.all_indices*self.h
        self.S=csr_matrix(basis(self.t));self.S.eliminate_zeros()
        self.B=kron(self.S[:,problem.mutable],eye(self.d),format='csr')
        self.Bd=kron(csr_matrix(basis(self.t,1))[:,problem.mutable],eye(self.d),format='csr')
        self.constant=self.S@problem.initial;self.constant-=self.S[:,problem.mutable]@problem.initial[problem.mutable]
        self.dconstant=basis(self.t,1)@problem.initial-basis(self.t,1)[:,problem.mutable]@problem.initial[problem.mutable]
        self.lookup={int(v):i for i,v in enumerate(self.all_indices)}
        self.centers=np.array([self.lookup[i] for i in self.indices])
        self.speed_indices=self.indices[self.indices>0];self.acc_indices=self.indices[(self.indices>0)&(self.indices<35398)]
        def differences(ids,offsets,coeffs):
            rows=[];cols=[];values=[]
            for j,i in enumerate(ids):
                for o,c in zip(offsets,coeffs):rows.append(j);cols.append(self.lookup[i+o]);values.append(c)
            return csr_matrix((values,(rows,cols)),shape=(len(ids),len(self.t)))
        self.Dv=differences(self.speed_indices,[-1,0],[-1/self.h,1/self.h]);self.Da=differences(self.acc_indices,[-1,0,1],[1/self.h**2,-2/self.h**2,1/self.h**2])
        # Equivalent Hermite-to-Bezier control coefficients bound the ENTIRE
        # joint curve by convex hull (sufficient, potentially conservative).
        # Avoid tens of thousands of almost duplicate sampled linear bounds.
        I=np.eye(len(problem.t));h=np.diff(problem.t)[:,None];slope=basis(problem.t,1)
        P=np.vstack([I[:-1]+h*slope[:-1]/3,I[1:]-h*slope[1:]/3])
        V=np.vstack([slope,3*(I[1:]-I[:-1])/h-slope[:-1]-slope[1:]])
        Pconst=P@problem.initial-P[:,problem.mutable]@problem.initial[problem.mutable]
        Vconst=V@problem.initial-V[:,problem.mutable]@problem.initial[problem.mutable]
        self.qlinear=vstack([kron(csr_matrix(P[:,problem.mutable]),eye(self.d)),kron(csr_matrix(V[:,problem.mutable]),eye(self.d))],format='csr')
        self.linear_lb=np.r_[(self.kin.low-Pconst).ravel(),(-.6-Vconst).ravel()]
        self.linear_ub=np.r_[(self.kin.high-Pconst).ravel(),(.6-Vconst).ravel()]
        self.linear_active=np.asarray(self.qlinear.getnnz(axis=1)>0)
        self.last_x=None;self.calls=0;self.jac_calls=0;self.began=None;self.fixed_bad=None
    def linear(self):
        return LinearConstraint(self.qlinear[self.linear_active],self.linear_lb[self.linear_active],self.linear_ub[self.linear_active],keep_feasible=True)
    def evaluate(self,x):
        if self.last_x is not None and np.array_equal(x,self.last_x):return
        if self.began is not None and time.perf_counter()-self.began>120:raise TimeoutError('Shared120s planning budget')
        q=(self.B@x).reshape(-1,self.d)+self.constant;poses=[];dist=[];grad=[]
        for v in q:
            poses.append(self.kin.at(v,True));distance,gradient=self.kin.minimum_distance_gradient()
            # Existing distance Jacobian is active below8mm. Saturation above
            # that activation distance keeps value/J consistent and preserves
            # exactly the original5mm feasible set; dense audit retains raw d.
            dist.append(min(distance,.008));grad.append(gradient)
        p=np.array([v[0] for v in poses]);R=np.array([v[1] for v in poses]);jp=block_diag([v[2] for v in poses],format='csr')@self.B;jr=block_diag([v[3] for v in poses],format='csr')@self.B
        values=[];matrices=[]
        for D,limit in [(self.Dv,.1),(self.Da,.2)]:
            v=D@p;norm=np.linalg.norm(v,axis=1);unit=v/np.maximum(norm[:,None],1e-12)
            values.append(norm/limit-1);matrices.append(block_diag([a[None,:]/limit for a in unit],format='csr')@kron(D,eye(3),format='csr')@jp)
        va=[];J0=[];J1=[];ids0=[];ids1=[]
        for i in self.speed_indices:
            i0=self.lookup[i-1];i1=self.lookup[i];E=R[i1]@R[i0].T;v=Rotation.from_matrix(E).as_rotvec();n=np.linalg.norm(v);row=(v/max(n,1e-12))@logjac(v)/(self.h*.5)
            va.append(n/(self.h*.5)-1);J0.append((-row@E)[None,:]);J1.append(row[None,:]);ids0.extend(range(i0*3,i0*3+3));ids1.extend(range(i1*3,i1*3+3))
        values.append(np.array(va));matrices.append(block_diag(J0,format='csr')@jr[ids0]+block_diag(J1,format='csr')@jr[ids1])
        values.append((.005-np.array(dist))/.005);matrices.append(-block_diag([a[None,:]/.005 for a in grad],format='csr')@self.B)
        self.values=np.concatenate(values);self.J=vstack(matrices,format='csr')
        # Select by curve support, not a current zero gradient at e.g. v=0.
        active_geom=np.asarray(self.B.getnnz(axis=1).reshape(-1,self.d).sum(axis=1)>0)
        speed_active=np.array([active_geom[self.lookup[i]] or active_geom[self.lookup[i-1]] for i in self.speed_indices])
        acc_active=np.array([any(active_geom[self.lookup[i+j]] for j in [-1,0,1]) for i in self.acc_indices])
        self.active=np.r_[speed_active,acc_active,speed_active,active_geom]
        self.fixed_bad=float(max(0.,self.values[~self.active].max(initial=-np.inf),self.linear_lb[~self.linear_active].max(initial=-np.inf),-self.linear_ub[~self.linear_active].min(initial=np.inf)))
        self.last_x=x.copy();self.calls+=1
    def fun(self,x):self.evaluate(x);return self.values[self.active]
    def jac(self,x):self.evaluate(x);self.jac_calls+=1;return self.J[self.active]
    def nonlinear(self,x):
        self.evaluate(x)
        return NonlinearConstraint(self.fun,-np.inf,0.,jac=self.jac,hess=lambda x,v:csr_matrix((self.width,self.width)),keep_feasible=True)

class ConstrainedSolve:
    def __init__(self,problem,dense_check):
        self.problem=problem;self.dense_check=dense_check;self.evaluations=0;self.jac_calls=0;self.rounds=[];self.trace=[];self.cache_x=None;self.last_x=None;self.restoration=[];self.constraints=[]
    def evaluate(self,x):
        if time.perf_counter()-self.began>120 or self.evaluations>=300:raise TimeoutError('Shared120s/300objective budget')
        if self.cache_x is not None and np.array_equal(self.cache_x,x):return
        self.res=self.problem.evaluate(x);self.J=self.problem.evaluate(x,True);self.cache_x=x.copy();self.evaluations+=1;self.jac_calls+=1
    def fun(self,x):self.evaluate(x);return float(self.res@self.res/2)
    def jac(self,x):self.evaluate(x);return self.J.T@self.res
    def hess(self,x):self.evaluate(x);return self.J.T@self.J+eye(len(x),format='csr')*1e-8
    def restore(self,c,x):
        """Standard phase-I constraint feasibility, before the hard NLP.

        No observation/task residual competes with feasibility. Every function
        evaluation consumes the SAME300 budget. Any residual remaining fails
        the hard check; phase-I convergence itself never means acceptance.
        """
        class Feasible(Exception):pass
        candidate=x.copy();begin=self.evaluations;start=time.perf_counter();A=c.qlinear[c.linear_active];lo=c.linear_lb[c.linear_active];hi=c.linear_ub[c.linear_active]
        def values(v):
            if time.perf_counter()-self.began>120 or self.evaluations>=300:raise TimeoutError('Shared120s/300objective budget')
            nonlinear=c.fun(v);linear=A@v
            return nonlinear,lo-linear,linear-hi
        def fun(v):
            nonlocal candidate
            parts=values(v);self.evaluations+=1;candidate=v.copy();self.last_x=v.copy()
            if max(a.max(initial=-np.inf) for a in parts)<=-1e-8:raise Feasible()
            return np.concatenate([np.maximum(a+1e-7,0.) for a in parts])
        def jac(v):
            parts=values(v);self.jac_calls+=1
            return vstack([c.jac(v).multiply((parts[0]>-1e-7)[:,None]),-A.multiply((parts[1]>-1e-7)[:,None]),A.multiply((parts[2]>-1e-7)[:,None])],format='csr')
        status='incomplete'
        try:
            result=least_squares(fun,x,jac=jac,bounds=(np.tile(self.problem.kin.low,sum(self.problem.mutable)),np.tile(self.problem.kin.high,sum(self.problem.mutable))),method='trf',tr_solver='lsmr',max_nfev=max(1,300-self.evaluations),ftol=1e-10,xtol=1e-10,gtol=1e-10,tr_options=dict(maxiter=100,atol=1e-8,btol=1e-8))
            candidate=result.x;status=str(result.message)
        except Feasible:status='collocation_feasible'
        finally:self.restoration.append(dict(status=status,function_evaluations=self.evaluations-begin,wall_s=time.perf_counter()-start))
        return candidate
    def solve(self):
        self.began=time.perf_counter();self.problem.began=None;x=self.problem.initial[self.problem.mutable].ravel();self.last_x=x.copy()
        # Starting .1s grid plus every knot and adjacent mid-knot. Uniform for all.
        knots=np.round(self.problem.t/.005).astype(int)
        indices=set(range(0,35399,20))|set(knots)|set(((knots[1:]+knots[:-1])//2).tolist())|{35398}
        status='not_started';result=None;check=None
        try:
            for refinement in range(4):
                if time.perf_counter()-self.began>120 or self.evaluations>=300:raise TimeoutError('Shared120s/300objective budget')
                c=CurveConstraints(self.problem,sorted(indices));c.began=self.began;self.constraints.append(c);nc=c.nonlinear(x)
                if c.fixed_bad>1e-7:status='fixed_outside_constraint_violation';break
                x=self.restore(c,x);self.last_x=x.copy()
                c.evaluate(x);linear=c.qlinear@x
                if c.fun(x).max(initial=-np.inf)>0 or np.any(linear[c.linear_active]<c.linear_lb[c.linear_active]) or np.any(linear[c.linear_active]>c.linear_ub[c.linear_active]):
                    status='restoration_failed';break
                stable=0;previous_cost=None;round_start=time.perf_counter();round_evaluations=self.evaluations
                def callback(value,state):
                    nonlocal stable,previous_cost
                    self.last_x=value.copy();self.trace.append(dict(nfev=self.evaluations,cost=float(state.fun),violation=float(state.constr_violation),wall_s=time.perf_counter()-self.began))
                    change=np.inf if previous_cost is None else abs(state.fun-previous_cost)/max(1.,abs(previous_cost))
                    stable=stable+1 if state.constr_violation<=1e-7 and change<1e-5 else 0;previous_cost=float(state.fun)
                    return stable>=5 or self.evaluations-round_evaluations>=60 or time.perf_counter()-round_start>=25 or self.evaluations>=300 or time.perf_counter()-self.began>120
                result=minimize(self.fun,x,method='trust-constr',jac=self.jac,hess=self.hess,
                    bounds=Bounds(np.tile(self.problem.kin.low,sum(self.problem.mutable)),np.tile(self.problem.kin.high,sum(self.problem.mutable)),keep_feasible=True),
                    constraints=[c.linear(),nc],callback=callback,options=dict(maxiter=300,gtol=1e-6,xtol=1e-9,barrier_tol=1e-8,sparse_jacobian=True,factorization_method='AugmentedSystem'))
                x=result.x.copy();self.last_x=x.copy()
                check,bad=self.dense_check(self.problem.unpack(x))
                self.rounds.append(dict(refinement=refinement,collocation_points=len(indices),constraint_evaluations=c.calls,jacobian_evaluations=c.jac_calls,status=int(result.status),message=str(result.message),optimality=float(result.optimality),constraint_violation=float(result.constr_violation),dense_valid=check['valid'],new_violating_points=len(bad)))
                if check['valid']:status='feasible';break
                if self.evaluations>=300 or time.perf_counter()-self.began>=120:status='budget_exhausted';break
                new=set(bad)-indices
                if not new:status='constraint_solver_failed';break
                indices|=new;status='refinement_limit'
        except TimeoutError as exc:status='budget_exhausted'
        return self.problem.unpack(self.last_x),dict(status=status,wall_s=time.perf_counter()-self.began,objective_evaluations=self.evaluations,objective_jacobian_evaluations=self.jac_calls,restoration=self.restoration,constraint_evaluations=sum(c.calls for c in self.constraints),constraint_jacobian_evaluations=sum(c.jac_calls for c in self.constraints),rounds=self.rounds,trace=self.trace,solver='scipy1.15.3 feasibility phaseI TRF/LSMR then trust-constr, sparse exact constraint Jacobian, GaussNewton objective Hessian, zero constraint-Hessian SQP approximation',optimum_claimed=False)
