"""Independent formula implementation of Gao RAL2021 III/IV, offline only.

No upstream source copied. p'=Phi(p), R'=G(p)R, spatial angular velocities.
Algorithm1 line9 in the author-hosted v1 reverses the residual for its stated
left multiplication: use target @ current.T, as required by equations2/8 and
the local relative-orientation constraint. This deviation is explicit/tested.
"""
from dataclasses import dataclass,asdict
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize_scalar
from .diffeomorphic_position import Translation


def rotations(r):
    r=np.asarray(r,float)
    if r.shape[-2:]!=(3,3) or not np.isfinite(r).all() or not np.allclose(np.swapaxes(r,-1,-2)@r,np.eye(3),atol=1e-7,rtol=0) or not np.allclose(np.linalg.det(r),1,atol=1e-7,rtol=0):
        raise ValueError('Proper finite rotations required, not projected missing data')
    return r


def rot_error(a,b):
    return Rotation.from_matrix(rotations(a)@np.swapaxes(rotations(b),-1,-2)).magnitude()


def fit_similarity(x,y,rh,rr):
    """Least-squares positive similarity; constant orientation uses SO3 mean.

    The spatial axis rotation and fixed orientation relation are separately
    fitted from the SAME paired task data, favoring no arbitrary identity basis.
    """
    x=np.asarray(x,float);y=np.asarray(y,float)
    if x.ndim!=2 or x.shape!=y.shape or x.shape[1]!=3 or len(x)<3 or not np.isfinite([x,y]).all():raise ValueError('Finite paired3D positions required')
    xc=x-x.mean(0);yc=y-y.mean(0)
    if np.linalg.matrix_rank(xc)<2:raise ValueError('Noncollinear landmarks required for spatial similarity')
    u,s,vt=np.linalg.svd(yc.T@xc);d=np.diag([1.,1.,np.linalg.det(u@vt)]);a=u@d@vt
    scale=float(np.sum(s*np.diag(d))/np.sum(xc*xc))
    if scale<=0:raise ValueError('Positive scale required')
    offset=y.mean(0)-scale*a@x.mean(0)
    g=Rotation.from_matrix(rotations(rr)@np.swapaxes(rotations(rh),-1,-2)).mean().as_matrix()
    return dict(scale=scale,axes=a.tolist(),offset=offset.tolist(),orientation=g.tolist())


@dataclass(frozen=True)
class PoseLayer:
    translation: dict | None
    rotation_center: list
    rotation_vector: list  # full SO3 angle; quaternion log would be half angle
    rotation_rho: float


class PairedPoseMap:
    def __init__(self,initial,layers=()):
        self.initial=initial;self.layers=tuple(layers)
        self.scale=float(initial['scale']);self.axes=rotations(initial['axes']);self.g0=rotations(initial['orientation']);self.offset=np.asarray(initial['offset'],float)
        if self.scale<=0 or not np.isfinite(self.scale) or self.offset.shape!=(3,) or not np.isfinite(self.offset).all():raise ValueError('Finite positive similarity required')
        for layer in self.layers:
            if layer.translation is not None:Translation(**layer.translation)
            if np.asarray(layer.rotation_center).shape!=(3,) or np.asarray(layer.rotation_vector).shape!=(3,) or not np.isfinite([*layer.rotation_center,*layer.rotation_vector,layer.rotation_rho]).all() or layer.rotation_rho<0:raise ValueError('Finite weighted rotation required')

    def fields(self,x):
        """Position, G, Dphi, and J_G (rad per normalized input unit)."""
        x=np.asarray(x,float);single=x.ndim==1;x=np.atleast_2d(x)
        if x.ndim!=2 or x.shape[1]!=3 or not np.isfinite(x).all():raise ValueError('Finite positions required')
        y=self.scale*x@self.axes.T+self.offset
        j=np.broadcast_to(self.scale*self.axes,(len(x),3,3)).copy()
        g=np.broadcast_to(self.g0,(len(x),3,3)).copy();b=np.zeros_like(g)
        for layer in self.layers:
            if layer.translation is not None:
                t=Translation(**layer.translation);j=t.jacobian(y)@j;y=t.apply(y)
            z=y-np.asarray(layer.rotation_center);rho=layer.rotation_rho;a=np.asarray(layer.rotation_vector)
            weight=np.exp(-rho*rho*np.sum(z*z,axis=1))
            grad=-2*rho*rho*weight[:,None]*z
            e=Rotation.from_rotvec(weight[:,None]*a).as_matrix()
            # dExp(k*a)Exp(k*a)^T has spatial angular vector a*dk.
            dk=np.einsum('ni,nij->nj',grad,j)
            b=a[None,:,None]*dk[:,None,:]+e@b;g=e@g
        return tuple(v[0] if single else v for v in [y,g,j,b])

    def forward(self,x,r):
        y,g,_,_=self.fields(x);return y,g@rotations(r)

    def inverse(self,y,r):
        x=np.asarray(y,float).copy()
        for layer in reversed(self.layers):
            if layer.translation is not None:x=Translation(**layer.translation).inverse(x)
        x=(x-self.offset)@self.axes/self.scale
        g=self.fields(x)[1]
        return x,np.swapaxes(g,-1,-2)@rotations(r)

    def velocity(self,x,v,omega):
        _,g,j,b=self.fields(x)
        return j@np.asarray(v),b@np.asarray(v)+g@np.asarray(omega)

    def global_bounds(self):
        lip=self.scale;angular=0.;inverse=1/self.scale
        for layer in self.layers:
            if layer.translation is not None:
                q=Translation(**layer.translation).q;lip*=1+q;inverse/=1-q
            angular+=np.linalg.norm(layer.rotation_vector)*np.sqrt(2/np.e)*layer.rotation_rho*lip
        return dict(position_lipschitz=float(lip),inverse_position_lipschitz=float(inverse),
            orientation_position_lipschitz_rad_per_unit=float(angular),
            scope='Global conservative product/sum bounds, not tight or interval arithmetic')

    def specification(self):return dict(initial=self.initial,layers=[asdict(x) for x in self.layers])
    @classmethod
    def restore(cls,s):return cls(s['initial'],[PoseLayer(**x) for x in s['layers']])


def fit_paired_pose(x,y,rh,rr,*,iterations=100,mu=.3,beta_position=.5,beta_rotation=1.,rho_rotation_bounds=(.001,6.)):
    """Gao greedy paired position+orientation updates after shared similarity.

    Paper toy hyperparameters used. Scalar solver bounded80 evaluations;
    orientation rho finite bracket is an explicit numerical reproduction limit.
    No robot-aware optimization, personal comfort fit or post-result tuning.
    """
    initial=fit_similarity(x,y,rh,rr);x=np.asarray(x,float);y=np.asarray(y,float);rh=rotations(rh);rr=rotations(rr)
    if not 0<mu<1 or not 0<beta_position<=1 or not 0<beta_rotation<=1 or not 0<rho_rotation_bounds[0]<rho_rotation_bounds[1]:raise ValueError('Bounded valid fit settings required')
    z,q=PairedPoseMap(initial).forward(x,rh);layers=[];trace=[]
    for i in range(iterations):
        errors=np.linalg.norm(y-z,axis=1);m=int(np.argmax(errors));translation=None
        if errors[m]>1e-10:
            center=z[m].copy();vector=beta_position*(y[m]-center)
            upper=mu*np.sqrt(np.e/2)/np.linalg.norm(vector)
            def cost(rho):return float(np.mean(np.linalg.norm(Translation(center.tolist(),vector.tolist(),float(rho)).apply(z)-y,axis=1)))
            sol=minimize_scalar(cost,bounds=(0,upper),method='bounded',options=dict(xatol=1e-10,maxiter=80))
            _,rho=min([(cost(0),0.),(cost(upper),upper),(float(sol.fun),float(sol.x))])
            t=Translation(center.tolist(),vector.tolist(),rho);translation=asdict(t);z=t.apply(z)
        angles=rot_error(q,rr);n=int(np.argmax(angles));c=z[n].copy();a=np.zeros(3);rho=0.
        if angles[n]>1e-10:
            # Correct LEFT increment: target current^-1, not its inverse.
            a=beta_rotation*Rotation.from_matrix(rr[n]@q[n].T).as_rotvec()
            def rcost(r):
                k=np.exp(-r*r*np.sum((z-c)**2,axis=1));mapped=Rotation.from_rotvec(k[:,None]*a).as_matrix()@q
                return float(np.mean(rot_error(mapped,rr)))
            lo,hi=rho_rotation_bounds;sol=minimize_scalar(rcost,bounds=(lo,hi),method='bounded',options=dict(xatol=1e-10,maxiter=80))
            _,rho=min([(rcost(lo),lo),(rcost(hi),hi),(float(sol.fun),float(sol.x))])
            q=Rotation.from_rotvec(np.exp(-rho*rho*np.sum((z-c)**2,axis=1))[:,None]*a).as_matrix()@q
        if translation is None and np.linalg.norm(a)==0:break
        layers.append(PoseLayer(translation,c.tolist(),a.tolist(),float(rho)))
        trace.append(dict(iteration=i,position_max=float(np.max(np.linalg.norm(z-y,axis=1))),rotation_max_rad=float(np.max(rot_error(q,rr))),rotation_rho=float(rho)))
    return PairedPoseMap(initial,layers),trace
