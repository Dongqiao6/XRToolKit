"""Independent position-only implementation of Gao et al. RAL2021, eqs 7/10.

Source: https://calinon.ch/papers/Gao-RAL2021.pdf (author-hosted arXiv v1).
No upstream source copied, imported, executed, or relicensed. No NN or sensing.
Each Gaussian translation has displacement Lipschitz constant q < 1. Thus
phi(x)=x+d(x) is globally bijective by Banach's theorem applied to x=y-d(x),
and its nonsingular analytic derivative gives a smooth inverse. Composition
with a positive similarity retains this property; sampled determinants alone
are never used as its proof. Coordinates must use one consistent length unit.
"""
from dataclasses import dataclass, asdict
import numpy as np
from scipy.optimize import minimize_scalar


@dataclass(frozen=True)
class Translation:
    center: list
    vector: list
    rho: float

    def __post_init__(self):
        c,v=np.asarray(self.center,float),np.asarray(self.vector,float)
        if c.ndim!=1 or c.shape!=v.shape or c.size not in (2,3) or not np.isfinite([*c,*v,self.rho]).all() or self.rho<0:
            raise ValueError('Finite 2D/3D Gaussian translation required')
        if self.q>=1:raise ValueError('Global contraction sufficient condition violated')

    @property
    def q(self):return float(np.sqrt(2/np.e)*self.rho*np.linalg.norm(self.vector))

    def apply(self,x):
        z=np.asarray(x,float)-self.center
        return np.asarray(x,float)+np.exp(-self.rho**2*np.sum(z*z,axis=-1))[...,None]*self.vector

    def jacobian(self,x):
        z=np.asarray(x,float)-self.center
        g=-2*self.rho**2*np.exp(-self.rho**2*np.sum(z*z,axis=-1))[...,None]*z
        return np.eye(len(self.center))+np.asarray(self.vector)[...,None]*g[...,None,:]

    def inverse(self,y,tol=1e-12,maxiter=300):
        y=np.asarray(y,float);x=y.copy()
        for _ in range(maxiter):
            nxt=y-(self.apply(x)-x)
            # A posteriori fixed-point error bound, not just the last step.
            error=np.max(np.linalg.norm(nxt-x,axis=-1))*self.q/(1-self.q)
            x=nxt
            if error<=tol:return x
        raise RuntimeError('Bounded inverse iteration did not converge')


class PositionMap:
    def __init__(self,dimension,layers=(),scale=1.,offset=None):
        if dimension not in (2,3) or not np.isfinite(scale) or scale<=0:raise ValueError('Positive similarity required')
        self.dimension=dimension;self.layers=tuple(layers);self.scale=float(scale)
        self.offset=np.zeros(dimension) if offset is None else np.asarray(offset,float)
        if self.offset.shape!=(dimension,) or not np.isfinite(self.offset).all():raise ValueError('Invalid offset')
        if any(len(x.center)!=dimension for x in self.layers):raise ValueError('Mixed dimensions')

    def _input(self,x):
        x=np.asarray(x,float)
        if x.ndim not in (1,2) or x.shape[-1]!=self.dimension or not np.isfinite(x).all():raise ValueError('Finite position required; missing input is not a point')
        return x

    def forward(self,x,jacobian=False):
        x=self._input(x);y=self.scale*x+self.offset
        j=np.broadcast_to(self.scale*np.eye(self.dimension),x.shape[:-1]+(self.dimension,self.dimension)).copy()
        for layer in self.layers:
            if jacobian:j=layer.jacobian(y)@j
            y=layer.apply(y)
        return (y,j) if jacobian else y

    def inverse(self,y):
        x=self._input(y).copy()
        for layer in self.layers[::-1]:x=layer.inverse(x)
        return (x-self.offset)/self.scale

    def ball_gain_bound(self,x,radius):
        """Upper bound on ||DF|| throughout closed input balls, not pointwise J.

        ||D² Gaussian|| <= 2 rho² gives a layer Jacobian Lipschitz bound.
        Propagate ball radii and Jacobian deviation about each center. The
        resulting integral bound covers any finite perturbation within the ball.
        Conservative floating-point evaluation; not interval arithmetic proof.
        """
        x=self._input(x);y=self.scale*x+self.offset
        j=np.broadcast_to(self.scale*np.eye(self.dimension),x.shape[:-1]+(self.dimension,self.dimension)).copy()
        r=np.broadcast_to(float(radius)*self.scale,x.shape[:-1]).copy();err=np.zeros_like(r)
        if not np.isfinite(radius) or radius<0:raise ValueError('Nonnegative finite radius required')
        for layer in self.layers:
            a=layer.jacobian(y);local=np.linalg.norm(a,ord=2,axis=(-2,-1))
            variation=2*np.linalg.norm(layer.vector)*layer.rho**2*r
            lip=np.minimum(1+layer.q,local+variation)
            err=lip*err+variation*np.linalg.norm(j,ord=2,axis=(-2,-1))
            j=a@j;y=layer.apply(y);r=lip*r
        return np.linalg.norm(j,ord=2,axis=(-2,-1))+err

    def specification(self):
        return dict(dimension=self.dimension,scale=self.scale,offset=self.offset.tolist(),layers=[asdict(x) for x in self.layers],
                    sufficient_condition='all displacement Lipschitz q < 1; global bijection and smooth inverse',q=[x.q for x in self.layers])

    @classmethod
    def restore(cls,s):return cls(s['dimension'],[Translation(**x) for x in s['layers']],s['scale'],s['offset'])


def fit_gao(source,target,*,iterations=100,mu=.6,beta=.9,initial_scale=1.,objective='norm',rho_bound_on_raw=True):
    """Paper IV positional greedy iteration, independent numerical implementation.

    Paper Fp and public 2D code both use mean Euclidean norm. Code beta=.9,
    with rho cap based on unscaled displacement. Both profiles explicit.
    Bounded scalar minimization replaces legacy two-variable fixed-beta solver.
    """
    x=np.asarray(source,float);y=np.asarray(target,float)
    if x.shape!=y.shape or x.ndim!=2 or not np.isfinite([x,y]).all():raise ValueError('Finite matched landmarks required')
    if not 0<mu<1 or not 0<beta<=1 or objective not in ('squared','norm'):raise ValueError('Invalid fit settings')
    z=initial_scale*x;layers=[];trace=[]
    for i in range(iterations):
        errors=np.linalg.norm(y-z,axis=1);m=int(np.argmax(errors));raw=y[m]-z[m]
        if errors[m]<1e-10:break
        v=beta*raw;center=z[m].copy();norm=np.linalg.norm(raw if rho_bound_on_raw else v)
        upper=mu*np.sqrt(np.e/2)/norm
        def cost(rho):
            difference=z+np.exp(-rho**2*np.sum((z-center)**2,axis=1))[:,None]*v-y
            values=np.linalg.norm(difference,axis=1)
            return float(np.mean(values**2 if objective=='squared' else values))
        solution=minimize_scalar(cost,bounds=(0,upper),method='bounded',options=dict(xatol=1e-10,maxiter=80))
        candidates=[(cost(0.),0.),(cost(upper),upper),(float(solution.fun),float(solution.x))]
        value,rho=min(candidates);layer=Translation(center.tolist(),v.tolist(),rho)
        layers.append(layer);z=layer.apply(z)
        trace.append(dict(iteration=i,max_before=float(errors[m]),objective_after=value,rho=rho,q=layer.q,scalar_success=bool(solution.success)))
    return PositionMap(x.shape[1],layers,initial_scale),trace
