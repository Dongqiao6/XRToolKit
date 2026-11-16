"""Live-only target boundary and camera persistence for the native viewer."""
import itertools
import numpy as np
import mujoco


def workspace_edges(lower, upper):
    lower,upper=np.asarray(lower,float),np.asarray(upper,float)
    if lower.shape!=(3,) or upper.shape!=(3,) or not np.isfinite([lower,upper]).all() or np.any(lower>=upper):
        raise ValueError('Invalid workspace box')
    corners=np.array(list(itertools.product(*zip(lower,upper))))
    return [(a,b) for i,a in enumerate(corners) for b in corners[i+1:] if np.count_nonzero(a!=b)==1]


def draw_workspace(scene, lower, upper):
    for a,b in workspace_edges(lower,upper):
        if scene.ngeom>=scene.maxgeom: raise RuntimeError('Viewer geometry capacity exhausted')
        g=scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([1.,.65,.1,.65],np.float32))
        mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_CAPSULE,.002,a,b)
        scene.ngeom+=1


def camera_values(camera):
    return dict(lookat=camera.lookat.tolist(),distance=float(camera.distance),azimuth=float(camera.azimuth),elevation=float(camera.elevation))


def apply_camera(camera, values):
    camera.lookat[:]=values['lookat']
    for name in ('distance','azimuth','elevation'): setattr(camera,name,float(values[name]))
