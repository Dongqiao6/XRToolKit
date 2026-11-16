"""Recorded RealSense RGB to official UmeTrack inputs, without changing the network.

MediaPipe supplies only crop extent, never the output orientation or 3D skeleton.
Generic upstream skeleton is a prior, not a measured user hand. No cameras opened.
"""
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
UPSTREAM=ROOT/'external/UmeTrack-e263ec0844abf73457468a050a75853444d33d69'


def make_crop(image, points, intrinsic, distortion, camera_to_world, size=96, focal=.95, camera_angle=0):
    from lib.common.camera import PinholePlaneCameraModel
    from lib.common.crop import gen_crop_parameters_from_points
    p=np.asarray(points,float)
    if p.shape!=(1,21,2) or not np.isfinite(p).all():
        raise ValueError('single_complete_hand_crop_required')
    p=p[0]
    if (p<0).any() or (p[:,0]>=image.shape[1]).any() or (p[:,1]>=image.shape[0]).any():
        raise ValueError('hand_crop_outside_image')
    k=np.asarray(intrinsic,float);d=np.asarray(distortion,float)
    rays=np.c_[cv2.undistortPoints(p.reshape(-1,1,2),k,d).reshape(-1,2),np.ones(21)]
    camera=PinholePlaneCameraModel(image.shape[1],image.shape[0],[k[0,0],k[1,1]],k[:2,2],[],camera_to_world)
    # Unit-depth rays only specify image crop extent; they are NOT hand 3D input.
    crop=gen_crop_parameters_from_points(camera,camera.eye_to_world(rays),(size,size),
                                         mirror_img_x=True,camera_angle=camera_angle,focal_multiplier=focal)
    v,u=np.indices((size,size));pixels=np.c_[u.ravel(),v.ravel()]
    world=crop.eye_to_world(crop.window_to_eye(pixels))
    eye=camera.world_to_eye(world)
    maps=cv2.projectPoints(eye,np.zeros(3),np.zeros(3),k,d)[0].reshape(size,size,2).astype(np.float32)
    maps[eye[:,2].reshape(size,size)<=0]=-1
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    patch=cv2.remap(gray,maps[:,:,0],maps[:,:,1],cv2.INTER_LINEAR)
    return patch,crop.uv_to_window_matrix(),np.linalg.inv(crop.camera_to_world_xf)


class UmeTrackAdapter:
    def __init__(self, calibration, settings, device='cuda'):
        sys.path.insert(0,str(UPSTREAM))
        import torch
        from lib.models.model_loader import load_pretrained_model
        from lib.models.umetrack_model import InputSkeletonData
        self.torch=torch;self.settings=settings;self.calibration=calibration;self.device=device
        if settings['hand_index']!=1:
            raise ValueError('This adapter is explicitly for the confirmed right hand')
        weights=UPSTREAM/'pretrained_models/pretrained_weights.torch'
        skeleton_path=UPSTREAM/'dataset/generic_hand_model.json'
        for path,key in ((weights,'weights_sha256'),(skeleton_path,'skeleton_sha256')):
            if hashlib.sha256(path.read_bytes()).hexdigest()!=settings[key]:
                raise ValueError('Upstream asset checksum mismatch')
        torch.set_num_threads(settings['torch_cpu_threads'])
        self.model=load_pretrained_model(str(weights)).to(device).eval()
        skeleton=json.loads(skeleton_path.read_text())
        # Upstream generic skeleton JSON uses millimetres; network uses metres.
        self.skeleton=InputSkeletonData(torch.tensor(skeleton['joint_rotation_axes'],dtype=torch.float32,device=device),
                                       torch.tensor(skeleton['joint_rest_positions'],dtype=torch.float32,device=device)*.001)
        self.last_time=None;self.memory=False

    def predict(self, views, timestamp):
        from lib.models.umetrack_model import InputFrameData,InputFrameDesc
        t=self.torch
        if self.last_time is not None and timestamp<=self.last_time:
            self.memory=False
            return dict(valid=False,reason='nonmonotonic_recorded_time')
        memory=self.memory and self.last_time is not None and timestamp-self.last_time<=self.settings['stale_after_s']
        self.last_time=timestamp
        patches=[];intrinsics=[];extrinsics=[];roles=[];reasons={}
        for role,image,hand in views:
            side=0 if role=='left' else 1
            eye_to_world=np.eye(4)
            if side:
                eye_to_world[:3,:3]=np.asarray(self.calibration['R']).T
                eye_to_world[:3,3]=-eye_to_world[:3,:3]@np.asarray(self.calibration['T']).reshape(3)
            try:
                patch,k,xf=make_crop(image,hand.get('points',[]),self.calibration['K1' if not side else 'K2'],
                                     self.calibration['D1' if not side else 'D2'],eye_to_world,
                                     self.settings['crop_size'],self.settings['crop_focal_multiplier'],
                                     self.settings['camera_angles_degrees'][side])
            except ValueError as exc:
                reasons[role]=str(exc);continue
            patches.append(patch.astype(np.float32)/255);intrinsics.append(k);extrinsics.append(xf);roles.append(role)
        if not patches:
            self.memory=False
            return dict(valid=False,reason='no_current_hand_crop',crop_reasons=reasons)
        tensor=lambda x,dtype=t.float32:t.tensor(np.asarray(x),dtype=dtype,device=self.device)
        data=InputFrameData(tensor(patches),tensor(intrinsics),tensor(extrinsics))
        desc=InputFrameDesc(tensor([[0,len(patches)]],t.long),tensor([1],t.long),
                            tensor([memory and self.settings['enable_memory']],t.bool),tensor([1],t.long))
        with t.inference_mode():
            output=self.model.regress_pose_use_skeleton(data,desc,self.skeleton)
        xf=output.wrist_xfs[0].cpu().numpy().astype(float)
        r=xf[:3,:3]
        finite=bool(np.isfinite(xf).all() and np.allclose(r.T@r,np.eye(3),atol=1e-4) and np.linalg.det(r)>.999)
        self.memory=finite
        return dict(valid=finite,reason='numerical_prediction_not_accuracy_verified' if finite else 'invalid_predicted_transform',
                    rotation=r.tolist() if finite else None,wrist_transform_m=xf.tolist(),
                    joint_angles=output.joint_angles[0].cpu().tolist(),views=roles,crop_reasons=reasons,
                    used_memory=bool(memory and self.settings['enable_memory']),
                    source='official_umetrack_generic_skeleton',anatomical_accuracy_verified=False)
