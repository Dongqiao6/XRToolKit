"""Equivalent frozen right-hand inference crop; unchanged global warp matrix.

Reuse the pinned author's dataset for metadata/box conventions and helpers.
Filter only source pixels that the output warp can sample, with full Gaussian
support halo. Keep original-image coordinates to avoid interpolation rounding
changes caused by translating the affine transform to a local cropped image.
"""
import cv2
import numpy as np
from teaching.wilor_crop import gaussian

def item(cfg, image, box):
    from wilor.datasets.vitdet_dataset import ViTDetDataset
    from wilor.datasets.utils import expand_to_aspect_ratio, gen_trans_from_patch_cv, convert_cvimg_to_tensor
    if image.dtype != np.uint8 or image.shape != (720,1280,3):
        raise ValueError('Frozen input shape/dtype required')
    if np.asarray(box).shape != (1,4) or not np.isfinite(box).all() or np.any(box[0,2:] <= box[0,:2]):
        raise ValueError('Exactly one valid current right-hand box required')
    ds=ViTDetDataset(cfg,image,box,np.ones(1),rescale_factor=2.)
    center=ds.center[0].copy()
    size=expand_to_aspect_ratio(ds.scale[0]*200,target_aspect_ratio=cfg.MODEL.get('BBOX_SHAPE',None)).max()
    out=int(ds.img_size)
    trans=gen_trans_from_patch_cv(center[0],center[1],size,size,out,out,1.,0.)
    factor=size/out/2
    source=image
    if factor > 1.1:
        sigma=(factor-1)/2;radius=int(4*sigma+.5)
        inverse=cv2.invertAffineTransform(trans)
        corners=np.array([[0,0,1],[out-1,0,1],[0,out-1,1],[out-1,out-1,1]])@inverse.T
        h,w=image.shape[:2]
        # Three pixels include bilinear support and OpenCV's fixed-point rounding.
        x0,y0=np.maximum(0,np.floor(corners.min(axis=0)).astype(int)-3)
        x1,y1=np.minimum([w,h],np.ceil(corners.max(axis=0)).astype(int)+4)
        source=np.zeros(image.shape,dtype=np.float64)
        if x1>x0 and y1>y0:
            hx0,hy0=max(0,x0-radius),max(0,y0-radius)
            hx1,hy1=min(w,x1+radius),min(h,y1+radius)
            crop=image[hy0:hy1,hx0:hx1]
            filtered=crop.astype(np.float64) if radius==0 else gaussian(crop,sigma,channel_axis=2,preserve_range=True)
            source[y0:y1,x0:x1]=filtered[y0-hy0:y1-hy0,x0-hx0:x1-hx0]
    patch=cv2.warpAffine(source,trans,(out,out),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=0)
    patch=convert_cvimg_to_tensor(patch[:,:,::-1])
    for c in range(3):patch[c]=(patch[c]-ds.mean[c])/ds.std[c]
    return dict(img=patch,personid=0,box_center=center,box_size=size,img_size=1.*np.array([image.shape[1],image.shape[0]]),right=ds.right[0].copy())
