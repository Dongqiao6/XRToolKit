"""Image-evidence veto for palm label inversions; never creates a pose."""
import cv2
import numpy as np


def signed_area(points):
    wrist,index,middle,pinky=points
    return float(np.linalg.det(np.stack((index-pinky,middle-wrist))))


class PalmFlowGuard:
    def __init__(self, tolerance_px, stale_after_s):
        self.tolerance=float(tolerance_px)
        self.stale=float(stale_after_s)
        self.gray=self.points=None
        self.timestamp=self.confirmed=None

    def observe(self, image, hand, timestamp):
        p=np.asarray(hand.get('points',[]),float)
        if not np.isfinite(timestamp) or (self.timestamp is not None and timestamp<=self.timestamp):
            return dict(valid=False,reason='flow_invalid_timestamp')
        if p.shape!=(1,21,2) or not np.isfinite(p).all():
            self.gray=self.points=None
            return dict(valid=False,reason='flow_missing_palm')
        p=p[0,[0,5,9,17]].astype(np.float32)
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        old,oldp=self.gray,self.points
        expired=self.confirmed is None or timestamp-self.confirmed>self.stale
        self.gray,self.timestamp=gray,timestamp
        if old is None or expired:
            self.points,self.confirmed=p,timestamp
            return dict(valid=False,reason='flow_reacquiring')
        tracked,status,_=cv2.calcOpticalFlowPyrLK(old,gray,oldp.reshape(-1,1,2),None)
        if tracked is None:
            self.points,self.confirmed=p,timestamp
            return dict(valid=False,reason='flow_failed')
        back,back_status,_=cv2.calcOpticalFlowPyrLK(gray,old,tracked,None)
        fb=float(np.max(np.linalg.norm(back.reshape(-1,2)-oldp,axis=1))) if back is not None else float('inf')
        if back is None or not status.all() or not back_status.all() or not np.isfinite(tracked).all() or not np.isfinite(fb) or fb>self.tolerance:
            self.points,self.confirmed=p,timestamp
            return dict(valid=False,reason='flow_unverified')
        tracked=tracked.reshape(-1,2)
        a,b=signed_area(tracked),signed_area(p)
        details=dict(tracked_area_px2=a,detected_area_px2=b,forward_backward_px=fb)
        if min(abs(a),abs(b))<=self.tolerance**2:
            self.points,self.confirmed=p,timestamp
            return dict(valid=False,reason='flow_edge_on',**details)
        if a*b<0:
            self.points=tracked
            return dict(valid=False,reason='flow_semantic_inversion',**details)
        self.points,self.confirmed=p,timestamp
        return dict(valid=True,reason='flow_consistent_not_semantic_truth',**details)
