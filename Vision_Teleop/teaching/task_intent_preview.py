"""Optional task display overlay: requested hand pose vs actual robot pose.

No model geoms, state mutation, control, input validity or mapping changes.
Missing data retains the last jointly fresh display pose in grey, explicitly
labelled stale. That display cache is never returned to any controller.
"""
import numpy as np
import mujoco,cv2
from PIL import Image,ImageDraw


class IntentPreview:
    def __init__(self):
        self.cached=None
        self.fresh=False

    def observe(self,latest):
        self.fresh=False
        if latest is None:return
        s=latest['sample']
        if not (s.get('active') and s.get('position_fresh') and s.get('pose_fresh')):return
        p=np.asarray(s.get('raw_p'),float);r=np.asarray(s.get('raw_r'),float)
        if p.shape!=(3,) or r.shape!=(3,3) or not np.isfinite(p).all() or not np.isfinite(r).all():return
        self.cached=(p.copy(),r.copy());self.fresh=True

    def markers(self,scene):
        if self.cached is None:return
        p,r=self.cached;color=[1.,.25,.85,.95] if self.fresh else [.65,.65,.65,.55]
        def connector(a,b,width=.002,kind=mujoco.mjtGeom.mjGEOM_LINE):
            if scene.ngeom>=scene.maxgeom:raise RuntimeError('Intent preview render capacity exhausted')
            g=scene.geoms[scene.ngeom];scene.ngeom+=1
            mujoco.mjv_initGeom(g,kind,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array(color,np.float32))
            mujoco.mjv_connector(g,kind,width,a,b)
        connector(p,p+.10*r[:,2],.003,mujoco.mjtGeom.mjGEOM_ARROW)
        center=p+.06*r[:,2]+.024*r[:,0]
        corners=[center+sx*.025*r[:,0]+sy*.011*r[:,1] for sx,sy in [(-1,-1),(1,-1),(1,1),(-1,1)]]
        for a,b in zip(corners,corners[1:]+corners[:1]):connector(a,b,2.)
        for axis in r.T:connector(p-.008*axis,p+.008*axis,2.)

    def install(self):
        """Attach only to existing task rendering, with an explicit undo hook."""
        from teaching import task_display
        old_frame,old_markers=task_display.frame,task_display.markers
        def markers(scene,p,r,condition):
            old_markers(scene,p,r,condition);self.markers(scene)
        def frame(renderer,data,views,frames,condition,heading,input_status,robot,mount,elapsed,age,latest,robot_error,font,small):
            self.observe(latest)
            result=old_frame(renderer,data,views,frames,condition,heading,input_status,robot,mount,elapsed,age,latest,robot_error,font,small)
            im=Image.fromarray(cv2.cvtColor(result,cv2.COLOR_BGR2RGB));d=ImageDraw.Draw(im)
            d.rectangle((0,40,1280,79),fill='black')
            d.text((14,45),'蓝：实际末端    绿：任务参照    橙：实际正面    紫色细框：手表达的要求（不跟踪瓶子）',font=small,fill='white')
            d.rectangle((0,851,1280,899),fill='black')
            message='先看紫色方向是否合意，再看蓝色是否跟上；空格暂停回位，Q结束。' if self.fresh else '灰色是末次有效要求，当前输入缺失、过期或已暂停；不把灰色当成实时手姿。'
            d.text((14,861),message,font=small,fill='#ff72d7' if self.fresh else '#b9c2ce')
            return cv2.cvtColor(np.asarray(im),cv2.COLOR_RGB2BGR)
        task_display.markers=markers;task_display.frame=frame
        def restore():task_display.frame=old_frame;task_display.markers=old_markers
        return restore
