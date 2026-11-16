"""User-defined camera-to-base directions and a single workspace gain."""
from collections import deque
import numpy as np


class DirectionSetup:
    prompts = (
        '1/7 右移起点：手放舒适位置，空格记录',
        '2/7 向希望机器人右移的方向移动，空格记录终点',
        '3/7 前移起点：手回舒适位置，空格记录',
        '4/7 向希望机器人前移的方向移动，空格记录终点',
        '5/7 上移起点：手回舒适位置，空格记录',
        '6/7 向希望机器人上移的方向移动，空格记录终点',
        '7/7 检查比例，空格确认（之后再归零开始）',
    )

    def __init__(self, robot_extent=.10):
        self.points=[]
        self.history=deque(maxlen=30)
        self.rotation=None
        self.scale=None
        self.lengths=None
        self.robot_extent=float(robot_extent)
        self.complete=False
        self.last_diagnostic={}

    @property
    def prompt(self):
        return '方向与比例已设好；空格开始' if self.complete else self.prompts[len(self.points)]

    def add(self, stamp, point):
        if point is None:
            self.history.clear()
        elif not self.history or stamp>self.history[-1][0]:
            self.history.append((stamp,np.asarray(point,float).copy()))

    def stable_point(self, now):
        recent=[(t,p) for t,p in self.history if 0<=now-t<=.5]
        if len(recent)<5 or recent[-1][0]-recent[0][0]<.25 or now-recent[-1][0]>.15:
            raise ValueError('需要有效位置，请停稳半秒再确认')
        points=np.array([p for _,p in recent])
        if not np.isfinite(points).all() or np.max(np.linalg.norm(points-points.mean(axis=0),axis=1))>.01:
            raise ValueError('手还在动，请在端点停稳半秒')
        return points.mean(axis=0)

    def confirm(self, now):
        self.last_diagnostic=dict(step=len(self.points)+1)
        if self.complete:
            return
        if len(self.points)==6:
            self.complete=True
            self.history.clear()
            return
        point=self.stable_point(now)
        candidate=self.points+[point]
        self.last_diagnostic=dict(step=len(candidate),candidate_point_m=point.tolist())
        if len(candidate)%2==0:
            length=float(np.linalg.norm(point-candidate[-2]))
            self.last_diagnostic['movement_m']=length
            if length<.10-1e-9:
                raise ValueError(f'本次移动仅{length*100:.1f}cm，需至少10cm；起点保留')
        if len(candidate)>=4:
            right=candidate[1]-candidate[0]
            forward=candidate[3]-candidate[2]
            right=right/np.linalg.norm(right)
            forward=forward/np.linalg.norm(forward)
            if abs(right@forward)>.5:
                raise ValueError('右移与前移方向过于接近，请重新示范前移')
            forward=forward-right*(right@forward)
            forward/=np.linalg.norm(forward)
            up=np.cross(right,forward)
            if len(candidate)==6:
                vertical=candidate[5]-candidate[4]
                projection=vertical@up
                lateral=float(np.linalg.norm(vertical-projection*up))
                self.last_diagnostic.update(up_projection_m=float(projection),lateral_m=lateral)
                if projection<=0:
                    raise ValueError('上移与右/前组成反向坐标；按B核对起点或前移方向')
                if projection<.10-1e-9:
                    raise ValueError(f'沿上轴仅{projection*100:.1f}cm，需10cm；本次起点保留')
                if lateral>projection*.5:
                    raise ValueError(f'偏轴{lateral*100:.1f}cm，上轴{projection*100:.1f}cm；B可重设起点')
                camera_basis=np.column_stack((right,forward,up))
                base_basis=np.array([[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]])
                self.rotation=base_basis@camera_basis.T
                self.lengths=np.array([np.linalg.norm(candidate[i+1]-candidate[i]) for i in (0,2,4)])
                self.scale=self.robot_extent/float(max(self.lengths))
        self.points=candidate
        self.history.clear()

    def adjust(self, delta):
        if len(self.points)!=6 or self.complete:
            return
        self.robot_extent=float(np.clip(round(self.robot_extent+delta,4),.05,.15))
        self.scale=self.robot_extent/float(max(self.lengths))

    def undo(self):
        if not self.complete and self.points:
            self.points.pop()
            self.rotation=self.scale=self.lengths=None
            self.history.clear()

    def result(self):
        if not self.complete:
            raise ValueError('Direction/scale setup not confirmed')
        return dict(camera_to_base_rotation=self.rotation.tolist(),position_scale=self.scale,
                    demonstrated_lengths_m=self.lengths.tolist(),robot_max_demonstrated_extent_m=self.robot_extent,
                    source='user demonstrated right/forward/up; one uniform gain',
                    limits='Directional setup, not camera calibration, full workspace capture or torso tracking')
