"""Large task close view and an interactive robot overview; rendering only."""
from pathlib import Path
import json
import cv2,mujoco,numpy as np
from PIL import Image,ImageDraw
from .task_demo import markers


class TaskViews:
    """Owns only virtual cameras; receives no command state or live MjData."""
    overview_rect=(800,80,1280,404)
    preference_path=Path(__file__).resolve().parents[1]/'results/task_teaching/view_preferences.json'

    def __init__(self,model,preference_path=None):
        self.close=[]
        for azimuth in [90,0]:
            camera=mujoco.MjvCamera();mujoco.mjv_defaultFreeCamera(model,camera)
            camera.lookat[:]=[.44,.04,1.45]
            camera.distance=.45;camera.azimuth=azimuth;camera.elevation=0
            camera.orthographic=True;self.close.append(camera)
        self.overview=mujoco.MjvCamera();mujoco.mjv_defaultFreeCamera(model,self.overview)
        self.overview.lookat[:]=[.13,0,1.33]
        self.mode=0;self.drag=None;self.dirty=False;self.save_error=None
        self.path=Path(preference_path) if preference_path is not None else self.preference_path
        self.reset()
        if self.path.exists():
            try:
                saved=json.loads(self.path.read_text(encoding='utf8'))
                a,e,d=[float(saved[k]) for k in ['azimuth','elevation','distance']]
                if saved.get('schema_version')!=1 or saved.get('close_view') not in [0,1] or not np.isfinite([a,e,d]).all() or not (-80<=e<=30 and .8<=d<=3.):
                    raise ValueError('Invalid saved display camera')
                self.overview.azimuth=a%360;self.overview.elevation=e;self.overview.distance=d;self.mode=saved['close_view']
            except (OSError,ValueError,KeyError,TypeError):
                self.save_error='已恢复默认视角'
        self.dirty=False

    def reset(self):
        self.overview.azimuth=125;self.overview.elevation=-20;self.overview.distance=1.7
        self.drag=None;self.dirty=True

    def state(self):
        return dict(schema_version=1,close_view=self.mode,azimuth=float(self.overview.azimuth),
                    elevation=float(self.overview.elevation),distance=float(self.overview.distance))

    def save(self):
        if not self.dirty or self.drag is not None:return
        try:
            self.path.parent.mkdir(parents=True,exist_ok=True)
            temporary=self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps(self.state(),indent=2),encoding='utf8')
            temporary.replace(self.path);self.save_error=None
        except OSError:self.save_error='视角本次有效，未能保存'
        self.dirty=False

    def key(self,key):
        if key in [ord('1'),ord('2')]:self.mode=key-ord('1')
        elif key in [ord('j'),ord('J')]:self.overview.azimuth=(self.overview.azimuth-10)%360
        elif key in [ord('l'),ord('L')]:self.overview.azimuth=(self.overview.azimuth+10)%360
        elif key in [ord('r'),ord('R')]:self.reset()
        else:return False
        self.dirty=True;return True

    def mouse(self,event,x,y,flags,param=None):
        x0,y0,x1,y1=self.overview_rect;inside=x0<=x<x1 and y0<=y<y1
        if event==cv2.EVENT_LBUTTONUP:self.drag=None;return
        if event==cv2.EVENT_LBUTTONDOWN:
            if inside:self.drag=(x,y)
            elif 804<=x<1276 and 414<=y<450:
                self.key(ord('j') if x<962 else ord('r') if x<1120 else ord('l'))
            return
        if event==cv2.EVENT_MOUSEMOVE and self.drag is not None:
            if not flags&cv2.EVENT_FLAG_LBUTTON:self.drag=None;return
            if inside:
                oldx,oldy=self.drag
                self.overview.azimuth=(self.overview.azimuth-(x-oldx)*.5)%360
                self.overview.elevation=float(np.clip(self.overview.elevation+(y-oldy)*.35,-80,30))
                self.dirty=True
            self.drag=(x,y)
        if event==cv2.EVENT_MOUSEWHEEL and inside:
            delta=(flags>>16)&0xffff
            if delta>=32768:delta-=65536
            if delta:
                self.overview.distance=float(np.clip(self.overview.distance*np.exp(-.12*delta/120),.8,3.))
                self.dirty=True


def cameras(model,preference_path=None):
    return TaskViews(model,preference_path)


def frame(renderer,data,views,frames,condition,heading,input_status,robot,mount,
          elapsed,age,latest,robot_error,font,small):
    """Two renders of one copied state; preserve aspect ratio in both panes."""
    canvas=np.zeros((900,1280,3),np.uint8)
    site=data.site('attachment_site')
    for camera,rect in [(views.close[views.mode],(0,80,800,620)),(views.overview,views.overview_rect)]:
        renderer.update_scene(data,camera)
        markers(renderer.scene,site.xpos,site.xmat.reshape(3,3),condition)
        x0,y0,x1,y1=rect
        canvas[y0:y1,x0:x1]=cv2.resize(renderer.render(),(x1-x0,y1-y0))
    for column,role in enumerate(['left','right']):
        if frames[role] is not None:
            thumbnail=cv2.resize(frames[role].image,(256,144))
            canvas[654:798,12+column*276:268+column*276]=cv2.cvtColor(thumbnail,cv2.COLOR_BGR2RGB)
    image=Image.fromarray(canvas);draw=ImageDraw.Draw(image)
    mode='A：位置、指向、正面都需要对齐' if condition=='A' else 'B：位置、指向需要对齐；绕轴转角不要求'
    draw.text((14,10),mode,font=font,fill='white')
    draw.text((14,45),'蓝色点／箭头：机器人当前状态    绿色：目标参照    '+('橙色片：工具正面' if condition=='A' else '灰色片：不受任务约束的正面'),font=small,fill='white')
    draw.rectangle((0,80,800,112),fill='#15202c')
    draw.text((14,84),('末端正视 · X–Z' if views.mode==0 else '末端侧视 · Y–Z')+'｜1 正视 / 2 侧视',font=small,fill='white')
    draw.rectangle((800,80,1280,112),fill='#15202c')
    draw.text((814,84),'整臂3D｜拖动旋转 · 滚轮缩放',font=small,fill='white')
    for x,label in [(804,'J 向左转'),(962,'R 复位'),(1120,'L 向右转')]:
        draw.rounded_rectangle((x,414,x+154,450),radius=5,fill='#24384c')
        draw.text((x+18,418),label,font=small,fill='white')
    draw.text((816,471),'末端为主，整臂辅助观察构型',font=small,fill='#b9c2ce')
    draw.text((816,505),'视角不改变手到机器人的映射',font=small,fill='#b9c2ce')
    draw.text((816,539),views.save_error or '视角自动保存，下次沿用',font=small,fill='#b9c2ce')
    draw.line((800,80,800,620),fill='#82909e',width=2)
    draw.text((12,626),'左相机',font=small,fill='#b9c2ce');draw.text((288,626),'右相机',font=small,fill='#b9c2ce')
    color='#ff9b81' if robot_error else 'white'
    draw.text((566,630),heading,font=font,fill=color)
    draw.text((566,670),input_status,font=small,fill='white')
    draw.text((566,702),f'{robot}｜安装Y {mount}°｜观测延迟 '+('--' if age is None else f'{age:.0f}ms'),font=small,fill='#b9c2ce')
    if latest and latest.get('task'):
        goal=latest['task']['final_reference']
        text=f'距参照：位置 {goal["position_mm"]:.0f}mm｜指向 {goal["axis_deg"]:.1f}°'
        if condition=='A':text+=f'｜完整朝向 {goal["full_pose_deg"]:.1f}°'
        draw.text((566,734),text,font=small,fill='white')
        draw.text((566,766),f'记录 {elapsed:.1f}s｜墙钟−模型 {latest["wall_minus_simulation_s"]:.2f}s（诊断信息）',font=small,fill='#b9c2ce')
    draw.text((14,821),'空格：暂停回位／继续接着调    Q / Esc / 关闭窗口：结束并保存',font=font,fill='white')
    draw.text((14,861),'暂停后手可回到舒适位置和姿势，继续时重接位置与姿态。暂停仍按原限制减速；执行保护后停止联动。',font=small,fill='#b9c2ce')
    return cv2.cvtColor(np.asarray(image),cv2.COLOR_RGB2BGR)
