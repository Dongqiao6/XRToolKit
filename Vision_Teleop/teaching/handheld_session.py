"""One bounded empty/held input check before the unchanged task-A controller."""
import hashlib,json
from pathlib import Path
import cv2,numpy as np
from PIL import Image,ImageDraw

ROOT=Path(__file__).resolve().parents[1]

def settings():
    return json.loads((ROOT/'config/wilor_presentation.json').read_text(encoding='utf8'))['handheld_demo']

def write_json(path,value):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf8');tmp.replace(path)

class HandheldSession:
    def __init__(self,folder,smoke=False):
        self.folder=Path(folder);self.config=settings();self.smoke=smoke
        self.stage='empty_ready';self.stage_start=None;self.epoch=None;self.last_write=-1
        self.rows=[];self.events=[];self.review=None;self.blocked=False;self.last_images={}
        self.check_seconds=2.5 if smoke else self.config['input_segment_s']
        self.test_actions=set()

    @property
    def execution_enabled(self):return self.stage=='task' and not self.blocked

    def change(self,stage,now):
        self.stage=stage;self.stage_start=now
        self.events.append(dict(type='handheld_stage',stage=stage,clock_s=now,software_smoke=self.smoke))

    def key(self,key,shared,events,now):
        """Return true if this key belongs to the pre-control session protocol."""
        if key==ord('x'):
            self.blocked=True;self.change('stopped',now)
            shared.update(active=False,request_start=False,user_stopped=True)
            events.append(dict(type='input_issue_stop',clock_s=now,reason='operator_or_reviewer_observed_input_problem',physical_braking_claim=False))
            return True
        if self.blocked:return key==32
        if key!=32:return False
        if self.stage=='empty_ready':self.change('empty',now);return True
        if self.stage=='held_ready':self.change('held',now);return True
        if self.stage in ['empty','held','review']:return True
        if self.stage=='task_ready':self.change('task',now);return False
        return False

    def observe(self,frames,pose,point,now,index):
        channels={}
        for role,f in frames.items():
            h=f.metadata.get('hand',{}) if f else {};selection=h.get('selection',{})
            channels[role]=dict(frame_id=f.identity if f else None,source_s=f.time_ms/1000 if f else None,
                age_ms=(now-f.time_ms/1000)*1000 if f else None,selection=selection,
                selected_points=h.get('points',[]),depth=h.get('rgbd_position'))
        t=None if self.stage_start is None else now-self.stage_start
        row=dict(index=index,clock_s=now,stage=self.stage,stage_elapsed_s=t,
            cue=self.cue(now),channels=channels,position_valid=point is not None,
            position_camera_m=None if point is None else np.asarray(point).tolist(),
            pose_valid=bool(pose and pose['valid'] and 0<=now-pose['stamp']<=.15),
            pose_source_s=pose['stamp'] if pose else None,pose_age_ms=(now-pose['stamp'])*1000 if pose else None,
            pose_camera_rotation=pose['rotation'] if pose else None,pose_reason=pose['reason'] if pose else 'no_result',
            execution_enabled=self.execution_enabled)
        self.rows.append(row)

    def cue(self,now):
        if self.stage not in ['empty','held']:return 'task' if self.stage=='task' else 'waiting'
        fraction=(now-self.stage_start)/self.check_seconds
        for end,name in [(.2,'still'),(.45,'translate'),(.7,'tilt'),(.9,'roll'),(1.01,'align')]:
            if fraction<end:return name
        return 'align'

    def tick(self,now):
        if self.epoch is None:self.epoch=now
        stop=self.folder/'input_stop_request.json'
        if stop.exists() and not self.blocked:
            self.events.append(dict(type='reviewer_input_stop',clock_s=now,detail=json.loads(stop.read_text(encoding='utf8'))))
            self.blocked=True;self.change('stopped',now)
        if self.stage in ['empty','held'] and now-self.stage_start>=self.check_seconds:
            if self.stage=='empty':self.change('held_ready',now)
            else:
                self.change('review',now)
                from scripts.report_handheld import input_report
                self.review=input_report(self.rows)
                write_json(self.folder/'input_review.json',self.review)
                # The review transcript and source are immutable after this gate.
                self.review_hash=hashlib.sha256((self.folder/'input_review.json').read_bytes()).hexdigest()
        approval=self.folder/'input_review_decision.json'
        if self.stage=='review' and approval.exists():
            decision=json.loads(approval.read_text(encoding='utf8'))
            if decision.get('input_review_sha256')!=self.review_hash:raise ValueError('Review decision does not match this input evidence')
            if decision.get('decision')=='allow_task':
                if not decision.get('reviewed_images') or not decision.get('reason'):raise ValueError('Image review and reason required')
                if bool(decision.get('software_smoke',False))!=self.smoke:raise ValueError('Software-only decision cannot authorize physical input')
                self.events.append(dict(type='input_review_decision',clock_s=now,decision=decision))
                self.change('task_ready',now)
            elif decision.get('decision')=='stop':self.blocked=True;self.change('stopped',now)
            else:raise ValueError('Unknown review decision')

    def publish(self,now,frames,composite):
        if now-self.last_write<1:return
        self.last_write=now
        write_json(self.folder/'session_progress.json',dict(stage=self.stage,stage_elapsed_s=None if self.stage_start is None else now-self.stage_start,
            execution_enabled=self.execution_enabled,rows=len(self.rows),software_smoke=self.smoke,
            review_sha256=getattr(self,'review_hash',None),remaining_session_s=self.config['session_limit_s']-(now-self.epoch)))
        cv2.imwrite(str(self.folder/'current_view.jpg'),composite)
        if self.stage in ['empty','held']:
            cue=self.cue(now);tag=f'{self.stage}_{cue}'
            for role,f in frames.items():
                if f is not None:
                    # A few representative frames for review, full raw video is saved separately.
                    path=self.folder/f'review_{tag}_{role}.jpg'
                    cv2.imwrite(str(path),annotated(f));self.last_images[f'{tag}_{role}']=str(path.name)
            write_json(self.folder/'review_images.json',self.last_images)

    def expired(self,now):return self.epoch is not None and now-self.epoch>=self.config['session_limit_s']

    def finish(self):
        (self.folder/'handheld_observations.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.rows),encoding='utf8')
        write_json(self.folder/'handheld_session.json',dict(stage=self.stage,events=self.events,settings=self.config,
            software_smoke=self.smoke,blocked=self.blocked,object_tracking=False,reference_measurement='not_instrumented',
            task_execution_started=any(e.get('stage')=='task' for e in self.events)))

def annotated(frame):
    image=frame.image.copy();h=frame.metadata.get('hand',{})
    for points in h.get('all_points_before_selection',[]):
        for x,y in points:
            if np.isfinite([x,y]).all():cv2.circle(image,(round(x),round(y)),3,(80,80,255),1)
    for points in h.get('points',[]):
        for x,y in points:
            if np.isfinite([x,y]).all():cv2.circle(image,(round(x),round(y)),4,(80,255,80),-1)
    return image

def decorate(composite,session,frames,now,font,small):
    image=Image.fromarray(cv2.cvtColor(composite,cv2.COLOR_BGR2RGB));d=ImageDraw.Draw(image)
    check=session.stage!='task'
    if check:
        d.rectangle((0,80,1280,620),fill='#15202c')
        for col,role in enumerate(['left','right']):
            f=frames[role]
            if f is not None:
                thumb=cv2.cvtColor(cv2.resize(annotated(f),(640,360)),cv2.COLOR_BGR2RGB)
                image.paste(Image.fromarray(thumb),(col*640,160))
        d.text((16,105),'输入检查：机器人尚未联动｜绿色点是已接纳手部点，红色点是选择前检测',font=font,fill='white')
        d.text((16,545),'空手阶段自然伸出右手；握持阶段舒适握住小瓶，左手放下。',font=font,fill='white')
        d.rectangle((550,621,1280,800),fill='#15202c')
        running=session.stage in ['empty','held']
        note=('本片段剩余 '+str(max(0,int(np.ceil(session.check_seconds-(now-session.stage_start)))))+' 秒') if running else '尚未联动；按顶部提示操作'
        d.text((566,632),note,font=font,fill='white')
        d.text((566,672),'片段会自动结束，无需反复按空格' if running else 'X停止联动；Q结束并保存',font=small,fill='#b9c2ce')
        if session.rows:
            latest=session.rows[-1]
            d.text((566,712),'当前输入：位置'+('有效' if latest['position_valid'] else '缺失/过期')+'｜姿态'+('有效' if latest['pose_valid'] else '缺失/过期'),font=small,fill='white')
        d.text((566,752),'这是手部输入检查，不是瓶体位姿跟踪',font=small,fill='#ffe3a1')
    headings={'empty_ready':'空手检查｜右手入镜，空格开始30秒','empty':'空手输入检查',
        'held_ready':'拿起小瓶｜握得舒适后，空格开始30秒','held':'握持输入检查',
        'review':'两个片段已保存｜正在核查输入，机器人未联动',
        'task_ready':'输入核查完成｜手持小瓶，空格接入FR3任务A',
        'task':'手持示教｜移动、转向、对准绿色参照', 'stopped':'已停止联动并保留证据｜Q结束保存'}
    cues={'still':'自然停住片刻','translate':'自然平移','tilt':'舒适倾斜','roll':'舒适地绕轴转腕','align':'停住并小幅对准'}
    d.rectangle((0,0,1280,78),fill='#15202c');d.text((14,8),headings[session.stage],font=font,fill='white')
    cue=cues.get(session.cue(now),'完整姿态A：位置、指向、正面均保留')
    if session.stage=='empty' and session.cue(now)=='roll':cue='舒适地转掌、转腕'
    d.text((14,43),cue+'｜X：发现识别错误/跳变时停止联动｜Q/Esc：结束',font=small,fill='#ffe3a1')
    d.rectangle((0,800,1280,900),fill='#15202c')
    d.text((14,805),'小瓶只辅助稳定手形和直觉操作；自然握持即可，不需要贴点、画线或标记。',font=small,fill='white')
    d.text((14,835),'系统仍观察手，沿用原位置和姿态映射；不跟踪瓶子，不改变控制方式。',font=small,fill='#ffe3a1')
    d.text((14,865),'联动后空格暂停→手回舒适位置/姿势→空格继续；不自动换任务或重置实际状态。',font=small,fill='white')
    return cv2.cvtColor(np.asarray(image),cv2.COLOR_RGB2BGR)
