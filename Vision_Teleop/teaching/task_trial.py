"""Timed instructions only; no access to control state or task targets."""
from copy import deepcopy
from PIL import Image,ImageDraw
import cv2
import numpy as np

STAGES=[
    dict(id='following',start_s=0,end_s=20,lines=['第一段：表达运动（20秒）','自然平移、倾斜、绕轴转腕','这一段观察跟随，不要求对准']),
    dict(id='alignment',start_s=20,end_s=65,lines=['第二段：对准绿色参照','对齐蓝点、箭头和橙色正面','到位后自然停住约2秒','没到位也保留，不必重做到通过']),
    dict(id='clutch',start_s=65,end_s=75,lines=['第三段：暂停—回位—继续','空格暂停，手回舒适位置/姿势','再按空格，继续同一任务']),
    dict(id='alignment_after_clutch',start_s=75,end_s=90,lines=['最后一段：继续对准并停住','沿用同一个绿色参照','自然停住约2秒；90秒自动结束']),
]


def protocol():
    return dict(version=1,duration_s=90,stages=deepcopy(STAGES),condition='A',robot='franka_fr3',mount_y_degrees=0,
        changes='Instruction overlay and extra display-stage log only. No changes to mapping, filters, targets, clutch, controls, safety or layout.',
        phase_clock='Starts at first display of an engaged common command. Save displayed source elapsed time; may follow actual first engagement by one display interval. Existing 90s control deadline is authoritative.',
        progression='Fixed source elapsed time, including pauses. Never advance or repeat because of success/failure. Ordinary user controls unchanged.',
        evaluation=dict(following='Requirement to common command, IK, issued and actual; valid-channel denominators, all missing/pause intervals.',
            alignment='Actual robot vs fixed world reference: original20mm/axis2deg/fullSO3 2deg/fresh0.5s, diagnostic only. Report coverage and longest span, no new absolute-accuracy claim.',
            raw_requirement='Parallel diagnostic, NOT a prerequisite for counting actual robot alignment; operator sees robot feedback.',
            unknown='Missing/paused input breaks fresh hold; retain unqualified actual holds separately.',
            hold_instruction_s=2,hold_diagnostic_s=.5,extra_tail_s=0),
        stop='One short acquisition; Q/Esc anytime; obvious wrong motion ends trial, all failures retained. No repeat-to-pass or coaching to shrink/slow input.')


class TrialInstructions:
    def __init__(self):
        self.origin=None
        self.rows=[]

    def observe(self,source_elapsed,latest):
        if self.origin is None and latest and latest.get('command',{}).get('engaged'):
            self.origin=float(source_elapsed)
        if self.origin is None:return None,0.
        elapsed=max(0.,float(source_elapsed)-self.origin)
        stage=next((s for s in STAGES if s['start_s']<=elapsed<s['end_s']),STAGES[-1])
        self.rows.append(dict(source_elapsed_s=float(source_elapsed),instruction_elapsed_s=elapsed,
            stage=stage['id'],control_index=latest['index'] if latest else None))
        return stage,elapsed

    def decorate(self,bgr,stage,elapsed,font):
        im=Image.fromarray(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB));d=ImageDraw.Draw(im)
        d.rectangle((808,462,1277,612),fill='#101923')
        text=['短任务准备就绪','握原来的轻小瓶，无需标记','空格开始；Q / Esc随时结束'] if stage is None else stage['lines']
        for i,line in enumerate(text):d.text((819,470+i*27),line,font=font,fill='white')
        if stage is not None:d.text((819,584),f'本段剩余约 {max(0,stage["end_s"]-elapsed):.0f} 秒',font=font,fill='#b9c2ce')
        return cv2.cvtColor(np.asarray(im),cv2.COLOR_RGB2BGR)
