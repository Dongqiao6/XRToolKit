"""User-triggered composite video; bounded async encoding with wall-time holds."""
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import time

import imageio_ffmpeg


class TeleopVideo:
    def __init__(self, root, settings, fps=20):
        self.path=Path(root)/datetime.now().strftime('clip_%Y%m%d_%H%M%S_%f')
        self.path.mkdir(parents=True)
        self.fps=fps
        self.start=time.perf_counter()
        self.end=None
        self.queue=queue.Queue(maxsize=4)
        self.stopping=threading.Event()
        self.error=None
        self.drops=0
        self.count=0
        self.repeated=0
        self.settings=settings
        self.thread=threading.Thread(target=self._write,daemon=True)
        self.thread.start()

    def submit(self,image,metadata):
        if self.error: raise RuntimeError(self.error)
        if self.stopping.is_set(): return
        try: self.queue.put_nowait((time.perf_counter()-self.start,image.copy(),metadata))
        except queue.Full: self.drops+=1

    def _write(self):
        proc=None
        previous=None
        try:
            with (self.path/'frames.jsonl').open('w') as log:
                while not self.stopping.is_set() or not self.queue.empty():
                    try: stamp,image,metadata=self.queue.get(timeout=.05)
                    except queue.Empty: continue
                    if proc is None:
                        h,w=image.shape[:2]
                        proc=subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-loglevel','error','-f','rawvideo','-pixel_format','bgr24',
                            '-video_size',f'{w}x{h}','-framerate',str(self.fps),'-i','-','-an','-c:v','libx264','-preset','ultrafast','-crf','22','-pix_fmt','yuv420p',str(self.path/'teleoperation.mp4')],
                            stdin=subprocess.PIPE,stderr=subprocess.PIPE,creationflags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0)
                    index=int(stamp*self.fps)
                    while self.count<index:
                        proc.stdin.write((previous if previous is not None else image).tobytes())
                        self.count+=1
                        self.repeated+=1
                    if self.count==index:
                        proc.stdin.write(image.tobytes()); self.count+=1
                    previous=image
                    log.write(json.dumps(dict(elapsed_s=stamp,video_index=index,**metadata))+'\n')
                if proc and previous is not None:
                    while self.count<int((self.end-self.start)*self.fps):
                        proc.stdin.write(previous.tobytes());self.count+=1;self.repeated+=1
        except Exception as exc: self.error=str(exc)
        finally:
            if proc:
                try:
                    proc.stdin.close()
                    if proc.wait(timeout=15)!=0: self.error=proc.stderr.read().decode(errors='replace')
                except Exception as exc: self.error=str(exc);proc.kill()
            (self.path/'manifest.json').write_text(json.dumps(dict(settings=self.settings,fps=self.fps,frames=self.count,held_video_frames=self.repeated,
                queue_drops=self.drops,error=self.error,duration_wall_s=(self.end or time.perf_counter())-self.start,
                limits='User-triggered presentation video, not raw synchronized acquisition. Repeated video frames preserve wall time; metadata records actual render submissions. No inferred frames treated as observations.'),indent=2))

    def close(self):
        self.end=time.perf_counter()
        self.stopping.set()
        self.thread.join(timeout=20)
        if self.thread.is_alive(): raise RuntimeError('Video writer did not finish')
        if self.error: raise RuntimeError('Video recording: '+self.error)
