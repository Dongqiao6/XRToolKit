"""Isolated stream variant: exact-input reuse and explicit publication timing."""
import time
import numpy as np
from .wilor_stream import WiLoRStream,right_hand_points,available_pair,orientation_stamp,wait_breakdown
from .dual_wilor import fuse_orientation

def prediction_key(frame):
    points=right_hand_points(frame.metadata['hand']) if frame else None
    return None if points is None else (frame.identity,frame.time_ms,np.asarray(points,dtype=float).tobytes())

class CandidateStream(WiLoRStream):
    def _run(self):
        try:
            self.conn=self.listener.accept();self.info=self.conn.recv();self.ready.set()
            previous=None;cache={}
            while not self.stop_event.is_set():
                with self.lock:
                    self.input_ready.clear();selection_started=self.clock()
                    left=self.frames['left'][-1] if self.frames['left'] else None
                    right=available_pair(left,self.frames['right'],self.clock()*1000,self.settings['max_pair_delta_ms']) if left else None
                if left is None or left.identity==previous:
                    self.input_ready.wait(.1);continue
                previous=left.identity;selected=self.clock();request=[];predictions={};reused=[];keys={}
                for role,frame in [('left',left),('right',right)]:
                    key=prediction_key(frame);keys[role]=key
                    if key is None:continue
                    if role in cache and key==cache[role][0]:
                        predictions[role]=cache[role][1];reused.append(role);continue
                    if frame.image.shape!=(720,1280,3) or frame.image.dtype!=np.uint8:raise ValueError('Fixed image format required')
                    np.ndarray(frame.image.shape,dtype=np.uint8,buffer=self.memory[role].buf)[:]=frame.image
                    request.append(dict(role=role,memory=self.memory[role].name,shape=frame.image.shape,points=right_hand_points(frame.metadata['hand'])))
                start=time.perf_counter();submitted=self.clock()
                if request:
                    self.conn.send(request);result=self.conn.recv()
                else:result=dict(predictions={},inference_ms=0.,batch_size=0,model_ms=0.,box_sizes=[],prepare_ms=0.)
                now=self.clock();predictions.update(result['predictions'])
                for role,pred in result['predictions'].items():cache[role]=(keys[role],pred)
                age=left.time_ms-right.time_ms if right else None
                fused=fuse_orientation(predictions.get('left'),predictions.get('right'),abs(age) if age is not None else None,self.calibration['R'],self.settings)
                stamp=orientation_stamp(left,right,fused['source'])
                row=dict(**fused,stamp=stamp,completed=now,left_frame=left,right_frame=right,predictions=predictions,inference_ms=result['inference_ms'],batch_size=result['batch_size'])
                with self.lock:
                    row['published']=self.clock();self.latest=row
                self.metrics.append(dict(inference_ms=result['inference_ms'],ipc_roundtrip_ms=(time.perf_counter()-start)*1000,model_ms=result['model_ms'],box_sizes=result['box_sizes'],
                    source_stamp_s=stamp,completed_s=now,published_s=row['published'],left_frame_id=left.identity,right_frame_id=right.identity if right else None,pair_delta_ms=age,reason=fused['reason'],rotation=fused['rotation'],
                    left_tracking_ms=left.metadata['hand']['inference_ms'],wait_breakdown=wait_breakdown(left,right,stamp,selected,submitted,selection_started),per_view_rotations={k:v['rotation'] for k,v in predictions.items()},
                    left_arrival_to_submit_ms=(submitted-left.time_ms/1000)*1000,prepare_ms=result['prepare_ms'],arrival_to_submit_ms=(submitted-stamp)*1000,arrival_to_result_ms=(now-stamp)*1000,
                    batch_size=result['batch_size'],source=fused['source'],valid=fused['valid'],reused_roles=reused,submitted_roles=[x['role'] for x in request],reuse_preserves_source_stamp=True))
        except Exception as exc:
            if not self.stop_event.is_set():self.error=str(exc)
        finally:self.ready.set()
