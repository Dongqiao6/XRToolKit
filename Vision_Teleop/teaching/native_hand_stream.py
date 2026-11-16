"""Raw dual RGB to fixed WiLoR wrist/depth and orientation, one publication."""
import time
import numpy as np
from .capture import Frame
from .wilor_stream import WiLoRStream,available_pair,orientation_stamp
from .dual_wilor import fuse_orientation
from .rgbd_position import wrist_position
from .native_hand_input import hand_observation
from .recording import PackedRecords

class NativeHandStream(WiLoRStream):
    worker_script='scripts/native_hand_worker.py'

    def __init__(self,root,calibration,settings,output,depth_settings,completed):
        self.processed={'left':None,'right':None};self.depth_settings=depth_settings;self.completed=completed
        super().__init__(root,calibration,settings,output)
        # Capture cannot submit until construction returns. Preserve the original
        # inference history bound; only completed evidence changes representation.
        assert not self.metrics
        self.metrics=PackedRecords(maxlen=100000)

    def snapshot(self):
        with self.lock:return dict(self.processed),self.latest

    def _run(self):
        try:
            self.conn=self.listener.accept();self.info=self.conn.recv();self.ready.set();previous=None;last_number=None
            stride=self.info.get('frame_stride',1)
            if stride not in (1,2):raise ValueError('Unsupported input frame cadence')
            while not self.stop_event.is_set():
                with self.lock:
                    self.input_ready.clear();left=self.frames['left'][-1] if self.frames['left'] else None
                    right=available_pair(left,self.frames['right'],self.clock()*1000,self.settings['max_pair_delta_ms']) if left else None
                if stride==2 and left is not None:
                    number=left.metadata['frame_number']
                    if left.metadata['fps']!=30 or not isinstance(number,int):raise ValueError('Fixed30Hz SDK frame counter required')
                    if last_number is not None and number<last_number:raise ValueError('SDK frame counter reversed')
                if (left is None or left.identity==previous or
                    (stride==2 and last_number is not None and number-last_number<2)):
                    self.input_ready.wait(.1);continue
                if stride==2:last_number=number
                previous=left.identity;chosen={'left':left,'right':right};request=[]
                for role,frame in chosen.items():
                    if frame is None:continue
                    if frame.image.shape!=(720,1280,3) or frame.image.dtype!=np.uint8:raise ValueError('Frozen raw image format required')
                    np.ndarray(frame.image.shape,dtype=np.uint8,buffer=self.memory[role].buf)[:]=frame.image
                    request.append(dict(role=role,memory=self.memory[role].name,shape=frame.image.shape))
                submitted=self.clock();self.conn.send(request);result=self.conn.recv();inferred=self.clock();processed={};predictions={}
                for role,frame in chosen.items():
                    if frame is None:processed[role]=None;continue
                    pred=result['predictions'].get(role);hand=hand_observation(pred,result['selections'][role])
                    if role=='left':hand['rgbd_position']=wrist_position(frame,hand,self.depth_settings)
                    hand.update(inference_ms=result['inference_ms'],detector_ms=result['detector_ms'][role],inference_started_ns=int(submitted*1e9),inference_done_monotonic_ns=int(inferred*1e9))
                    processed[role]=Frame(frame.image,{**frame.metadata,'hand':hand},frame.depth_image)
                    if hand['selection']['reason']=='selected':predictions[role]=pred
                delta=left.time_ms-right.time_ms if right else None
                fused=fuse_orientation(predictions.get('left'),predictions.get('right'),abs(delta) if delta is not None else None,self.calibration['R'],self.settings)
                stamp=orientation_stamp(left,right,fused['source']);published=self.clock()
                row=dict(**fused,stamp=stamp,completed=inferred,published=published,left_frame=processed['left'],right_frame=processed['right'],predictions=predictions,inference_ms=result['inference_ms'],batch_size=result['batch_size'])
                with self.lock:self.processed=processed;self.latest=row
                for role,frame in processed.items():
                    if frame is not None:self.completed(role,frame)
                self.metrics.append(dict(source_stamp_s=stamp,completed_s=inferred,published_s=published,left_frame_id=left.identity,right_frame_id=right.identity if right else None,
                    pair_delta_ms=delta,source=fused['source'],reason=fused['reason'],valid=fused['valid'],rotation=fused['rotation'],disagreement_deg=fused.get('disagreement_deg'),
                    per_view_rotations={k:v['rotation'] for k,v in predictions.items()},selections=result['selections'],detector_ms=result['detector_ms'],inference_ms=result['inference_ms'],
                    detector_batch_ms=result['detector_batch_ms'],model_ms=result['model_ms'],prepare_ms=result['prepare_ms'],batch_size=result['batch_size'],arrival_to_result_ms=(published-stamp)*1000,
                    source_definition='official_detector_wilor_projected_wrist',position_frame_id=left.identity,position_valid=processed['left'].metadata['hand']['rgbd_position']['valid']))
        except Exception as exc:
            if not self.stop_event.is_set():self.error=str(exc)
        finally:self.ready.set()
