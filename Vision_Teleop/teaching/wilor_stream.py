"""Latest-frame bridge to original WiLoR; positions keep independent RGB-D timing."""
from collections import deque
from multiprocessing.connection import Listener
from multiprocessing.shared_memory import SharedMemory
import os
import subprocess
import threading
import time

import numpy as np

from .dual_wilor import fuse_orientation


def right_hand_points(hand):
    points = np.asarray(hand.get('points', []))
    labels = hand.get('handedness', [])
    if points.shape != (1, 21, 2) or len(labels) != 1 or labels[0].get('label') != 'Right':
        return None
    return points[0] if np.isfinite(points).all() else None


def causal_pair(left, right_history, max_delta_ms):
    candidates = [r for r in right_history if 0 <= left.time_ms-r.time_ms <= max_delta_ms]
    return max(candidates, key=lambda r: r.time_ms) if candidates else None


def available_pair(left, right_history, now_ms, max_delta_ms):
    """Nearest pair already available at submission; never use future evidence.

    Unlike recorded-left-time evaluation, live causality is relative to now.
    A slightly later right exposure may already have completed detection.
    """
    if not np.isfinite([left.time_ms,now_ms,max_delta_ms]).all() or left.time_ms>now_ms or max_delta_ms<=0:
        return None
    candidates=[r for r in right_history if np.isfinite(r.time_ms) and
                r.time_ms<=now_ms and abs(left.time_ms-r.time_ms)<=max_delta_ms]
    return min(candidates,key=lambda r:(abs(left.time_ms-r.time_ms),-r.time_ms)) if candidates else None


def observation_fresh(stamp, now, max_age):
    return stamp is not None and 0 <= now-stamp <= max_age


def orientation_stamp(left, right, source):
    if source=='right_rgb': return right.time_ms/1000
    if source=='dual_rgb': return min(left.time_ms,right.time_ms)/1000
    return left.time_ms/1000


class WiLoRStream:
    def __init__(self, root, calibration, settings, output):
        self.calibration, self.settings = calibration, settings
        clock_name = settings.get('timestamp_clock', 'monotonic')
        if clock_name not in ('monotonic', 'perf_counter'):
            raise ValueError('Unsupported WiLoR timestamp clock')
        self.clock = getattr(time, clock_name)
        self.lock = threading.Lock()
        self.frames = {r: deque(maxlen=8) for r in ('left', 'right')}
        self.latest = None
        self.stop_event = threading.Event()
        self.input_ready = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.metrics = deque(maxlen=100000)
        self.key = os.urandom(32)
        self.listener = Listener(('127.0.0.1', 0), authkey=self.key)
        self.conn = None
        self.memory = {k: SharedMemory(create=True, size=1280*720*3) for k in ('left','right')}
        self.log = (output/'gpu.stderr.log').open('w')
        self.proc = subprocess.Popen([str(root/'.venv-wilor/Scripts/python.exe'),
                                      str(root/getattr(self,'worker_script','scripts/wilor_stream_worker.py')),
                                      '--port', str(self.listener.address[1]), '--key', self.key.hex(),
                                      '--matmul-precision', settings.get('matmul_precision', 'highest'),
                                      '--crop-blur', settings.get('crop_blur', 'upstream'),
                                      '--timestamp-clock', clock_name],
                                     cwd=root, stdout=self.log, stderr=self.log,
                                     creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
        self.thread = threading.Thread(target=self._run, name='wilor-ipc', daemon=True)
        self.thread.start()

    def submit(self, role, frame):
        with self.lock:
            self.frames[role].append(frame)
            self.input_ready.set()

    def snapshot(self):
        with self.lock:
            return {k: v[-1] if v else None for k, v in self.frames.items()}, self.latest

    def _run(self):
        try:
            self.conn = self.listener.accept()
            self.info = self.conn.recv()
            self.ready.set()
            previous = None
            while not self.stop_event.is_set():
                with self.lock:
                    self.input_ready.clear()
                    selection_started = self.clock()
                    left = self.frames['left'][-1] if self.frames['left'] else None
                    available = self.settings.get('pair_policy') == 'nearest_already_available'
                    right = (available_pair(left,self.frames['right'],self.clock()*1000,self.settings['max_pair_delta_ms'])
                             if available else causal_pair(left,self.frames['right'],self.settings['max_pair_delta_ms'])) if left else None
                if left is None or left.identity == previous:
                    self.input_ready.wait(.1)
                    continue
                previous = left.identity
                selected = self.clock()
                request = []
                for role, frame in [('left', left), ('right', right)]:
                    points = right_hand_points(frame.metadata['hand']) if frame else None
                    if points is not None:
                        if frame.image.shape != (720,1280,3) or frame.image.dtype != np.uint8:
                            raise ValueError('WiLoR shared input requires the fixed 1280x720 BGR stream')
                        np.ndarray(frame.image.shape,dtype=np.uint8,buffer=self.memory[role].buf)[:] = frame.image
                        request.append(dict(role=role, memory=self.memory[role].name, shape=frame.image.shape, points=points))
                start = time.perf_counter()
                submitted = self.clock()
                self.conn.send(request)
                result = self.conn.recv()
                now = self.clock()
                predictions = result['predictions']
                age = left.time_ms-right.time_ms if right else None
                # The legacy fuser uses nonnegative pair separation. In live available
                # mode both inputs passed the explicit <= submission-time check above;
                # keep signed delta in the evidence and use its magnitude for compatibility.
                fused = fuse_orientation(predictions.get('left'), predictions.get('right'),
                                         abs(age) if available and age is not None else age,
                                         self.calibration['R'], self.settings)
                source_stamp = orientation_stamp(left,right,fused['source'])
                row = dict(**fused, stamp=source_stamp, completed=now,
                           left_frame=left, right_frame=right, predictions=predictions,
                           inference_ms=result['inference_ms'], batch_size=result['batch_size'])
                with self.lock:
                    self.latest = row
                self.metrics.append(dict(inference_ms=result['inference_ms'], ipc_roundtrip_ms=(time.perf_counter()-start)*1000,
                                         model_ms=result['model_ms'], box_sizes=result['box_sizes'],
                                         source_stamp_s=source_stamp, completed_s=now,
                                         left_frame_id=left.identity, right_frame_id=right.identity if right else None,
                                         pair_delta_ms=age, reason=fused['reason'],
                                         rotation=fused['rotation'],
                                         left_tracking_ms=left.metadata['hand']['inference_ms'],
                                         wait_breakdown=wait_breakdown(left,right,source_stamp,selected,submitted,selection_started),
                                         per_view_rotations={k:v['rotation'] for k,v in predictions.items()},
                                         playback_wait_backend=left.metadata.get('playback_wait_backend'),
                                         left_arrival_to_submit_ms=(submitted-left.time_ms/1000)*1000,
                                         prepare_ms=result['prepare_ms'], arrival_to_submit_ms=(submitted-source_stamp)*1000,
                                         arrival_to_result_ms=(now-source_stamp)*1000,
                                         batch_size=result['batch_size'], source=fused['source'], valid=fused['valid']))
        except Exception as exc:
            if not self.stop_event.is_set():
                self.error = str(exc)
        finally:
            self.ready.set()

    def check(self):
        if self.error:
            raise RuntimeError('WiLoR worker: '+self.error)
        if self.proc.poll() is not None and not self.stop_event.is_set():
            raise RuntimeError('WiLoR process exited; see gpu.stderr.log')

    def close(self):
        self.stop_event.set()
        self.input_ready.set()
        # Closing our dedicated child also releases any blocked IPC receive.
        if self.proc.poll() is None:
            self.proc.terminate()
            self.proc.wait(timeout=10)
        if self.conn:
            self.conn.close()
        self.listener.close()
        self.thread.join(timeout=3)
        for memory in self.memory.values():
            memory.close()
            memory.unlink()
        self.log.close()


def wait_breakdown(left, right, source_stamp, selected, submitted,selection_started=None):
    """Additive timing for the left-anchored submission, in one clock domain.

    Pair offset is older evidence age, not blocking time in the pair lookup.
    Selected-frame readiness wait includes time while the GPU processes its predecessor.
    """
    meta=left.metadata; hand=meta['hand']
    arrival=left.time_ms/1000
    queued=meta['tracking_submitted_ns']/1e9
    started=hand['inference_started_ns']/1e9
    done=hand['inference_done_monotonic_ns']/1e9
    selection_started=selected if selection_started is None else selection_started
    dispatch=meta.get('playback_dispatch_monotonic_ns',meta['host_arrival_monotonic_ns'])/1e9
    return dict(source_pair_offset_ms=(arrival-source_stamp)*1000,
        capture_dispatch_ms=(dispatch-arrival)*1000,
        dispatch_to_detector_queue_ms=(queued-dispatch)*1000,
        detector_queue_ms=(started-queued)*1000,
        detector_ms=hand['detector_ms'],selection_depth_ms=hand['selection_depth_ms'],
        detected_frame_wait_ms=(selection_started-done)*1000,
        pair_lookup_ms=(selected-selection_started)*1000,
        input_copy_ms=(submitted-selected)*1000,
        total_ms=(submitted-source_stamp)*1000)
