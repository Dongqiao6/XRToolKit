"""Independent upstream VIDEO trackers with bounded, replace-old work slots."""
import hashlib
import threading
import time

import cv2
import numpy as np

from .capture import Frame


class VideoHand:
    def __init__(self, model):
        import mediapipe as mp
        from mediapipe.tasks.python import vision
        if hashlib.sha256(model.read_bytes()).hexdigest() != "fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1":
            raise ValueError("Unexpected hand model checksum")
        self.mp = mp
        self.model = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model)),
            running_mode=vision.RunningMode.VIDEO, num_hands=2))
        self.last_ms = -1
        self.last_source_ms = -1

    def tracker_time(self, source_ms):
        if source_ms < self.last_source_ms:
            raise ValueError("Host arrival clock moved backwards")
        self.last_source_ms = source_ms
        # MediaPipe requires strict millisecond order. This is an API clock,
        # never the timestamp used for stereo pairing or physical diagnostics.
        self.last_ms = max(int(source_ms), self.last_ms+1)
        return self.last_ms

    def __call__(self, frame):
        timestamp = self.tracker_time(frame.time_ms)
        rgb = np.ascontiguousarray(cv2.cvtColor(frame.image, cv2.COLOR_BGR2RGB))
        result = self.model.detect_for_video(self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb), timestamp)
        h, w = frame.image.shape[:2]
        return dict(points=[[[p.x*w, p.y*h] for p in hand] for hand in result.hand_landmarks],
                    world_points_m=[[[p.x, p.y, p.z] for p in hand] for hand in result.hand_world_landmarks],
                    world_points_source="mediapipe_prediction_not_measured_depth",
                    handedness=[dict(label=h[0].category_name, score=h[0].score) for h in result.handedness],
                    per_point_confidence_supported=False, per_point_scores=None,
                    handedness_verified=False, tracker_timestamp_ms=timestamp, tracker_mode="VIDEO",
                    tracker_source_host_ms=frame.time_ms, tracker_time_adjustment_ms=timestamp-int(frame.time_ms),
                    tracker_time_purpose="API ordering only; not pairing or exposure time")

    def close(self):
        self.model.close()


def select_right_hand(observation):
    """Select a unique labelled right hand before both depth sampling and WiLoR.

    This is a single-demonstrator interface, not person identity tracking.
    Keep ambiguous right hands invalid; never reinterpret a left label.
    """
    result = dict(observation)
    labels = observation.get('handedness', [])
    points = np.asarray(observation.get('points', []))
    count = len(points) if points.ndim else 0
    candidates = [i for i, label in enumerate(labels) if label.get('label') == 'Right']
    reason = ('no_hand' if count == 0 else
              'malformed_hand' if points.shape != (len(labels), 21, 2) else
              'no_right_hand' if not candidates else
              'ambiguous_right_hands' if len(candidates) != 1 else
              'nonfinite_hand' if not np.isfinite(points[candidates[0]]).all() else 'selected')
    index = candidates[0] if reason == 'selected' else None
    result['selection'] = dict(reason=reason, detected_count=count, labels=labels, index=index)
    for key in ('points', 'world_points_m', 'handedness'):
        values = observation.get(key, [])
        result[key] = [values[index]] if index is not None and len(values) == count else []
    # A depth result from an unselected observation cannot be reused.
    result.pop('rgbd_position', None)
    return result


class TrackingWorker:
    def __init__(self, role, factory, output, event, rgbd_settings=None, selector=None, clock_ns=time.monotonic_ns):
        self.role, self.factory, self.output, self.event = role, factory, output, event
        self.rgbd_settings = rgbd_settings
        self.selector = selector
        self.clock_ns = clock_ns
        self.condition = threading.Condition()
        self.pending = None
        self.closing = False
        self.error = None
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self._run, name=f"hand-{role}", daemon=True)
        self.thread.start()

    def submit(self, frame):
        with self.condition:
            if self.closing:
                raise RuntimeError("Tracker already closed")
            if self.pending is not None:
                self.event(dict(type="inference_skipped", frame_id=self.pending.identity, reason="newer_frame_waiting"))
            self.pending = Frame(frame.image, {**frame.metadata,
                'tracking_submitted_ns':self.clock_ns()},frame.depth_image)
            self.condition.notify()

    def _run(self):
        model = None
        try:
            model = self.factory()
            self.ready.set()
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.pending is not None or self.closing)
                    if self.closing:
                        if self.pending is not None:
                            self.event(dict(type="inference_skipped", frame_id=self.pending.identity, reason="shutdown"))
                        self.pending = None
                        break
                    frame, self.pending = self.pending, None
                start = time.perf_counter_ns()
                started = self.clock_ns()
                observation = model(frame)
                detector_done = self.clock_ns()
                if self.selector is not None:
                    observation = self.selector(observation)
                if self.rgbd_settings is not None:
                    from .rgbd_position import wrist_position
                    observation['rgbd_position'] = wrist_position(frame,observation,self.rgbd_settings)
                done = self.clock_ns()
                observation.update(inference_ms=(time.perf_counter_ns()-start)/1e6, inference_done_monotonic_ns=done,
                    detector_ms=(detector_done-started)/1e6, selection_depth_ms=(done-detector_done)/1e6,
                    inference_started_ns=started)
                self.event(dict(type="hand_2d", frame_id=frame.identity, role=self.role, **observation))
                self.output(self.role, Frame(frame.image, {**frame.metadata, "hand": observation},frame.depth_image))
        except BaseException as exc:
            self.error = str(exc)
        finally:
            try:
                if model is not None:
                    model.close()
            except Exception as exc:
                self.error = str(exc)
            self.ready.set()

    def check(self):
        if self.error is not None:
            raise RuntimeError(f"{self.role} tracker: {self.error}")

    def close(self):
        with self.condition:
            self.closing = True
            self.condition.notify()
        self.thread.join(timeout=10)
        if self.thread.is_alive():
            raise RuntimeError(f"{self.role} tracker failed to stop")
        self.check()
