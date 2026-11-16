"""Independent SDK capture, bounded queues, and explicit host-time pairing."""
from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
import threading
import time
from typing import Callable

import numpy as np


@dataclass
class Frame:
    image: np.ndarray
    metadata: dict
    depth_image: np.ndarray | None = None

    @property
    def time_ms(self):
        return self.metadata["host_arrival_monotonic_ns"] / 1e6

    @property
    def identity(self):
        return self.metadata["frame_id"]


class PairBuffer:
    def __init__(self, capacity=8, tolerance_ms=33.3, max_age_ms=150, event: Callable | None = None, clock_ns=time.monotonic_ns):
        if capacity < 1 or tolerance_ms <= 0 or max_age_ms <= 0:
            raise ValueError("Invalid pairing limits")
        self.queues = {role: deque() for role in ("left", "right")}
        self.capacity, self.tolerance, self.max_age = capacity, tolerance_ms, max_age_ms
        self.lock = threading.Lock()
        self.counts = Counter()
        self.clock_ns = clock_ns
        self.event = event or (lambda row: None)
        self.pair_index = 0

    def _finish(self, role, frame, reason, pair_id=None):
        self.counts[f"{role}_{reason}"] += 1
        self.event({"type": "frame_disposition", "frame_id": frame.identity,
                    "role": role, "reason": reason, "pair_id": pair_id,
                    "selected_timestamp_ms": frame.time_ms, "source": "host_arrival_monotonic"})

    def push(self, role, frame):
        with self.lock:
            queue = self.queues[role]
            self.counts[f"{role}_captured"] += 1
            frame.metadata["queue_depth_on_arrival"] = len(queue)
            self.event({"type": "frame", **frame.metadata})
            if len(queue) >= self.capacity:
                self._finish(role, queue.popleft(), "queue_overflow")
            queue.append(frame)

    def pop(self, now_ms=None):
        now_ms = self.clock_ns() / 1e6 if now_ms is None else now_ms
        with self.lock:
            left, right = self.queues["left"], self.queues["right"]
            for role, queue in self.queues.items():
                while queue and now_ms - queue[0].time_ms > self.max_age:
                    self._finish(role, queue.popleft(), "staleness")
            while left and right:
                anchor = left[0]
                # Wait for the right stream to bracket this left sample, except
                # near expiry where the available nearest candidate is final.
                if right[-1].time_ms < anchor.time_ms and now_ms - anchor.time_ms < self.max_age / 2:
                    return None
                best = min(range(len(right)), key=lambda i: abs(right[i].time_ms - anchor.time_ms))
                delta = abs(right[best].time_ms - anchor.time_ms)
                if delta > self.tolerance:
                    role = "left" if left[0].time_ms < right[0].time_ms else "right"
                    self._finish(role, self.queues[role].popleft(), "tolerance")
                    continue
                for _ in range(best):
                    self._finish("right", right.popleft(), "superseded_nearest")
                l, r = left.popleft(), right.popleft()
                self.pair_index += 1
                pair_id = f"pair_{self.pair_index:08d}"
                self._finish("left", l, "matched", pair_id)
                self._finish("right", r, "matched", pair_id)
                info = {"type": "pair", "pair_id": pair_id, "left_frame_id": l.identity,
                        "right_frame_id": r.identity, "timestamp_diff_ms": delta,
                        "pairing_timestamp_type": "host_arrival_monotonic",
                        "raw_frame_delta_ms": r.metadata["frame_timestamp_ms"] - l.metadata["frame_timestamp_ms"],
                        "raw_delta_interpretation": "diagnostic_only_unverified_cross_device_epochs",
                        "pair_age_ms": now_ms - min(l.time_ms, r.time_ms),
                        "exposure_synchronization_verified": False}
                self.event(info)
                return l, r, info
            return None

    def close(self):
        with self.lock:
            for role, queue in self.queues.items():
                while queue:
                    self._finish(role, queue.popleft(), "shutdown")


def enumerate_devices():
    import pyrealsense2 as rs
    rows = []
    for device in rs.context().query_devices():
        row = {}
        for key, field in (("name", "name"), ("serial", "serial_number"),
                           ("firmware", "firmware_version"), ("usb", "usb_type_descriptor")):
            info = getattr(rs.camera_info, field)
            row[key] = device.get_info(info) if device.supports(info) else None
        profiles = set()
        for sensor in device.query_sensors():
            for profile in sensor.get_stream_profiles():
                if profile.stream_type() == rs.stream.color and profile.format() == rs.format.bgr8:
                    video = profile.as_video_stream_profile()
                    profiles.add((video.width(), video.height(), video.fps()))
        row["color_profiles"] = sorted(profiles, reverse=True)
        rows.append(row)
    return rows


def common_color_profiles(left, right):
    return sorted(set(map(tuple, left["color_profiles"])) & set(map(tuple, right["color_profiles"])), reverse=True)


def optional_metadata(frame, enums):
    meta = {}
    for name, enum in (("sensor_timestamp_us", "sensor_timestamp"), ("backend_timestamp_us", "backend_timestamp"),
                       ("exposure_us", "actual_exposure"), ("gain", "gain_level"), ("auto_exposure_enabled", "auto_exposure")):
        field = getattr(enums, enum, None)
        supported = field is not None and frame.supports_frame_metadata(field)
        value = frame.get_frame_metadata(field) if supported else None
        meta[name] = bool(value) if name == "auto_exposure_enabled" and supported else value
        meta[name + "_supported"] = bool(supported)
    # rs_frame.h defines backend_timestamp in microseconds, unlike get_timestamp().
    value = meta["backend_timestamp_us"]
    meta["backend_timestamp_ms"] = None if value is None else value / 1000
    meta["backend_timestamp_ms_supported"] = meta["backend_timestamp_us_supported"]
    return meta


class SDKSource:
    def __init__(self, role, serial, config, clock_ns=time.monotonic_ns):
        self.role, self.serial, self.config = role, serial, config
        self.clock_ns = clock_ns
        self.pipeline = None

    def start(self):
        import pyrealsense2 as rs
        self.rs = rs
        self.depth_enabled = bool(self.config.get('depth',False) and self.role == self.config.get('depth_role','left'))
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(self.serial)
        cfg.enable_stream(rs.stream.color, self.config["width"], self.config["height"], rs.format.bgr8, self.config["fps"])
        if self.depth_enabled:
            cfg.enable_stream(rs.stream.depth,self.config.get('depth_width',640),self.config.get('depth_height',480),rs.format.z16,self.config['fps'])
        profile = pipe.start(cfg)
        self.pipeline = pipe
        device = profile.get_device()
        self.align = rs.align(rs.stream.color) if self.depth_enabled else None
        self.depth_scale = device.first_depth_sensor().get_depth_scale() if self.depth_enabled else None
        self.name = device.get_info(rs.camera_info.name)
        video = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intrinsics = video.get_intrinsics()
        self.profile_metadata = dict(type='camera_profile',role=self.role,serial=self.serial,model=self.name,
            width=video.width(),height=video.height(),fps=video.fps(),format=str(video.format()),
            sdk_intrinsics=dict(fx=intrinsics.fx,fy=intrinsics.fy,ppx=intrinsics.ppx,ppy=intrinsics.ppy,
                                model=str(intrinsics.model),coeffs=list(intrinsics.coeffs)),
            options={},purpose='Startup audit only; SDK intrinsics do not replace the selected stereo calibration')
        sensor = device.first_color_sensor()
        for name in ('enable_auto_exposure','exposure','gain'):
            option = getattr(rs.option,name,None)
            supported = option is not None and sensor.supports(option)
            entry = dict(supported=bool(supported),value=None,source='SDK sensor option at startup',units='SDK native')
            if supported:
                try:
                    entry['value'] = float(sensor.get_option(option))
                except RuntimeError as exc:
                    entry['error'] = str(exc)
            self.profile_metadata['options'][name] = entry

    def read(self):
        rs = self.rs
        ok, frames = self.pipeline.try_wait_for_frames(100)
        if not ok:
            return None
        frame = frames.get_color_frame()
        arrival = self.clock_ns()
        if not frame:
            return None
        meta = {"camera_role": self.role, "camera_serial": self.serial, "camera_model": self.name,
                "frame_number": frame.get_frame_number(), "frame_timestamp_ms": frame.get_timestamp(),
                "host_arrival_monotonic_ns": arrival, "host_clock": self.clock_ns.__name__,
                "timestamp_domain": str(frame.get_frame_timestamp_domain()),
                "width": frame.get_width(), "height": frame.get_height(), "fps": self.config["fps"], "pixel_format": "bgr8"}
        # Identity includes arrival time, so a frame-number reset cannot reuse it.
        meta["frame_id"] = f"{self.role}:{self.serial}:{meta['frame_number']}:{arrival}"
        meta.update(optional_metadata(frame, rs.frame_metadata_value))
        depth_image = None
        meta['depth_enabled'] = self.depth_enabled
        if self.depth_enabled:
            aligned = self.align.process(frames)
            depth = aligned.get_depth_frame()
            if depth:
                intr = depth.profile.as_video_stream_profile().get_intrinsics()
                depth_image = np.asanyarray(depth.get_data()).copy()
                meta['aligned_depth'] = dict(scale_m=self.depth_scale,frame_number=depth.get_frame_number(),
                    timestamp_ms=depth.get_timestamp(),timestamp_domain=str(depth.get_frame_timestamp_domain()),
                    color_depth_delta_ms=float(depth.get_timestamp()-frame.get_timestamp()),
                    width=intr.width,height=intr.height,fx=intr.fx,fy=intr.fy,ppx=intr.ppx,ppy=intr.ppy,
                    distortion=str(intr.model),coeffs=list(intr.coeffs),source='SDK align depth to color',
                    exposure_synchronization_verified=False)
        return Frame(np.asanyarray(frame.get_data()).copy(), meta, depth_image)

    def stop(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None


class DualCapture:
    def __init__(self, config, event=None, source_factory=None, on_frame=None, clock_ns=time.monotonic_ns):
        if not config.get("left_serial") or not config.get("right_serial"):
            raise ValueError("Both left and right serial numbers are required")
        if config["left_serial"] == config["right_serial"]:
            raise ValueError("Left and right serial numbers must differ")
        self.config = config
        self.factory = source_factory or (lambda role, serial, cfg: SDKSource(role, serial, cfg, clock_ns=clock_ns))
        self.event = event or (lambda row: None)
        self.on_frame = on_frame or (lambda frame: None)
        self.buffer = PairBuffer(config["queue_capacity"], config["max_pair_delta_ms"], config["max_age_ms"], self.event, clock_ns=clock_ns)
        self.stop_event = threading.Event()
        self.ready = {r: threading.Event() for r in ("left", "right")}
        self.errors = []
        self.threads = []

    def _run(self, role):
        source = None
        try:
            source = self.factory(role, self.config[f"{role}_serial"], self.config)
            source.start()
            if hasattr(source, 'profile_metadata'):
                self.event(source.profile_metadata)
            self.ready[role].set()
            last_arrival, previous = time.monotonic(), None
            while not self.stop_event.is_set():
                frame = source.read()
                if frame is None:
                    if time.monotonic() - last_arrival > 3:
                        raise RuntimeError("No color frames for three seconds; device may be disconnected")
                    continue
                last_arrival = time.monotonic()
                number = frame.metadata["frame_number"]
                if previous is not None and number != previous + 1:
                    self.event({"type": "frame_number_gap_or_reset", "role": role, "previous": previous, "current": number})
                previous = number
                self.buffer.push(role, frame)
                self.on_frame(frame)
        except Exception as exc:
            self.errors.append(f"{role}: {exc}")
            self.stop_event.set()
        finally:
            try:
                if source is not None:
                    source.stop()
            except Exception as exc:
                self.errors.append(f"{role} stop: {exc}")
            self.ready[role].set()

    def start(self):
        if self.threads:
            raise RuntimeError("Capture objects are single-use")
        for role in self.ready:
            thread = threading.Thread(target=self._run, args=(role,), name=f"capture-{role}", daemon=True)
            self.threads.append(thread)
            thread.start()
        try:
            for ready in self.ready.values():
                if not ready.wait(15):
                    raise RuntimeError("Camera start timed out")
            self.check()
        except BaseException:
            self.stop()
            raise
        return self

    def check(self):
        if self.errors:
            raise RuntimeError("; ".join(self.errors))

    def get_pair(self):
        self.check()
        return self.buffer.pop()

    def stop(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=5)
        alive = [t.name for t in self.threads if t.is_alive()]
        if alive:
            raise RuntimeError(f"SDK shutdown did not complete: {alive}")
        self.buffer.close()

    def __enter__(self):
        return self.start()

    def __exit__(self, *args):
        self.stop()
