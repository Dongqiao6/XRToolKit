"""Bounded asynchronous disk sink; capture never waits for video encoding."""
from __future__ import annotations

from collections import Counter, deque
from datetime import datetime, timezone
import json
from pathlib import Path
import queue
import threading
import copy
import pickle

import cv2


class PackedRecords:
    """Completed evidence without a growing GC-traversed Python object graph.

    Only internally produced bytes are decoded; this is not a pickle file reader.
    Active control state stays outside this archive. Public output remains JSONL.
    The optional bound preserves the inference history's existing retention limit.
    """
    def __init__(self, maxlen=None):
        self._rows = deque(maxlen=maxlen)

    def append(self, row):
        self._rows.append(pickle.dumps(row, protocol=5))

    def __len__(self):
        return len(self._rows)

    def __iter__(self):
        return (pickle.loads(row) for row in self._rows)


class PreviewOnly:
    """No filesystem or writer thread; observations exist only in live memory."""
    def __init__(self):
        self.record_video = threading.Event()
        self.drops = Counter()
        self.path = None

    def event(self, row):
        pass

    def frame(self, frame):
        pass

    def check(self):
        pass

    def close(self):
        pass


class Recorder:
    def __init__(self, root, config):
        self.path = Path(root) / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.path.mkdir(parents=True, exist_ok=False)
        self.events = queue.Queue(maxsize=4096)
        self.images = queue.Queue(maxsize=16)
        self.closing = threading.Event()
        self.record_video = threading.Event()
        self.error = None
        self.drops = Counter()
        self.dropped_frames = []
        self.saved_pairs = []
        self.saved_ids = set()
        self.config = copy.deepcopy(config)
        from .calibration_state import snapshot_calibration
        binding = snapshot_calibration(Path(__file__).resolve().parents[1], self.config, self.path)
        self.manifest = {"created_at": datetime.now(timezone.utc).isoformat(), "config": self.config,
                         "calibration_binding": binding,
                         "video_encoding": "MJPG AVI, original camera view, lossy; calibration PNG lossless",
                         "timing": "host arrival proxy, NOT verified exposure synchronization"}
        (self.path / "manifest.json").write_text(json.dumps(self.manifest, indent=2), encoding="utf-8")
        self.thread = threading.Thread(target=self._run, name="session-recorder", daemon=True)
        self.thread.start()

    def event(self, row):
        try:
            self.events.put_nowait(dict(row))
        except queue.Full:
            self.drops["metadata"] += 1

    def frame(self, frame):
        if not self.record_video.is_set():
            return
        try:
            self.images.put_nowait(("video", frame))
        except queue.Full:
            self.drops["video"] += 1
            self.dropped_frames.append(frame.identity)

    def save_pair(self, pair, corners, motion, automatic):
        self.check()
        l, r, info = pair
        if info["pair_id"] in self.saved_ids:
            return False
        index = len(self.saved_ids) + 1
        row = {**info, "type": "calibration_pair", "pair_index": index,
               "left_filename": f"left_{index:06d}.png", "right_filename": f"right_{index:06d}.png",
               "left": l.metadata, "right": r.metadata, "board_motion_px_per_frame": motion,
               "automatic": automatic, "left_corners": corners[0].tolist(), "right_corners": corners[1].tolist()}
        try:
            self.images.put_nowait(("pair", (l, r, row)))
        except queue.Full:
            self.drops["calibration_save_request"] += 1
            return False
        self.saved_ids.add(info["pair_id"])
        return True

    def _run(self):
        videos, video_indices = {}, Counter()
        try:
            with (self.path / "metadata.jsonl").open("w", encoding="utf-8") as metadata:
                while not self.closing.is_set() or not self.events.empty() or not self.images.empty():
                    for _ in range(128):
                        try:
                            row = self.events.get_nowait()
                        except queue.Empty:
                            break
                        metadata.write(json.dumps(row, allow_nan=False) + "\n")
                    try:
                        kind, payload = self.images.get(timeout=0.01)
                    except queue.Empty:
                        continue
                    if kind == "video":
                        frame = payload
                        role = frame.metadata["camera_role"]
                        if role not in videos:
                            height, width = frame.image.shape[:2]
                            videos[role] = cv2.VideoWriter(str(self.path / f"{role}.avi"), cv2.VideoWriter_fourcc(*"MJPG"),
                                                         frame.metadata.get("video_fps", self.config["cameras"]["fps"]), (width, height))
                            if not videos[role].isOpened():
                                raise RuntimeError(f"Cannot open {role} video encoder")
                        videos[role].write(frame.image)
                        depth_file = None
                        if frame.depth_image is not None:
                            folder = self.path / ('depth_'+role)
                            folder.mkdir(exist_ok=True)
                            depth_file = f'depth_{role}/{video_indices[role]:06d}.png'
                            if not cv2.imwrite(str(self.path/depth_file),frame.depth_image):
                                raise RuntimeError('Cannot write lossless depth image')
                        metadata.write(json.dumps({"type": "video_frame", "role": role,
                                                   "depth_filename": depth_file,
                                                   "video_index": video_indices[role], **frame.metadata}) + "\n")
                        video_indices[role] += 1
                    else:
                        l, r, row = payload
                        for frame, key in ((l, "left_filename"), (r, "right_filename")):
                            if not cv2.imwrite(str(self.path / row[key]), frame.image):
                                raise RuntimeError(f"Cannot write {row[key]}")
                        metadata.write(json.dumps(row, allow_nan=False) + "\n")
                        self.saved_pairs.append(row)
                metadata.flush()
        except Exception as exc:
            self.error = str(exc)
        finally:
            for writer in videos.values():
                writer.release()
            self.manifest.update(video_frames=dict(video_indices), drops=dict(self.drops),
                                 dropped_frame_ids=self.dropped_frames, saved_pairs=len(self.saved_pairs),
                                 writer_error=self.error)
            (self.path / "manifest.json").write_text(json.dumps(self.manifest, indent=2), encoding="utf-8")

    def check(self):
        if self.error:
            raise RuntimeError(f"Recording failed: {self.error}")

    def close(self):
        self.record_video.clear()
        self.closing.set()
        self.thread.join(timeout=30)
        if self.thread.is_alive():
            raise RuntimeError("Recording did not finish flushing")
        self.check()
