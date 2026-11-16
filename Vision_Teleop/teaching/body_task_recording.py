"""Both-depth task archive: overlap independent RGB and lossless depth writes.

Keeps the original 16-frame input queue and recorder lifecycle. Exactly one
depth job can be outstanding; commit metadata only after both writes finish.
This explicit task wrapper is not the default recorder or a control component.
"""
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import json
import queue
import time

import cv2
from .recording import Recorder


class BodyTaskRecorder(Recorder):
    def _write_depth(self, frame, role, index):
        if frame.depth_image is None:
            return None  # Preserve an invalid payload; capture wrapper rejects it.
        folder = self.path / ('depth_' + role)
        folder.mkdir(exist_ok=True)
        name = f'depth_{role}/{index:06d}.png'
        if not cv2.imwrite(str(self.path / name), frame.depth_image):
            raise RuntimeError('Cannot write lossless depth image')
        return name

    def _run(self):
        videos, indices = {}, Counter()
        count, total_ns, maximum_ns, queue_high_water = 0, 0, 0, 0
        try:
            with ThreadPoolExecutor(max_workers=1, thread_name_prefix='body-depth-writer') as depth_writer:
                with (self.path / 'metadata.jsonl').open('w', encoding='utf-8') as metadata:
                    while not self.closing.is_set() or not self.events.empty() or not self.images.empty():
                        for _ in range(128):
                            try:
                                row = self.events.get_nowait()
                            except queue.Empty:
                                break
                            metadata.write(json.dumps(row, allow_nan=False) + '\n')
                        queue_high_water = max(queue_high_water, self.images.qsize())
                        try:
                            kind, frame = self.images.get(timeout=0.01)
                        except queue.Empty:
                            continue
                        if kind != 'video':
                            raise RuntimeError('Body-task recorder accepts video only, not calibration requests')
                        started = time.perf_counter_ns()
                        role = frame.metadata['camera_role']
                        if role not in videos:
                            height, width = frame.image.shape[:2]
                            videos[role] = cv2.VideoWriter(
                                str(self.path / f'{role}.avi'), cv2.VideoWriter_fourcc(*'MJPG'),
                                frame.metadata.get('video_fps', self.config['cameras']['fps']), (width, height))
                            if not videos[role].isOpened():
                                raise RuntimeError(f'Cannot open {role} video encoder')
                        # No backlog of depth futures: always join this frame before
                        # consuming the next frame or committing its metadata.
                        future = depth_writer.submit(self._write_depth, frame, role, indices[role])
                        videos[role].write(frame.image)
                        depth_name = future.result()
                        metadata.write(json.dumps(dict(type='video_frame', role=role,
                            depth_filename=depth_name, video_index=indices[role], **frame.metadata), allow_nan=False) + '\n')
                        indices[role] += 1
                        elapsed = time.perf_counter_ns() - started
                        count += 1
                        total_ns += elapsed
                        maximum_ns = max(maximum_ns, elapsed)
                    metadata.flush()
        except Exception as exc:
            self.error = str(exc)
        finally:
            for writer in videos.values():
                writer.release()
            self.manifest.update(video_frames=dict(indices), drops=dict(self.drops),
                dropped_frame_ids=self.dropped_frames, saved_pairs=len(self.saved_pairs), writer_error=self.error,
                body_archive_writer=dict(mode='overlap_rgb_depth_one_frame', input_queue_limit=self.images.maxsize,
                    maximum_outstanding_depth_jobs=1, committed_frames=count,
                    mean_frame_service_ms=total_ns / max(count, 1) / 1e6,
                    max_frame_service_ms=maximum_ns / 1e6,
                    queue_high_water_observed_by_writer=queue_high_water,
                    partial_files_on_failure='Retained; metadata commits require both writes to return successfully'))
            (self.path / 'manifest.json').write_text(json.dumps(self.manifest, indent=2), encoding='utf-8')
