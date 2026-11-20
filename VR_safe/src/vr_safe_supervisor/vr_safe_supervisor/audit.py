"""JSONL 安全事件审计。只追加，不阻塞控制回路。"""

import json
import os
import threading
import time


class AuditLog:
    def __init__(self, path: str):
        self.path = os.path.abspath(path)
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        self._lock = threading.Lock()
        self._f = open(self.path, "a", buffering=1)

    def event(self, kind: str, **fields):
        rec = {"ts": time.time(), "kind": kind}
        rec.update(fields)
        line = json.dumps(rec, ensure_ascii=False, default=str)
        with self._lock:
            if not self._f.closed:
                self._f.write(line + "\n")

    def close(self):
        with self._lock:
            if not self._f.closed:
                self._f.close()
